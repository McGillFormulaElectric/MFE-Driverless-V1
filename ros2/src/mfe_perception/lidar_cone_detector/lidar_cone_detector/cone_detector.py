#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from mfe_msgs.msg import Cone, Track
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs_py import point_cloud2

from sklearn.cluster import DBSCAN

class LiDARConeNode(Node):
    def __init__(self):

        super().__init__("cone_detector_node")
        self.init_params()

        self.get_logger().info("Cone detector node initialized and spinning...")

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.create_subscription(PointCloud2, "pcl/objects", self.callback, qos_profile)
        self.point_cloud_publisher = self.create_publisher(PointCloud2, "pcl/objects2", qos_profile)
        self.cone_publisher = self.create_publisher(Cone, "pcl/cones", qos_profile)
        self.track_publisher = self.create_publisher(Track, "pcl/track", qos_profile)
        self.centres_cloud_publisher = self.create_publisher(PointCloud2, "pcl/cone_centres", qos_profile)


    def init_params(self):
        """Load DBSCAN hyperparameters from config."""
        self.declare_parameter("dbscan_cluster_min_samples", value=3)
        self.cluster_min_samples = self.get_parameter("dbscan_cluster_min_samples").value

        self.declare_parameter("dbscan_epsilon", value=0.5)
        self.epsilon = self.get_parameter("dbscan_epsilon").value

        return
    
    
    def create_cone_msg(self, x: float, y: float, z: float, cone_color: int):
        """Create a Cone message from (x,y,z) and color enum."""
        if (cone_color not in [0, 1, 2, 3, 4]):
            raise Exception("MessageCreationException")

        location = Point()
        location.x = x
        location.y = y
        location.z = z
        return Cone(location=location, color=cone_color)
    

    def find_clusters(self, points):
        """Return (cluster_list, centres) from DBSCAN on points (Nx3)."""
        if points is None or len(points) == 0:
            return [], np.empty((0, 3))

        pts = np.asarray(points)
        if pts.ndim != 2 or pts.shape[1] != 3:
            self.get_logger().warn("Unexpected point shape for clustering")
            return [], np.empty((0, 3))

        db = DBSCAN(eps=float(self.epsilon), min_samples=int(self.cluster_min_samples),
                    metric="euclidean", n_jobs=2).fit(pts)
        labels = db.labels_

        unique_labels = [lab for lab in np.unique(labels) if lab != -1]
        objects = []
        centres = []

        for lab in unique_labels:
            cluster_pts = pts[labels == lab]
            if cluster_pts.size == 0:
                continue

            objects.append(cluster_pts)
            centres.append(np.mean(cluster_pts, axis=0))

        if len(centres) == 0:
            return objects, np.empty((0, 3))

        return objects, np.vstack(centres)

    

    def filter_cones(self, objects, object_centres):
        """Filter clusters by size and spatial extent."""
        if len(objects) == 0:
            return [], np.empty((0, 3))

        if isinstance(object_centres, list):
            object_centres = np.array(object_centres)

        mask = np.zeros(len(objects), dtype=bool)

        for i, cluster in enumerate(objects):
            if cluster.shape[0] < 5 or cluster.size > 70:
                continue

            dists = np.linalg.norm(cluster - object_centres[i][:3], axis=1)
            if np.max(dists) > 150:
                continue

            mask[i] = True

        # Extract filtered clusters
        cone_clusters = [objects[i] for i in range(len(objects)) if mask[i]]

        # Extract filtered centroids
        cone_locations = object_centres[mask] if object_centres.size > 0 else np.empty((0, 3))

        return cone_clusters, cone_locations
        

    def make_pointcloud2_from_xyz(self, points: np.ndarray, frame_id: str, stamp):
        """
        points: (N,3) float32 numpy array
        stamp: builtin_interfaces.msg.Time or compatible (use incoming msg.header.stamp)
        """
        if points is None or points.size == 0:
            return None

        points_list = [tuple(p) for p in points]  # Nx3 -> list of tuples
        pc2_msg = point_cloud2.create_cloud_xyz32(
            header=Header(stamp=stamp, frame_id=frame_id),
            points=points_list,
        )

        return pc2_msg


    def callback(self, msg: PointCloud2):
        """
        Called when point cloud data is received.
        Runs clustering, finds the clusters that are cones, and returns the centre location
        """  

        try:
            self.get_logger().info(f"Received point cloud of width {msg.width} height {msg.height}")

            # Convert PointCloud2 to a numpy arr (Nx3)
            try:
                arr = rnp.numpify(msg)

                if 'xyz' in arr:
                    points = arr['xyz']
                elif arr.dtype.names and all(n in arr.dtype.names for n in ('x','y','z')):
                    points = np.vstack((arr['x'], arr['y'], arr['z'])).T
                else:
                    self.get_logger().warn("PointCloud2 structure unexpected; aborting callback")
                    return
            except Exception as e:
                self.get_logger().error(f"Failed to convert PointCloud2 to numpy: {e}")
                return

            # Remove NaNs
            points = points[~np.isnan(points).any(axis=1)]

            if points.size == 0 or len(points) == 0:
                self.get_logger().warn("No valid points received")
                return

            # FILTER: Remove points that are outside of lidar range or min range
            # point_norms = np.linalg.norm(points, axis=1)  # axis=1 -> row-wise norm
            # TODO: tune better range values, these were chosen visually
            # mask = (point_norms < 8) & (point_norms > 2)  # keep points within range
            # points = points[mask]
            # NO FILTERING FOR NOW - PARAMETERS MUST BE TUNED

            # run cluster detection
            objects, object_centres = self.find_clusters(points)

            if object_centres.size == 0:
                self.get_logger().debug("No clusters found")
                return

            # FILTER: removes the non-cone clusters
            # OPTIONAL: further filter clusters to cones
            cone_clusters, cone_locations = self.filter_cones(objects, object_centres)
            # cone_clusters, cone_locations = objects, object_centres # no filtering for now for dev, parameters must be tuned

            if cone_locations.size != 0:
                centres_msg = self.make_pointcloud2_from_xyz(np.asarray(cone_locations), msg.header.frame_id, msg.header.stamp)
                
            if len(cone_locations) == 0:
                return

            # TODO: Draw cylinders from the raw PointCloud and bring extra points back
            # Create cone message types and chain them into an array 
            # detected_cones: list = [self.create_cone_msg(cone[0], cone[1], cone[2]) for cone in cone_locations]

            track_msg = Track()
            track = []

            # Publish each cone individually, create Track message
            for centre in cone_locations:
                # TODO: determine cone color/type
                cone_msg = self.create_cone_msg(float(centre[0]), float(centre[1]), float(centre[2]), 0)

                # if Cone type needs header/frame, add that here (depends on message definition)
                # cone_msg_header = Header()
                # cone_msg_header.stamp = self.get_clock().now().to_msg()
                # cone_msg_header.frame_id = msg.header.frame_id
                # cone_msg = Cone(header=cone_msg_header, cones=rnp.msgify())

                track.append(cone_msg)

                self.get_logger().info(f"Publishing {cone_locations.shape[0]} centres to pcl/cone_centres")
                self.cone_publisher.publish(cone_msg)

            # publish locs
            track_msg.track = track
            self.track_publisher.publish(track_msg)

            try:
                # Publish pointcloud of cone cluster points (all points that belong to cone clusters)
                if len(cone_clusters) > 0:
                    all_cluster_points = np.vstack(cone_clusters)
                    self.get_logger().info(f"Publishing {all_cluster_points.shape[0]} points to pcl/objects2")
                    pc_clusters_msg = self.make_pointcloud2_from_xyz(all_cluster_points, msg.header.frame_id, msg.header.stamp)
                    
                    if pc_clusters_msg is not None:
                        self.point_cloud_publisher.publish(pc_clusters_msg)

                # Publish pointcloud of cone centres (one point per cone)
                if cone_locations.size != 0:
                    centres_msg = self.make_pointcloud2_from_xyz(np.asarray(cone_locations), msg.header.frame_id, msg.header.stamp)
                    if centres_msg is not None:
                        self.centres_cloud_publisher.publish(centres_msg)

            except Exception as e:
                self.get_logger().error(f"Failed to create/publish cone pointclouds: {e}")

        except Exception as e:
            self.get_logger().error(f"Exception in callback: {e}")

        return
