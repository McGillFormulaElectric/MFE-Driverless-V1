import rclpy
from rclpy.node import Node

import numpy as np
import array
import time

from geometry_msgs.msg import Point
from mfe_msgs import Cone
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp

from sklearn.cluster import DBSCAN


class LiDARConeNode(Node):

    def __init__(self):
        super().__init__("lidar_cone_node")

        self.init_params()

        # subscribe to the outputs of the ground removal script
        # should be the pointcloud with the ground removed
        self.create_subscription(PointCloud2, "lidar/pcl/cones", self.callback, 10) # Queue size buffer placeholder

        # publishers - probably point cloud for debugging, then cones
        #self.cone_publisher = self.create_publisher(ConeDetectionStamped, "lidar/cone_detection", 1)
        #self.point_cloud_publisher = self.create_publisher(PointCloud2, "lidar/cone_points", 10)
        self.cone_publusher = self.create_publisher(Cone, "lidar/pcl/coords", 10) # buffer placeholder

    
    def init_params(self):
        # init launch params in here

        return
    
    
    def create_cone_msg(self, x, y, z):
        """
        Creates a message of type Cone given a location

        """

        location: Point = Point(x, y, z)

        return Cone(location=location, color=Cone.UNKNOWN) # TODO: colour?
    

    def find_clusters(self, points):
        """
        Applies DBSCAN clustering algorithm to an np array of points in the point cloud received
        """

        # apply DBSCAN clustering
        clusters = DBSCAN(eps=0.8, min_samples=2).fit(points) # params to be changed based on cone specs
        labels = clusters.labels_

        # gets clusters
        unq_labels = np.unique(labels)

        # creates arrays for objects and their centres
        objects = np.empty(unq_labels.size, dtype=object)
        object_centres = np.empty((unq_labels.size, 3))

        # iterates over unique clusters,
        for idx, label in enumerate(unq_labels):
            objects[idx] = object_centres[np.where(labels == label)] # extract the points in that cluster
            
            # compute the centre
            object_centres[idx] = np.mean(
                np.column_stack((objects[idx]["x"], objects[idx]["y"], objects[idx]["z"])), axis=0
            )

        return object_centres, objects
    
    def filter_cones(self, objects, object_centres):

        mask = np.zeros(len(objects),dtype=bool)

        for i in range(objects):
            if objects[i].size > 15:
                continue
            
            dists = np.linalg.norm(np.array([objects[i]["x"], objects[i]["y"], objects[i]["z"]]) - object_centres[i][:3], axis=1)

            if np.max(dists) > 5: # TODO: cone size = 10???
                continue

            # this object is a cone
            mask[i] = True

        # extract only the found cones -- the ones that passed size and diameter requirements
        cone_clusters = objects[mask]
        cone_locations = object_centres[mask]

        return cone_clusters, cone_locations
    

    def callback(self, msg: PointCloud2):
        """
        Called when point cloud data is received.
        Runs clustering, finds the clusters that are cones, and returns the centre location
        """
        
        # Convert PointCloud2 to a numpy arr
        pc_array = rnp.pointcloud2_to_array(msg)
        
        # Extract (x, y, z) coordinates
        points = np.vstack((pc_array['x'], pc_array['y'], pc_array['z'])).T

        # FILTER: remove null values
        points = points[~np.isnan(points).any(axis=1)]
        
        if len(points) == 0:
            self.get_logger().warn("No valid points received")
            return

        # FILTER: Remove points that are outside of lidar range or min range
        point_norms = np.linalg.norm([points["x"], points["y"], points["z"]], axis=0) # calc dist
        mask = (point_norms < 30) & (point_norms > 1)  # max=30, min=1 dists from lidar
        point_norms = point_norms[mask]
        points = points[mask]

        # FILTER: run cluster detection
        objects, object_centres = self.find_clusters(points)

        # FILTER: removes the non-cone clusters
        cone_clusters, cone_locations = self.filter_cones(objects, object_centres)

        if len(cone_locations) == 0:
            return

        # TODO: bring back points

        # convert cone locs into correct message
        # TODO: Stamped msg ver???
        detected_cones: list = [self.create_cone_msg(cone[0], cone[1], cone[2]) for cone in cone_locations]
        detection_msg = Cone(header=msg.header, cones=detected_cones)

        # publish message
        self.cone_publusher.publish(detection_msg)

        return
    
def main(args=None):
    rclpy.init(args=args)

    node = LiDARConeNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()