#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import Point
from mfe_msgs.msg import Cone, Track
from sensor_msgs.msg import PointCloud2
import ros2_numpy as rnp

from sklearn.cluster import DBSCAN

class LiDARConeNode(Node):
    def __init__(self):

        super().__init__("cone_detector_node")

        self.init_params()

        # subscribe to the outputs of the ground removal script
        # should be the pointcloud with the ground removed
        self.create_subscription(PointCloud2, "pcl/objects", self.callback, 10)

        # publishers - probably point cloud for debugging, then cones
        self.point_cloud_publisher = self.create_publisher(PointCloud2, "pcl/cone_cloud", 10)
        self.track_publisher = self.create_publisher(Track, "pcl/track", 10) # buffer placeholder
    
    def init_params(self):
        """
        Initializes the parameters of the node using rclpy
        """
        self.declare_parameter("dbscan_cluster_min_samples", value=3)
        self.cluster_min_samples = self.get_parameter("dbscan_cluster_min_samples").value

        self.declare_parameter("dbscan_epsilon", value=0.5)
        self.epsilon = self.get_parameter("dbscan_epsilon").value

        return
    
    
    def create_cone_msg(self, x: float, y: float, z: float, cone_color: int):
        """
        Creates a message of type Cone given a location

        """
        if (cone_color not in [0, 1, 2, 3, 4]):
            raise Exception("MessageCreationException")
        
        location: Point = Point(x, y, z)
        return Cone(location=location, color=cone_color)
    

    def find_clusters(self, points):
        """
        Applies DBSCAN clustering algorithm to an np array of points in the point cloud received
        """

        # apply DBSCAN clustering
        # dbscan = DBSCAN(
        #   eps=self.epsilon, 
        #   min_samples=self.cluster_min_samples, 
        #   metric="euclidean", 
        #   n_jobs=2
        # )
        dbscan = DBSCAN(eps=0.5, min_samples=2, metric="euclidean", n_jobs=2)
        clusters = dbscan.fit(points) # params to be changed based on cone specs
        labels = clusters.labels_
        self.get_logger().debug(f"Labels: {labels}")

        # gets clusters by only getting unique labels
        unq_labels = np.unique(labels)
        unq_cluster_labels = unq_labels[unq_labels != -1]
        self.get_logger().info(f"Unique Labels: {unq_cluster_labels}")

        # creates arrays for objects and their centres
        objects = np.empty(unq_labels.size, dtype=object)
        object_centres = np.empty((unq_labels.size, 3))

        # iterate over each cluster and calculate mean 
        for idx, label in enumerate(unq_labels):
            objects[idx] = points[labels == label] # extract the points in that cluster
            object_centres[idx] = np.mean(objects[idx], axis=0)
            self.get_logger().info(f"Cluster {label} centre: {object_centres[idx]}")

        return object_centres, objects
    

    def filter_cones(self, objects, object_centres):
        """
        Filters the cones using data validation of measuring diameter
        """

        mask = np.zeros(len(objects),dtype=bool)

        for i in range(len(objects)):
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
        # BUG: might get error here, not sure of ros2_numpy has this
        points = rnp.numpify(msg)['xyz']

        # # Extract (x, y, z) coordinates
        # points = np.vstack((pc_array['x'], pc_array['y'], pc_array['z'])).T

        # Remove incompatible values using mask
        points = (points[~np.isnan(points).any(axis=1)])
        points_t = points.T

        if len(points) == 0:
            self.get_logger().warn("No valid points received")
            return

        # FILTER: Remove points that are outside of lidar range or min range
        # point_norms = np.linalg.norm([points[0], points[1], points[2]], axis=0) # calc dist'
        # print(point_norms)
        # mask = (point_norms < 30) & (point_norms > 1)  # max=30, min=1 dists from lidar
        # point_norms = point_norms[mask]
        # points = points[mask]

        # run cluster detection
        objects, object_centres = self.find_clusters(points)

        # # FILTER: removes the non-cone clusters
        # cone_clusters, cone_locations = self.filter_cones(objects, object_centres)

        # if len(cone_locations) == 0:
        #     return

        # TODO: Draw cylinders from the raw PointCloud and bring extra points back

        # Create cone message types and chain them into an array 
        # detected_cones: list = [self.create_cone_msg(cone[0], cone[1], cone[2]) for cone in cone_locations]

        track_msg = Track()
        
        track = []
        for centre in object_centres:
            cone_msg_header = Header()
            cone_msg_header.stamp = self.get_clock().now().to_msg()
            cone_msg_header.frame_id = msg.header.frame_id
            cone_msg = Cone(header=cone_msg_header, cones=rnp.msgify())
            track.append(cone_msg)

        track_msg.track = track
            
        # publish locs
        self.track_publisher.publish(track_msg)

        # publish pc w/ cones only
        # cone_clusters = np.concatenate(cone_clusters)
        # new_point_cloud_msg = rnp.msgify(msg, cone_clusters, points)
        # self.point_cloud_publisher.publish(new_point_cloud_msg)

        return
