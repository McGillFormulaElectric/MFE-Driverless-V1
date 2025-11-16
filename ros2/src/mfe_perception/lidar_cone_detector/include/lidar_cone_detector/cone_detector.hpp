// include/lidar_cone_detector/cone_detector.hpp
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "mfe_msgs/msg/cone.hpp"
#include "mfe_msgs/msg/track.hpp"

// Forward-declare PCL types to keep the header light if you like
namespace pcl {
  template <typename T> class PointCloud;
  struct PointXYZ;
}

namespace lidar_cone_detector {

class ConeDetectorNode : public rclcpp::Node {
public:
  explicit ConeDetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // --- Callbacks ---
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // Fill with cluster indices (each cluster is vector<int> of point indices)
  void clusterPoints_DBSCAN(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                            std::vector<std::vector<int>>& clusters) const;

  void publishDebugClouds(const pcl::PointCloud<pcl::PointXYZ>& all_cluster_pts,
                          const pcl::PointCloud<pcl::PointXYZ>& centres,
                          const std_msgs::msg::Header& header);

  // --- ROS I/O ---
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_clusters_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_centres_;
  rclcpp::Publisher<mfe_msgs::msg::Cone>::SharedPtr            pub_cone_;
  rclcpp::Publisher<mfe_msgs::msg::Track>::SharedPtr           pub_track_;

  // --- Parameters ---
  double eps_{0.5};        // clustering radius (meters)
  int    min_pts_{3};      // minimum points per cluster
};

} // namespace lidar_cone_detector

