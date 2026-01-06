#include "../include/lidar_cone_detector/cone_detector.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include "../include/lidar_cone_detector/dbscan.h"

#include <Eigen/Dense>

using sensor_msgs::msg::PointCloud2;

namespace lidar_cone_detector {

ConeDetectorNode::ConeDetectorNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("cone_detector_node", options) {
  // Declare/get params
  this->declare_parameter<double>("dbscan_epsilon", 0.5);
  this->declare_parameter<int>("dbscan_cluster_min_samples", 8);
  this->get_parameter("dbscan_epsilon", eps_);
  this->get_parameter("dbscan_cluster_min_samples", min_pts_);

  auto qos_cloud = rclcpp::SensorDataQoS(); // best-effort, small depth

  sub_cloud_   = this->create_subscription<PointCloud2>(
    "pcl/objects", qos_cloud,
    std::bind(&ConeDetectorNode::onCloud, this, std::placeholders::_1));

  pub_clusters_ = this->create_publisher<PointCloud2>("pcl/objects2", qos_cloud); // clustered points of cones
  pub_centres_  = this->create_publisher<PointCloud2>("pcl/cone_centres", qos_cloud);

  // Keep cones/track as RELIABLE if another node consumes them; for RViz-only, SensorDataQoS is fine too.

  pub_cone_     = this->create_publisher<mfe_msgs::msg::Cone>("pcl/cones", qos_cloud);
  pub_track_    = this->create_publisher<mfe_msgs::msg::Track>("pcl/track", qos_cloud);

  RCLCPP_INFO(this->get_logger(), "cone_detector_node up");
}

void ConeDetectorNode::onCloud(const PointCloud2::SharedPtr msg) {
  // convert ROS -> PCL and drop NaNs
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  std::vector<int> idx; pcl::removeNaNFromPointCloud(cloud, cloud, idx);
  if (cloud.empty()) return;
  
  //debug
  RCLCPP_INFO(this->get_logger(), "Received cloud with %zu points", cloud.size());

  // cluster
  std::vector<std::vector<int>> clusters;
  clusterPoints_DBSCAN(cloud, clusters);

  // build outputs
  pcl::PointCloud<pcl::PointXYZ> centres, all_pts;
  mfe_msgs::msg::Track track;
  for (const auto& ci : clusters) {
    if (ci.empty()) continue;
    Eigen::Vector3d sum(0,0,0);
    for (int k : ci) { 
        const auto& p = cloud.points[k]; 
        all_pts.push_back(p); 
        Eigen::Vector3d temp(p.x,p.y,p.z);
        sum += temp; 
    }
    Eigen::Vector3d c = sum / double(ci.size());
    centres.push_back(pcl::PointXYZ(c.x(), c.y(), c.z()));

    mfe_msgs::msg::Cone cone; cone.location.x=c.x(); cone.location.y=c.y(); cone.location.z=c.z(); cone.color=0;
    pub_cone_->publish(cone);
    track.track.push_back(cone);
  }
  pub_track_->publish(track);

  publishDebugClouds(all_pts, centres, msg->header);

}

void ConeDetectorNode::clusterPoints_DBSCAN(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    std::vector<std::vector<int>>& clusters) const
{
  clusters.clear();
  if (cloud.empty()) return;

  // 1) Build the library’s Point array, mark all UNCLASSIFIED
  std::vector<Point> pts; pts.reserve(cloud.size());
  for (const auto& p : cloud.points) {
    Point q;
    q.x = p.x; q.y = p.y; q.z = p.z;
    q.clusterID = UNCLASSIFIED;
    pts.push_back(q);
  }

  // 2) eps in this lib is COMPARED TO SQUARED DISTANCE → pass eps^2
  const float eps_sq = static_cast<float>(eps_ * eps_);

  DBSCAN db(min_pts_, eps_sq, pts);
  db.run();

  // 3) Group indices by clusterID (>0 are valid clusters)
  const auto& labeled = db.m_points;  // contains updated clusterID
  // find max cluster id
  int max_id = 0;
  for (const auto& p : labeled) if (p.clusterID > max_id) max_id = p.clusterID;
  if (max_id <= 0) return; // only noise/unclassified

  clusters.assign(static_cast<size_t>(max_id), {}); // 1..max_id
  for (size_t i = 0; i < labeled.size(); ++i) {
    int id = labeled[i].clusterID;
    if (id > 0) clusters[static_cast<size_t>(id - 1)].push_back(static_cast<int>(i));
  }
}

void ConeDetectorNode::publishDebugClouds(
    const pcl::PointCloud<pcl::PointXYZ>& all_cluster_pts,
    const pcl::PointCloud<pcl::PointXYZ>& centres,
    const std_msgs::msg::Header& header) {
  // if (!all_cluster_pts.empty()) {
    PointCloud2 out; pcl::toROSMsg(all_cluster_pts, out); out.header = header;
    pub_clusters_->publish(out);
  // }
  // if (!centres.empty()) {
    PointCloud2 out; pcl::toROSMsg(centres, out); out.header = header;
    pub_centres_->publish(out);
  // }
  // // Debug output
  // RCLCPP_INFO(this->get_logger(), "Published %zu cluster points and %zu centres",
  //             all_cluster_pts.size(), centres.size());
}

} // namespace lidar_cone_detector

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_cone_detector::ConeDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
