#include <lidar_cone_detector/lidar_preprocessor.hpp>

namespace lidar_cone_detector {

LidarPreprocessor::LidarPreprocessor(const rclcpp::NodeOptions &options)
: Node("ground_plane_removal_node", options) {
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable);

    scans_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pcl/raw", rclcpp::SensorDataQoS(),
        std::bind(&LidarPreprocessor::scanCallback, this, std::placeholders::_1));

    acc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "pcl/acc_cloud", qos_profile);

    vg_.setLeafSize(0.02f, 0.02f, 0.02f);
}

void LidarPreprocessor::scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr scan) {   

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*scan, *pcl_cloud);

    // Add to sliding window
    clouds_.push_back(pcl_cloud);
    if (clouds_.size() > window_scans_) clouds_.pop_front();

    // Concatenate
    pcl::PointCloud<pcl::PointXYZ>::Ptr concat_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto &cloud : clouds_) *concat_cloud += *cloud;

     // Down-sample
    vg_.setInputCloud(concat_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
    vg_.filter(*filtered);

    sensor_msgs::msg::PointCloud2 out_cloud;
    pcl::toROSMsg(*concat_cloud, out_cloud);

    // TODO: Change frame_id to appropriate world frame
    out_cloud.header.frame_id = "map";
    out_cloud.header.stamp = scan->header.stamp;
    acc_pub_->publish(out_cloud);
}

};  /* lidar_cone_detector */

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lidar_cone_detector::LidarPreprocessor>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
