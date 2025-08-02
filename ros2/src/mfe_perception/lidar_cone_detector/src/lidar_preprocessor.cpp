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

    this->last_theta = 0;
}

void LidarPreprocessor::scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr scan) {   

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*scan, *pcl_cloud);

    // Add to sliding window
    for (auto &p: *pcl_cloud) {
        // Obtain the azimuth of the scan
        double theta = std::atan2(p.x, p.y) * 180.0 / M_PI;

        if (theta < last_theta) {
            // Down-sample for performance concerns on Jetson Nano
            vg_.setInputCloud(full_scan_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
            vg_.filter(*filtered);

            // Convert back to ROS msg type
            sensor_msgs::msg::PointCloud2 out_cloud;
            pcl::toROSMsg(*filtered, out_cloud);

            full_scan_->clear();    // clear the buffer

            out_cloud.header.frame_id = "map";
            out_cloud.header.stamp = scan->header.stamp;
            acc_pub_->publish(out_cloud);
        }

        last_theta = theta;
    }
}

};  /* lidar_cone_detector */

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lidar_cone_detector::LidarPreprocessor>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
