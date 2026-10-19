#include <lidar_cone_detector/lidar_preprocessor.hpp>

namespace lidar_cone_detector {

LidarPreprocessor::LidarPreprocessor(const rclcpp::NodeOptions &options)
: Node("lidar_preprocessor", options) {
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable);

    scans_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pcl/raw", rclcpp::SensorDataQoS(),
        std::bind(&LidarPreprocessor::scanCallback, this, std::placeholders::_1));

    acc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "pcl/acc_cloud", qos_profile);

    vg_.setLeafSize(0.02f, 0.02f, 0.02f);

  full_scan_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    this->last_theta = 0;
}

void LidarPreprocessor::scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr scan) {   

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*scan, *pcl_cloud);

    for (auto &p: *pcl_cloud) {
        full_scan_->points.push_back(p);
        // Azimuth in degrees, measured from +X.
        double theta = std::atan2(p.y, p.x) * 180.0 / M_PI;

        if (theta < last_theta) {
            // Downsample to limit cost on Jetson Nano.
            vg_.setInputCloud(full_scan_);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
            vg_.filter(*filtered);

            sensor_msgs::msg::PointCloud2 out_cloud;
            pcl::toROSMsg(*filtered, out_cloud);

            full_scan_->clear();
            full_scan_->width = 0;
            full_scan_->height = 0;
            // Accumulated cloud stays in the sensor frame.
            out_cloud.header.frame_id = scan->header.frame_id;
            out_cloud.header.stamp = scan->header.stamp;
            acc_pub_->publish(out_cloud);
        }

        last_theta = theta;
    }
}

};  

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lidar_cone_detector::LidarPreprocessor>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
