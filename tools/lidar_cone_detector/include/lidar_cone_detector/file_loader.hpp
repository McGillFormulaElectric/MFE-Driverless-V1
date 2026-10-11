#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>

#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_field_conversion.hpp>

namespace lidar_cone_detector {

class FileLoaderNode : public rclcpp::Node {
    public:
        FileLoaderNode(const rclcpp::NodeOptions &options);

    private:
        rclcpp::TimerBase::SharedPtr timer_;

        bool run_visualization;
        double timeout;
        double time_interval;

        // Naming scheme: underscore denotes use with caution
        size_t current_file_index_;
        std::string dirname;
        std::vector<std::string> binary_files_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub;

        void initialize_params();

        void stream_test_pointcloud();

        pcl::PointCloud<pcl::PointXYZ>::Ptr load_bin_pointcloud(const std::string &bin_file);
}; 

} // namespace lidar_cone_detector