#ifndef LIDAR_PREPROCESSOR_H
#define LIDAR_PREPROCESSOR_H
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

namespace lidar_cone_detector {

    class LidarPreprocessor : public rclcpp::Node {
    public:
    LidarPreprocessor(const rclcpp::NodeOptions &options);

    private:
    void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr scan);

    pcl::VoxelGrid<pcl::PointXYZ> vg_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scans_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr acc_pub_;
    std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
    const size_t window_scans_{180};   // 720×0.25° = 180 sweep for a 0.25° lidar
    };
}


#endif /* LIDAR_PREPROCESSOR_H */