#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
// Include your custom message headers here
// #include "mfe_msgs/msg/cone.hpp"
// #include "mfe_msgs/msg/track.hpp"

namespace lidar_cone_detector {
    class ConeDetectorNode : public rclcpp::Node {
    public:
        ConeDetectorNode(const rclcpp::NodeOptions &options);

    private:
        void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        
        // Tunable parameters
        double cluster_tolerance_; // Similar to epsilon in DBSCAN
        int min_cluster_size_;
        int max_cluster_size_;

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_clusters_;
        // rclcpp::Publisher<mfe_msgs::msg::Track>::SharedPtr pub_track_;
    };
}
