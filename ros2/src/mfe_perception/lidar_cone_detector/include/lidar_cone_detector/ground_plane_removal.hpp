#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <Eigen/Dense>

#include "pcl/common/angles.h"
#include "pcl/point_types.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/pcd_io.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/visualization/pcl_visualizer.h"

#include "pcl_conversions/pcl_conversions.h"

namespace lidar_cone_detector {
    class GroundPlaneRemovalNode : public rclcpp::Node 
    {
    public:
        GroundPlaneRemovalNode(const rclcpp::NodeOptions &options);

        bool run_visualization;

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_ground_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_cones_pub;

        void remove_ground_plane_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void visualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd, const Eigen::Vector3f &vehicle_position);

        rclcpp::TimerBase::SharedPtr timer_;    // Used to measure performance metrics

        pcl::PointCloud<pcl::PointXYZ>::Ptr acc_cloud;

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_outlier;
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
    }; // ← semicolon here is mandatory

    std::vector<Eigen::Vector3f> load_bin_pointcloud(const std::string &bin_file);
}