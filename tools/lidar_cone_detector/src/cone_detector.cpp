#include "lidar_cone_detector/cone_detector.hpp"

namespace lidar_cone_detector {

ConeDetectorNode::ConeDetectorNode(const rclcpp::NodeOptions &options)
: Node("cone_detector_node", options) {
    // Parameters (Match your Python DBSCAN settings)
    this->declare_parameter("cluster_tolerance", 0.5); // 0.5m distance between points
    this->declare_parameter("min_cluster_size", 3);
    this->declare_parameter("max_cluster_size", 70);

    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();

    sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pcl/objects", rclcpp::SensorDataQoS(),
        std::bind(&ConeDetectorNode::cloud_callback, this, std::placeholders::_1));

    pub_debug_clusters_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "pcl/debug_clusters", 10);
}

void ConeDetectorNode::cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1. Convert ROS msg to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) return;

    // 2. Set up Euclidean Cluster Extraction (The C++ equivalent to your DBSCAN)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_); 
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // 3. Process Clusters
    pcl::PointCloud<pcl::PointXYZ>::Ptr debug_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // mfe_msgs::msg::Track track_msg; // Your custom message
    
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Extract points for this specific cluster
        for (const auto& index : indices.indices) {
            cluster->points.push_back(cloud->points[index]);
            debug_cloud->points.push_back(cloud->points[index]); // Combine for debug visualization
        }

        // Calculate Centroid (The "Cone Location")
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);

        // TODO: Add your filtering logic here (check width/height of 'cluster')

        // Create Cone Message
        // mfe_msgs::msg::Cone cone;
        // cone.location.x = centroid[0];
        // cone.location.y = centroid[1];
        // cone.location.z = centroid[2];
        // track_msg.track.push_back(cone);
    }

    // 4. Publish
    // pub_track_->publish(track_msg);

    // Publish visual debug cloud
    sensor_msgs::msg::PointCloud2 debug_msg;
    pcl::toROSMsg(*debug_cloud, debug_msg);
    debug_msg.header = msg->header;
    pub_debug_clusters_->publish(debug_msg);
}

} // namespace

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lidar_cone_detector::ConeDetectorNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
