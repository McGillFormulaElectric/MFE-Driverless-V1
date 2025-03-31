#include <lidar_cone_detector/file_loader.hpp>

namespace fs = std::filesystem;

namespace lidar_cone_detector {

FileLoaderNode::FileLoaderNode(const rclcpp::NodeOptions &options)
: Node("file_loader_node", options)
{
    using rclcpp::QoS;
    using rclcpp::ReliabilityPolicy;

    this->initialize_params();

    rclcpp::QoS qos_profile = QoS(rclcpp::KeepLast(10)).reliability(ReliabilityPolicy::BestEffort);

    this->point_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "lidar/pcl/raw", 
        qos_profile
    );

    RCLCPP_INFO(this->get_logger(), "Initializing FileLoader to load point cloud dataset...'");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&FileLoaderNode::stream_test_pointcloud, this)
    );

    // Load the directory and list all the pc files in it
    for (const auto & entry : fs::directory_iterator(this->dirname)) {
        if (entry.is_regular_file()) {
          this->binary_files_.push_back(entry.path().string());
        }
    }
    std::sort(binary_files_.begin(), binary_files_.end());
}

void FileLoaderNode::initialize_params()
{
    this->declare_parameter("run_visualization", false);
    this->declare_parameter("timeout", 100.0);
    this->declare_parameter("time_interval", 100.00);
    this->declare_parameter("dirname", "/");

    this->run_visualization = this->get_parameter("run_visualization").as_bool();
    this->timeout = this->get_parameter("timeout").as_double();
    this->time_interval = this->get_parameter("time_interval").as_double();
    this->dirname = this->get_parameter("dirname").as_string();

    this->current_file_index_ = 0;  // initialize file index
}

void FileLoaderNode::stream_test_pointcloud() {
    if (this->binary_files_.empty()) {
        RCLCPP_INFO(this->get_logger(), "No bin files found in directory %s", this->dirname.c_str());
    }

    const std::string & file_path = binary_files_[current_file_index_];

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = this->load_bin_pointcloud(file_path);

    if (!point_cloud) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load point cloud from file: %s", file_path.c_str());
        return;
    }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*point_cloud, msg);

    msg.header.frame_id = "map";
    msg.header.stamp = this->now();

    this->point_cloud_pub->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Published point cloud from file: %s", file_path.c_str());

    current_file_index_ = (current_file_index_ + 1) % binary_files_.size();
}

/*
    Loads a LiDAR point cloud from a binary file using the KITTI format.

    (Deprecation warning) Currently not in use.

    @param bin_file pointer to the binary file
    @return point cloud as a pcl::PointCloud<pcl::PointXYZ>::Ptr
*/
pcl::PointCloud<pcl::PointXYZ>::Ptr FileLoaderNode::load_bin_pointcloud(const std::string &bin_file)
{
    std::ifstream file(bin_file, std::ios::binary);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << bin_file << std::endl;
        RCLCPP_ERROR(this->get_logger(), "Failed to load point cloud from file");
        return {};
    }

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    while (file)
    {
        Eigen::Vector4f eigen_point;
        file.read(reinterpret_cast<char *>(&eigen_point), sizeof(Eigen::Vector4f));
        if (file)
        {
            // Convert the by tranforming the data
            pcl::PointXYZ pcl_point;
            pcl_point.x = eigen_point[0];
            pcl_point.y = eigen_point[1];
            pcl_point.z = eigen_point[2];
            cloud->push_back(pcl_point);
            RCLCPP_INFO(this->get_logger(), "Loaded point: %6.2f %6.2f %6.2f", pcl_point.x, pcl_point.y, pcl_point.z);
        }
    }
    return cloud;
}
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lidar_cone_detector::FileLoaderNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}