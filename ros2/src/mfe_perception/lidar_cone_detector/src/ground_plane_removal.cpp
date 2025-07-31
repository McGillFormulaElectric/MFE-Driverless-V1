#include <lidar_cone_detector/ground_plane_removal.hpp>

namespace lidar_cone_detector {
    
GroundPlaneRemovalNode::GroundPlaneRemovalNode(const rclcpp::NodeOptions &options)
: Node("ground_plane_removal_node", options)
{

    this->declare_parameter("run_visualization", false);
    this->get_parameter("run_visualization", this->run_visualization);

    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable);


    // Subscribes to the general point cloud and publishes ground data and the rest in two separate streams
    point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pcl/acc_cloud", rclcpp::SensorDataQoS(),
        std::bind(&GroundPlaneRemovalNode::remove_ground_plane_callback, this, std::placeholders::_1)
    );
    point_cloud_ground_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "pcl/ground",
        qos_profile
    );
    point_cloud_objs_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "pcl/objects",
        qos_profile
    );

    // params defined in header file
}
        
// copy of the main method from 02/2025 demo ransac
void GroundPlaneRemovalNode::remove_ground_plane_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcd);

    // Voxel downsampling    
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(pcd);
    sor.setLeafSize(0.05f, 0.05f, 0.05f); // sets box size in which will only contain 1 point
    // can change how many points are in each box with setMinimumPointsNumberPerVoxel
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pcd(new pcl::PointCloud<pcl::PointXYZ>);
    sor.filter(*downsampled_pcd);

    // Flatten PCL to work with CUDA
    int pointCount = downsampled_pcd->points.size();

    if (pointCount == 0){
        rclcpp_warn(this->get_logger(), "No points to parse");
        return;
    }

    float* flattenedPCL = new float[pointCount * 3];
    int* indexArray = new int[pointCount];

    for (int i; i < pointCount; i++){
        flattenedPCL[i*3 + 0] = downsampled_pcd->points[i].x;
        flattenedPCL[i*3 + 1] = downsampled_pcd->points[i].y;
        flattenedPCL[i*3 + 2] = downsampled_pcd->points[i].z;
        index[i] = 0;
    }


    // CUDA Segmentation setup
    cudaStream_t stream = 0; // Use default stream
    cudaSegmentation cudaSeg(SACMODEL_PLANE, SAC_RANSAC, stream);

    float modelCoefficients[4];

    segParam_t setP;
    setP.distanceThreshold = 0.01;
    setP.maxIterations = 50;
    setP.probability = 0.99;
    setP.optimizeCoefficients = true;
    cudaSeg.set(setP);
    cudaSeg.segment(flattenedPCL, pointCount, indexArray, modelCoefficients);

    // logging coefficient data
    if (modelCoefficients.size() >= 4) {
        rclcpp_info(
            this->get_logger(),
            "ransac plane coefficients: [a=%f, b=%f, c=%f, d=%f]",
            modelCoefficients[0],
            modelCoefficients[1],
            modelCoefficients[2],
            modelCoefficients[3]
       );  // macro for logger
    } else {
        rclcpp_warn(this->get_logger(), "not enough coefficients returned by the segmenter.");
    }



    // Extract inliers and outliers
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < pointCount; i++){
        if (index[i] == 1){
            // This is when points are part of the ground
            outlier_cloud->points.push_back(downsampled_pcd[i]);
        } else {
            // Not part of ground plane
            inlier_cloud->points.push_back(downsampled_pcd[i]);
        }
    }


    // inlier point parameter
    inlier_cloud->width = inlier_cloud->points.size();
    inlier_cloud->height = 1;
    inlier_cloud->is_dense = false;
    // outlier point parameter
    outlier_cloud->width = outlier_cloud->points.size();
    outlier_cloud->height = 1;
    outlier_cloud->is_dense = false;


    // Output processed data and publish to topics
    sensor_msgs::msg::PointCloud2 cones_output;
    sensor_msgs::msg::PointCloud2 ground_output;

    pcl::toROSMsg(*inlier_cloud, cones_output);
    pcl::toROSMsg(*outlier_cloud, ground_output);

    // Set times and frame_ids
    cones_output.header.frame_id = "fsds/FSCar"; 
    cones_output.header.stamp = this->get_clock()->now();
    ground_output.header.frame_id = "fsds/FSCar";
    ground_output.header.stamp = this->get_clock()->now();

    this->point_cloud_objs_pub->publish(cones_output);
    point_cloud_ground_pub->publish(ground_output);


    delete[] indexArray;
    delete[] flattenedPCL; 

    // Perform visualization depending on if set in ROS params
    // WARNING: this does not work yet
    if (this->run_visualization)
    {
        // Visualize the point cloud using PCL visualization
        // Eigen::Vector3f vehicle_position(data["data"][i]["odom"]["x"].asFloat(),
        // data["data"][i]["odom"]["y"].asFloat(),
        // data["data"][i]["odom"]["z"].asFloat());
        // this->visualize(inlier_cloud, vehicle_position);
    }
}   // void remove_ground_plane_callback

/*
    Performs visualization of the RANSAC algorithm using RViz2. 

    Note: this function should only execute when ROS2 Parameter is setup properly.
    
    @param pcd pcl::PointerCloud<pcl::PointXYZ> The point cloud
    @param vehicle_position Eigen::Vector3f The 3D coordinate of the vehicle from SLAM
*/
void GroundPlaneRemovalNode::visualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd, const Eigen::Vector3f &vehicle_position)
{
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

    // Add point cloud to the viewer
    viewer.addPointCloud<pcl::PointXYZ>(pcd, "cloud");

    // Set up the viewer
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

    // Add coordinate frame at the vehicle position
    viewer.addCoordinateSystem(1.0, vehicle_position.x(), vehicle_position.y(), vehicle_position.z());

    // Main loop for visualization
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }
}   // void visualize
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<lidar_cone_detector::GroundPlaneRemovalNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}