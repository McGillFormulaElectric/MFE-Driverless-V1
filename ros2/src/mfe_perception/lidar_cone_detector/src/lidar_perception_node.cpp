#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

// =================================================================================
// 1. CONDITIONAL HEADERS (The Switch)
// =================================================================================
#if defined(PLATFORM_JETSON) || defined(PLATFORM_DESKTOP_GPU)
    #define USE_GPU_PIPELINE 1
    #include <cuda_runtime.h>
    #include <vector>
    #include "cudaFilter.h"
    #include "cudaSegmentation.h"
    #include <pcl/search/kdtree.h>
    #include <pcl/segmentation/extract_clusters.h>
#else
    #define USE_GPU_PIPELINE 0
    #include <pcl/filters/voxel_grid.h>
    #include <pcl/filters/passthrough.h>
    #include <pcl/segmentation/sac_segmentation.h>
    #include <pcl/filters/extract_indices.h>
    #include <pcl/search/kdtree.h>
    #include <pcl/segmentation/extract_clusters.h>
#endif

class LidarPerceptionNode : public rclcpp::Node {
public:
    LidarPerceptionNode() : Node("lidar_perception_node") {
        // --- Parameters ---
        this->declare_parameter("leaf_size", 0.05f); 
        this->declare_parameter("ground_threshold", 0.1f);
        this->declare_parameter("cluster_tolerance", 0.4f);
        this->declare_parameter("min_cluster_size", 3);
        this->declare_parameter("max_cluster_size", 150);
        this->declare_parameter("lidar_frame_id", "velodyne");

        leaf_size_ = this->get_parameter("leaf_size").as_double();
        ground_threshold_ = this->get_parameter("ground_threshold").as_double();
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
        lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();

        // --- Communication ---
        sub_raw_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points_raw", rclcpp::SensorDataQoS(),
            std::bind(&LidarPerceptionNode::cloud_callback, this, std::placeholders::_1));

        // Output: List of 3D Cone Centroids (Uncolored) -> Goes to Fusion Node
        pub_centroids_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/perception/cones_uncolored", 10);

        // Debug: Visualizing what the GPU actually processed
        pub_debug_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/debug/gpu_objects", 10);

        RCLCPP_INFO(this->get_logger(), "LiDAR Perception Node Initialized. Mode: %s", 
            USE_GPU_PIPELINE ? "CUDA-PCL (Jetson/GPU)" : "Standard PCL (CPU)");
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert Input to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr host_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *host_cloud);

        if (host_cloud->empty()) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr centroids_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr debug_object_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        #if USE_GPU_PIPELINE
            // =========================================================
            // OPTION A: GPU PIPELINE (Using cuFilter, cuSegmentation, cuCluster)
            // =========================================================
            
            cudaStream_t stream = NULL;
            checkCudaErrors(cudaStreamCreate(&stream));

            int point_count = host_cloud->points.size();

            // --- 1. Upload (Host -> GPU) ---
            float* d_input = NULL;
            checkCudaErrors(cudaMalloc(&d_input, point_count * sizeof(pcl::PointXYZ)));
            checkCudaErrors(cudaMemcpy(d_input, host_cloud->points.data(), point_count * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice));

            // --- 1b. cuFilter (Passthrough ROI: X range 0-20m ahead) ---
            float* d_roi = NULL;
            checkCudaErrors(cudaMalloc(&d_roi, point_count * sizeof(pcl::PointXYZ)));
            unsigned int roi_count = 0;

            cudaFilter roi_filter_x(stream);
            FilterParam_t roi_param_x;
            roi_param_x.type = PASSTHROUGH;
            roi_param_x.dim = 0;
            roi_param_x.downFilterLimits = -20.0f;
            roi_param_x.upFilterLimits = 20.0f;
            roi_param_x.limitsNegative = false;
            roi_filter_x.set(roi_param_x);
            roi_filter_x.filter(d_roi, &roi_count, d_input, point_count);
            cudaStreamSynchronize(stream);
            cudaFree(d_input);

            if (roi_count < 50) {
                cudaFree(d_roi);
                checkCudaErrors(cudaStreamDestroy(stream));
                return;
            }

            // --- 2. cuFilter (VoxelGrid) ---
            float* d_filtered = NULL;
            checkCudaErrors(cudaMalloc(&d_filtered, roi_count * sizeof(pcl::PointXYZ)));
            unsigned int filtered_count = 0;

            cudaFilter filter(stream);
            FilterParam_t filter_param;
            filter_param.type = VOXELGRID;
            filter_param.dim = 0;
            filter_param.voxelX = static_cast<float>(leaf_size_);
            filter_param.voxelY = static_cast<float>(leaf_size_);
            filter_param.voxelZ = static_cast<float>(leaf_size_);
            filter.set(filter_param);
            filter.filter(d_filtered, &filtered_count, d_roi, roi_count);
            cudaStreamSynchronize(stream);
            cudaFree(d_roi);

            if (filtered_count < 50) {
                cudaFree(d_filtered);
                checkCudaErrors(cudaStreamDestroy(stream));
                return;
            }

            // --- 3. Ground Removal via Z-axis passthrough (replaces broken cuSegmentation RANSAC)
            // cuSegmentation::getSamples reads GPU device pointers from CPU — causes SIGSEGV.
            // Z-passthrough is faster and equally effective on flat FS tracks.
            float* d_objects = NULL;
            checkCudaErrors(cudaMalloc(&d_objects, filtered_count * sizeof(pcl::PointXYZ)));
            unsigned int object_count = 0;

            cudaFilter ground_filter(stream);
            FilterParam_t ground_param;
            ground_param.type = PASSTHROUGH;
            ground_param.dim = 2;              // Z axis
            ground_param.downFilterLimits = -2.0f; // tune once sensor height is known
            ground_param.upFilterLimits   =  2.0f;
            ground_param.limitsNegative   = false;
            ground_filter.set(ground_param);
            ground_filter.filter(d_objects, &object_count, d_filtered, filtered_count);
            cudaStreamSynchronize(stream);
            cudaFree(d_filtered);

            if (object_count == 0) {
                cudaFree(d_objects);
                checkCudaErrors(cudaStreamDestroy(stream));
                return;
            }

            // Download for visualisation
            std::vector<float> h_objects(object_count * 4);
            cudaMemcpy(h_objects.data(), d_objects, object_count * sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost);
            for (unsigned int i = 0; i < object_count; i++) {
                pcl::PointXYZ p;
                p.x = h_objects[i*4]; p.y = h_objects[i*4+1]; p.z = h_objects[i*4+2];
                debug_object_cloud->points.push_back(p);
            }

            // --- 4. Download GPU-filtered cloud and cluster on CPU ---
            // GPU filters reduce millions of raw points to a small cloud (typically
            // tens to low-hundreds of points on a FSAE track). CPU PCL clustering
            // on that result is near-instant, reliable, and has no minimum-point
            // constraints. cudaExtractCluster is not used.
            cudaFree(d_objects);
            checkCudaErrors(cudaStreamDestroy(stream));

            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (unsigned int i = 0; i < object_count; i++) {
                filtered_cloud->points.emplace_back(h_objects[i*4], h_objects[i*4+1], h_objects[i*4+2]);
            }

            if (!filtered_cloud->empty()) {
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud(filtered_cloud);
                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                ec.setClusterTolerance(cluster_tolerance_);
                ec.setMinClusterSize(min_cluster_size_);
                ec.setMaxClusterSize(max_cluster_size_);
                ec.setSearchMethod(tree);
                ec.setInputCloud(filtered_cloud);
                ec.extract(cluster_indices);
                for (const auto& idxs : cluster_indices) {
                    Eigen::Vector4f c;
                    pcl::compute3DCentroid(*filtered_cloud, idxs, c);
                    centroids_cloud->points.emplace_back(c[0], c[1], c[2]);
                }
            }

        #else
            // =========================================================
            // OPTION B: CPU FALLBACK (Standard PCL)
            // =========================================================
            
            // 1. Downsample
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(host_cloud);
            vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            vg.filter(*downsampled);

            // 2. Ground Removal
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
            seg.setEpsAngle(10.0f * static_cast<float>(M_PI) / 180.0f);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(ground_threshold_);
            seg.setInputCloud(downsampled);
            seg.segment(*inliers, *coefficients);

            // 3. Extract Objects
            pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(downsampled);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*object_cloud);

            // 3b. Z-passthrough: keep cone-body height range in the velodyne sensor frame.
            //     In the EUFS sim the VLP-16 sensor origin is ~1.0 m above the track surface
            //     (confirmed by probing /lidar/points_raw: ground returns at z ≈ -1.014 m median).
            //     FSAE cones are 0.325 m tall, so cone bodies span z ∈ [-1.0, -0.675 m].
            //     After RANSAC removes the ground plane (threshold 0.05 m around z≈-1.0 m),
            //     cone-body returns survive at roughly z ∈ [-0.95, -0.60 m].
            //     Lower limit -1.10 m: keeps returns just below the ground plane (RANSAC may
            //     leave some near-ground cone-base points).  Upper limit -0.50 m: excludes
            //     car-body / roll-bar returns that appear above the cone tops.
            {
                pcl::PassThrough<pcl::PointXYZ> pt;
                pt.setInputCloud(object_cloud);
                pt.setFilterFieldName("z");
                pt.setFilterLimits(-1.10f, -0.50f);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cone_body(new pcl::PointCloud<pcl::PointXYZ>);
                pt.filter(*cone_body);
                object_cloud = cone_body;
            }

            *debug_object_cloud = *object_cloud; // For vis

            // 4. Clustering
            if (!object_cloud->empty()) {
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud(object_cloud);
                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                ec.setClusterTolerance(cluster_tolerance_);
                ec.setMinClusterSize(min_cluster_size_);
                ec.setMaxClusterSize(max_cluster_size_);
                ec.setSearchMethod(tree);
                ec.setInputCloud(object_cloud);
                ec.extract(cluster_indices);

                // 5. Centroids
                for (const auto& indices : cluster_indices) {
                    Eigen::Vector4f centroid_vec;
                    pcl::compute3DCentroid(*object_cloud, indices, centroid_vec);
                    centroids_cloud->points.emplace_back(centroid_vec[0], centroid_vec[1], centroid_vec[2]);
                }
            }
        #endif

        // Publish
        publish_cloud(pub_centroids_, centroids_cloud, msg->header);
        if(!debug_object_cloud->empty()) publish_cloud(pub_debug_, debug_object_cloud, msg->header);
    }

    void publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       const std_msgs::msg::Header& header) {
        if (pub->get_subscription_count() == 0) return;
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*cloud, out_msg);
        out_msg.header = header;
        out_msg.header.frame_id = lidar_frame_id_;
        pub->publish(out_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_raw_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_centroids_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_;

    double leaf_size_, ground_threshold_, cluster_tolerance_;
    int min_cluster_size_, max_cluster_size_;
    std::string lidar_frame_id_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
