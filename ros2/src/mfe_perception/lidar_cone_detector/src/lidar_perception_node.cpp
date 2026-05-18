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
    #include <map>
    
    // NVIDIA cuda-pcl Headers
    // Ensure these files are in your include path (e.g., lib/cuda-pcl/...)
    #include "cudaFilter.h"
    #include "cudaSegmentation.h"
    #include "cudaCluster.h" 
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
            checkCudaErrors(cudaMalloc(&d_input, point_count * sizeof(pcl::PointXYZ))); // x,y,z,padding
            checkCudaErrors(cudaMemcpyAsync(d_input, host_cloud->points.data(), point_count * sizeof(pcl::PointXYZ), cudaMemcpyHostToDevice, stream));

            // --- 1b. cuFilter (Passthrough ROI: X range 0-20m ahead, Y range ±8m) ---
            float* d_roi = NULL;
            checkCudaErrors(cudaMalloc(&d_roi, point_count * sizeof(pcl::PointXYZ)));
            unsigned int roi_count = 0;

            cudaFilter roi_filter_x(stream);
            FilterParam_t roi_param_x;
            roi_param_x.type = PASSTHROUGH;
            roi_param_x.dim = 0; // X axis
            roi_param_x.downFilterLimits = 0.0f;
            roi_param_x.upFilterLimits = 20.0f;
            roi_param_x.limitsNegative = false;
            roi_filter_x.set(roi_param_x);
            roi_filter_x.filter(d_roi, &roi_count, d_input, point_count);

            // Then run VoxelGrid on the ROI output instead of d_input
            // Change the VoxelGrid filter call to use d_roi and roi_count

            // --- 2. cuFilter (VoxelGrid) ---
            float* d_filtered = NULL;
            checkCudaErrors(cudaMalloc(&d_filtered, point_count * sizeof(pcl::PointXYZ)));
            unsigned int filtered_count = 0;

            cudaFilter filter(stream);
            FilterParam_t filter_param;
            filter_param.type = VOXELGRID;
            filter_param.dim = 3;
            filter_param.voxelX = static_cast<float>(leaf_size_);
            filter_param.voxelY = static_cast<float>(leaf_size_);
            filter_param.voxelZ = static_cast<float>(leaf_size_);
            // Note: Different cuFilter versions set leaf size differently.
            // If your version takes it in set(), use that. Otherwise use defaults or modify lib.
            // Assuming this struct supports it or defaults are acceptable:
            filter.set(filter_param);

            // Execute Filter: input -> filtered
            // Note: Check your specific header if it takes explicit leaf sizes in function args
            filter.filter(d_filtered, &filtered_count, d_roi, roi_count);

            // --- 3. cuSegmentation (Ground Removal) ---
            cudaSegmentation segmenter(SACMODEL_PLANE, SAC_RANSAC, stream);
            segParam_t seg_param;
            seg_param.distanceThreshold = ground_threshold_;
            seg_param.maxIterations = 50;
            seg_param.probability = 0.99;
            seg_param.optimizeCoefficients = true;
            segmenter.set(seg_param);

            int* d_seg_indices = NULL;
            checkCudaErrors(cudaMalloc(&d_seg_indices, filtered_count * sizeof(int)));
            float* d_model_coeffs = NULL;
            checkCudaErrors(cudaMalloc(&d_model_coeffs, 4 * sizeof(float)));

            segmenter.segment(d_filtered, filtered_count, d_seg_indices, d_model_coeffs);

            // --- 4. Separate Ground vs Objects (Compaction) ---
            // We download to CPU to remove ground points because it's simpler/robust 
            // without custom kernels.
            std::vector<float> h_filtered(filtered_count * 4);
            std::vector<int> h_seg_indices(filtered_count);
            
            checkCudaErrors(cudaMemcpyAsync(h_filtered.data(), d_filtered, filtered_count * sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost, stream));
            checkCudaErrors(cudaMemcpyAsync(h_seg_indices.data(), d_seg_indices, filtered_count * sizeof(int), cudaMemcpyDeviceToHost, stream));
            checkCudaErrors(cudaStreamSynchronize(stream));

            // Re-pack only OBJECT points
            std::vector<float> object_points;
            object_points.reserve(filtered_count * 4);

            for(size_t i=0; i<filtered_count; i++) {
                // index[i] == 1 usually means Ground Inlier in cuPCL
                if(h_seg_indices[i] != 1) { 
                    object_points.push_back(h_filtered[i*4 + 0]);
                    object_points.push_back(h_filtered[i*4 + 1]);
                    object_points.push_back(h_filtered[i*4 + 2]);
                    object_points.push_back(0.0f); // Maintain padding
                    
                    // Visualization
                    pcl::PointXYZ p;
                    p.x = h_filtered[i*4 + 0]; p.y = h_filtered[i*4 + 1]; p.z = h_filtered[i*4 + 2];
                    debug_object_cloud->points.push_back(p);
                }
            }
            int object_count = object_points.size() / 4;

            // --- 5. cuCluster (Object Detection) ---
            if (object_count > 0) {
                // Re-upload clean objects
                float* d_objects = NULL;
                checkCudaErrors(cudaMalloc(&d_objects, object_points.size() * sizeof(float)));
                checkCudaErrors(cudaMemcpyAsync(d_objects, object_points.data(), object_points.size() * sizeof(float), cudaMemcpyHostToDevice, stream));

                // Prepare Output buffers
                unsigned int* d_cluster_indices = NULL;
                checkCudaErrors(cudaMalloc(&d_cluster_indices, object_count * sizeof(unsigned int)));
                float* d_cluster_out_unused = NULL;
                checkCudaErrors(cudaMalloc(&d_cluster_out_unused, object_count * 4 * sizeof(float)));

                // Configure Clusterer
                cudaExtractCluster clusterer(stream);
                extractClusterParam_t ecp;
                ecp.minClusterSize = min_cluster_size_;
                ecp.maxClusterSize = max_cluster_size_;
                ecp.voxelX = cluster_tolerance_; 
                ecp.voxelY = cluster_tolerance_;
                ecp.voxelZ = cluster_tolerance_;
                ecp.countThreshold = 1; 
                clusterer.set(ecp);

                // Execute
                clusterer.extract(d_objects, object_count, d_cluster_out_unused, d_cluster_indices);

                // --- 6. Centroid Calculation ---
                std::vector<unsigned int> h_cluster_labels(object_count);
                checkCudaErrors(cudaMemcpyAsync(h_cluster_labels.data(), d_cluster_indices, object_count * sizeof(unsigned int), cudaMemcpyDeviceToHost, stream));
                checkCudaErrors(cudaStreamSynchronize(stream));

                // Compute Centroids (Linear Pass)
                std::map<unsigned int, int> counts;
                std::map<unsigned int, std::vector<float>> sums; 

                for (int i = 0; i < object_count; i++) {
                    unsigned int id = h_cluster_labels[i];
                    if (id == 0 || id > 999999) continue; // Noise

                    if (counts.find(id) == counts.end()) {
                        sums[id] = {0.0f, 0.0f, 0.0f};
                        counts[id] = 0;
                    }
                    sums[id][0] += object_points[i*4 + 0];
                    sums[id][1] += object_points[i*4 + 1];
                    sums[id][2] += object_points[i*4 + 2];
                    counts[id]++;
                }

                // Average to find center
                for (auto const& [id, count] : counts) {
                    pcl::PointXYZ p;
                    p.x = sums[id][0] / count;
                    p.y = sums[id][1] / count;
                    p.z = sums[id][2] / count;
                    centroids_cloud->points.push_back(p);
                }

                cudaFree(d_objects);
                cudaFree(d_cluster_indices);
                cudaFree(d_cluster_out_unused);
            }

            // Cleanup
            cudaFree(d_input);
            cudaFree(d_roi);
            cudaFree(d_filtered);
            cudaFree(d_seg_indices);
            cudaFree(d_model_coeffs);
            checkCudaErrors(cudaStreamDestroy(stream));

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
            //     VLP-16 is mounted at z=+0.30 m above ground (base_footprint origin).
            //     FSAE small cones are 0.33 m tall, so in sensor frame:
            //       cone base  ≈ -0.30 m  (ground)
            //       cone top   ≈ +0.03 m  (just above sensor)
            //     After RANSAC removes the ground plane at z≈-0.30 m (±ground_threshold),
            //     surviving cone points span approximately z ∈ [-0.20, +0.05 m].
            //     Upper bound 0.15 m excludes roll-bar / car-body returns above the sensor.
            {
                pcl::PassThrough<pcl::PointXYZ> pt;
                pt.setInputCloud(object_cloud);
                pt.setFilterFieldName("z");
                pt.setFilterLimits(-0.30f, 0.15f);
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
