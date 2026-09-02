#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <cfloat>
#include <pcl/kdtree/kdtree_flann.h>

// =================================================================================
// 1. CONDITIONAL HEADERS (The Switch)
// =================================================================================
#if defined(PLATFORM_JETSON) || defined(PLATFORM_DESKTOP_GPU)
    #define USE_GPU_PIPELINE 1
    #include <cuda_runtime.h>
    #include <vector>
    #include "cudaFilter.h"
    #include <pcl/search/kdtree.h>
    #include <pcl/segmentation/sac_segmentation.h>
    #include <pcl/filters/extract_indices.h>
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
        this->declare_parameter("min_intensity", 0.0);  // 0 = disabled; try 100.0 for real VLP-16

        leaf_size_ = this->get_parameter("leaf_size").as_double();
        ground_threshold_ = this->get_parameter("ground_threshold").as_double();
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
        lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();
        min_intensity_  = this->get_parameter("min_intensity").as_double();

        // --- Communication ---
        sub_raw_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points_raw", rclcpp::SensorDataQoS(),
            std::bind(&LidarPerceptionNode::cloud_callback, this, std::placeholders::_1));

        // Output: List of 3D Cone Centroids (Uncolored) -> Goes to Fusion Node
        pub_centroids_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/perception/cones_uncolored", 10);

        // Output: Bounding box markers for Foxglove visualisation
        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/perception/cones_markers", 10);

        // Debug: Visualizing what the GPU actually processed
        pub_debug_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/debug/gpu_objects", 10);

        RCLCPP_INFO(this->get_logger(), "LiDAR Perception Node Initialized. Mode: %s", 
            USE_GPU_PIPELINE ? "CUDA-PCL (Jetson/GPU)" : "Standard PCL (CPU)");
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert Input to PCL and strip NaN (organized Velodyne clouds use NaN for invalid returns)
        pcl::PointCloud<pcl::PointXYZ>::Ptr host_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *host_cloud);
        std::vector<int> nan_indices;
        pcl::removeNaNFromPointCloud(*host_cloud, *host_cloud, nan_indices);

        if (host_cloud->empty()) return;

        // Load PointXYZI in parallel — used for intensity filtering after geometric pipeline
        pcl::PointCloud<pcl::PointXYZI>::Ptr host_cloud_i(new pcl::PointCloud<pcl::PointXYZI>);
        if (min_intensity_ > 0.0) {
            pcl::fromROSMsg(*msg, *host_cloud_i);
            std::vector<int> nan_i;
            pcl::removeNaNFromPointCloud(*host_cloud_i, *host_cloud_i, nan_i);
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr centroids_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr debug_object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        visualization_msgs::msg::MarkerArray marker_array;
        // Clear previous markers
        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all);
        int marker_id = 0;

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
            roi_param_x.downFilterLimits = -1.0f;   // 1m behind (side cones)
            roi_param_x.upFilterLimits = 15.0f;    // 15m ahead — beyond this VLP-16 has <2 hits/cone
            roi_param_x.limitsNegative = false;
            roi_filter_x.set(roi_param_x);
            roi_filter_x.filter(d_roi, &roi_count, d_input, point_count);
            cudaStreamSynchronize(stream);
            cudaFree(d_input);

            RCLCPP_INFO(this->get_logger(), "raw=%d  roi=%u", point_count, roi_count);

            if (roi_count < 50) {
                cudaFree(d_roi);
                checkCudaErrors(cudaStreamDestroy(stream));
                return;
            }

            // --- 2. cuFilter (VoxelGrid) ---
            // Managed memory: cudaSegmentation::getSamples() reads cloud_in from CPU.
            // cudaMallocManaged lets both GPU kernels and CPU access the same buffer.
            float* d_filtered = NULL;
            checkCudaErrors(cudaMallocManaged(&d_filtered, roi_count * sizeof(pcl::PointXYZ)));
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

            RCLCPP_INFO(this->get_logger(), "voxel=%u", filtered_count);

            if (filtered_count < 50) {
                cudaFree(d_filtered);
                checkCudaErrors(cudaStreamDestroy(stream));
                return;
            }

            // --- 3. Download voxelgrid cloud and run CPU RANSAC ground removal ---
            // cudaSegmentation only supports SACMODEL_PLANE (any plane), which
            // incorrectly fits a tilted plane through all points on a flat track.
            // CPU RANSAC with SACMODEL_PERPENDICULAR_PLANE constrains to horizontal
            // ground and correctly separates cones from the ground plane.
            // On ~5k points this takes <5ms — not a bottleneck.
            std::vector<float> h_filtered(filtered_count * 4);
            cudaMemcpy(h_filtered.data(), d_filtered,
                       filtered_count * sizeof(pcl::PointXYZ), cudaMemcpyDeviceToHost);
            cudaFree(d_filtered);
            checkCudaErrors(cudaStreamDestroy(stream));

            pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            voxel_cloud->reserve(filtered_count);
            for (unsigned int i = 0; i < filtered_count; i++) {
                voxel_cloud->points.emplace_back(h_filtered[i*4], h_filtered[i*4+1], h_filtered[i*4+2]);
            }

            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
            seg.setEpsAngle(10.0f * static_cast<float>(M_PI) / 180.0f);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(ground_threshold_);
            seg.setMaxIterations(100);
            seg.setInputCloud(voxel_cloud);

            pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            seg.segment(*ground_inliers, *coefficients);

            pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(voxel_cloud);
            extract.setIndices(ground_inliers);
            extract.setNegative(true);
            extract.filter(*object_cloud);

            // --- 3b. Cone isolation filters ---
            // Use the fitted ground plane to keep only points at cone height (2–45 cm above ground).
            // Also apply Y-range filter — track is narrow, no need to look far sideways.
            if (coefficients->values.size() >= 4) {
                float a = coefficients->values[0];
                float b = coefficients->values[1];
                float c_n = coefficients->values[2];
                float d = coefficients->values[3];
                float norm = std::sqrt(a*a + b*b + c_n*c_n);

                pcl::PointCloud<pcl::PointXYZ>::Ptr cone_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto& p : object_cloud->points) {
                    if (std::abs(p.y) > 6.0f) continue;           // Y range: ±6 m
                    float h = std::abs((a*p.x + b*p.y + c_n*p.z + d) / norm);
                    if (h < 0.005f || h > 0.7f) continue;         // cone height band (0–70 cm above ground)
                    cone_cloud->points.push_back(p);
                }
                object_cloud = cone_cloud;
            }

            // Intensity filter — keep only high-reflectivity points (retroreflective cone tape)
            // Disabled when min_intensity == 0 (sim has no meaningful intensity values)
            if (min_intensity_ > 0.0 && !host_cloud_i->empty()) {
                pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_i;
                kdtree_i.setInputCloud(host_cloud_i);
                pcl::PointCloud<pcl::PointXYZ>::Ptr hi_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto& p : object_cloud->points) {
                    pcl::PointXYZI sp; sp.x = p.x; sp.y = p.y; sp.z = p.z; sp.intensity = 0;
                    std::vector<int> idx(1); std::vector<float> dist(1);
                    if (kdtree_i.nearestKSearch(sp, 1, idx, dist) > 0 &&
                        host_cloud_i->points[idx[0]].intensity >= static_cast<float>(min_intensity_))
                        hi_cloud->points.push_back(p);
                }
                object_cloud = hi_cloud;
            }

            unsigned int object_count = object_cloud->size();
            RCLCPP_INFO(this->get_logger(), "seg: ground=%zu  cone_band=%u",
                        ground_inliers->indices.size(), object_count);

            if (object_count == 0) return;

            *debug_object_cloud = *object_cloud;

            // --- 4. Cluster on CPU + bounding-box cone validation ---
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

                for (const auto& idxs : cluster_indices) {
                    // Bounding box check: FSAE cones are ≤0.28 m wide, ≤0.325 m tall
                    float mn_x = FLT_MAX, mx_x = -FLT_MAX;
                    float mn_y = FLT_MAX, mx_y = -FLT_MAX;
                    float mn_z = FLT_MAX, mx_z = -FLT_MAX;
                    for (int idx : idxs.indices) {
                        const auto& p = object_cloud->points[idx];
                        mn_x = std::min(mn_x, p.x); mx_x = std::max(mx_x, p.x);
                        mn_y = std::min(mn_y, p.y); mx_y = std::max(mx_y, p.y);
                        mn_z = std::min(mn_z, p.z); mx_z = std::max(mx_z, p.z);
                    }
                    // Reject clusters too large to be cones (walls, barriers, car body)
                    // FSAE cone: 0.28m base diameter, 0.325m tall
                    if ((mx_x - mn_x) > 0.35f || (mx_y - mn_y) > 0.35f || (mx_z - mn_z) > 0.4f)
                        continue;

                    Eigen::Vector4f c;
                    pcl::compute3DCentroid(*object_cloud, idxs, c);
                    centroids_cloud->points.emplace_back(c[0], c[1], c[2]);

                    // Bounding box marker
                    visualization_msgs::msg::Marker m;
                    m.header = msg->header;
                    m.header.frame_id = lidar_frame_id_;
                    m.ns = "cones"; m.id = marker_id++;
                    m.type = visualization_msgs::msg::Marker::CUBE;
                    m.action = visualization_msgs::msg::Marker::ADD;
                    m.pose.position.x = c[0];
                    m.pose.position.y = c[1];
                    m.pose.position.z = (mn_z + mx_z) * 0.5f;
                    m.pose.orientation.w = 1.0;
                    m.scale.x = std::max(mx_x - mn_x, 0.05f);
                    m.scale.y = std::max(mx_y - mn_y, 0.05f);
                    m.scale.z = std::max(mx_z - mn_z, 0.05f);
                    m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f; m.color.a = 0.5f;
                    m.lifetime = rclcpp::Duration::from_seconds(0.3);
                    marker_array.markers.push_back(m);
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
        if (pub_markers_->get_subscription_count() > 0) pub_markers_->publish(marker_array);
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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

    double leaf_size_, ground_threshold_, cluster_tolerance_, min_intensity_;
    int min_cluster_size_, max_cluster_size_;
    std::string lidar_frame_id_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
