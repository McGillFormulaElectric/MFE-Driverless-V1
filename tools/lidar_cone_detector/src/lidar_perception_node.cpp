#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/filter.h>
#include <cfloat>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Dense>
#include "lidar_cone_detector/motion_distortion.hpp"

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
    #include <pcl/filters/statistical_outlier_removal.h>
    #include <pcl/segmentation/sac_segmentation.h>
    #include <pcl/filters/extract_indices.h>
    #include <pcl/search/kdtree.h>
    #include <pcl/segmentation/extract_clusters.h>
#endif

class LidarPerceptionNode : public rclcpp::Node {
public:
    LidarPerceptionNode() : Node("lidar_perception_node") {
        this->declare_parameter("leaf_size", 0.05f);
        this->declare_parameter("ground_threshold", 0.1f);
        this->declare_parameter("cluster_tolerance", 0.4f);
        this->declare_parameter("min_cluster_size", 3);
        this->declare_parameter("max_cluster_size", 150);
        this->declare_parameter("lidar_frame_id", "velodyne");
        this->declare_parameter("min_intensity", 0.0);
        this->declare_parameter("enable_motion_distortion", true);
        this->declare_parameter("motion_scan_duration_sec", 0.1);
        this->declare_parameter("high_intensity_threshold", 150.0);
        this->declare_parameter("low_intensity_threshold", 50.0);

        leaf_size_ = this->get_parameter("leaf_size").as_double();
        ground_threshold_ = this->get_parameter("ground_threshold").as_double();
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
        lidar_frame_id_ = this->get_parameter("lidar_frame_id").as_string();
        min_intensity_ = this->get_parameter("min_intensity").as_double();
        enable_motion_distortion_ = this->get_parameter("enable_motion_distortion").as_bool();
        motion_scan_duration_sec_ = this->get_parameter("motion_scan_duration_sec").as_double();

        distortion_corrector_ = std::make_shared<perception::MotionDistortionCorrector>();
        intensity_classifier_ = std::make_shared<perception::IntensityClassifier>(
            this->get_parameter("high_intensity_threshold").as_double(),
            this->get_parameter("low_intensity_threshold").as_double()
        );

        sub_raw_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points_raw", rclcpp::SensorDataQoS(),
            std::bind(&LidarPerceptionNode::cloud_callback, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ekf/output", rclcpp::SensorDataQoS(),
            std::bind(&LidarPerceptionNode::odom_callback, this, std::placeholders::_1));

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
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_vx_ = msg->twist.twist.linear.x;
        last_vy_ = msg->twist.twist.linear.y;
        last_omega_ = msg->twist.twist.angular.z;
        distortion_corrector_->setEgoMotion(last_vx_, last_vy_, last_omega_);
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr host_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *host_cloud);
        std::vector<int> nan_indices;
        pcl::removeNaNFromPointCloud(*host_cloud, *host_cloud, nan_indices);

        if (host_cloud->empty()) return;

        if (enable_motion_distortion_) {
            host_cloud = distortion_corrector_->correctMotionDistortion(host_cloud, motion_scan_duration_sec_);
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr host_cloud_i(new pcl::PointCloud<pcl::PointXYZI>);
        if (min_intensity_ > 0.0) {
            pcl::fromROSMsg(*msg, *host_cloud_i);
            std::vector<int> nan_i;
            pcl::removeNaNFromPointCloud(*host_cloud_i, *host_cloud_i, nan_i);

            auto classified = intensity_classifier_->classify(host_cloud_i);
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

            // --- 3b. Cone isolation filter ---
            // Use the fitted ground plane to keep only points at cone height.
            if (coefficients->values.size() >= 4) {
                float a = coefficients->values[0];
                float b = coefficients->values[1];
                float c_n = coefficients->values[2];
                float d = coefficients->values[3];
                float norm = std::sqrt(a*a + b*b + c_n*c_n);

                pcl::PointCloud<pcl::PointXYZ>::Ptr cone_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto& p : object_cloud->points) {
                    // No lateral (Y) bound here — track width varies by discipline
                    // (accel/peanut/autocross), and the forward-range ROI (dim-0
                    // passthrough, -1..15m) plus this height-above-plane check are
                    // the only geometry constraints needed to isolate cone-height
                    // returns; a fixed Y cutoff previously rejected real cones
                    // confirmed present at y=6-9m by ground truth on 2026-10-08.
                    float h = std::abs((a*p.x + b*p.y + c_n*p.z + d) / norm);
                    if (h < 0.005f || h > 0.75f) continue;        // 0–75 cm above ground (covers small 325mm & large 505mm cones + margin)
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

            // --- 4. Cluster on CPU + bounding-box cone validation ---
            if (!object_cloud->empty()) {
                // Near/far split, same density rationale as the CPU pipeline below:
                // a VLP-16 gives only 1-2 returns per cone beyond ~5-6m, so the dense
                // near-field minimum cluster size would silently discard every far cone.
                pcl::PointCloud<pcl::PointXYZ>::Ptr near_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr far_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                std::vector<int> near_indices, far_indices;
                for (size_t i = 0; i < object_cloud->points.size(); ++i) {
                    if (object_cloud->points[i].x < 6.0f) {
                        near_cloud->points.push_back(object_cloud->points[i]);
                        near_indices.push_back(static_cast<int>(i));
                    } else {
                        far_cloud->points.push_back(object_cloud->points[i]);
                        far_indices.push_back(static_cast<int>(i));
                    }
                }

                std::vector<pcl::PointIndices> cluster_indices;
                if (!near_cloud->empty()) {
                    pcl::search::KdTree<pcl::PointXYZ>::Ptr near_tree(new pcl::search::KdTree<pcl::PointXYZ>);
                    near_tree->setInputCloud(near_cloud);
                    std::vector<pcl::PointIndices> near_clusters;
                    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_near;
                    ec_near.setClusterTolerance(cluster_tolerance_);
                    ec_near.setMinClusterSize(min_cluster_size_);
                    ec_near.setMaxClusterSize(max_cluster_size_);
                    ec_near.setSearchMethod(near_tree);
                    ec_near.setInputCloud(near_cloud);
                    ec_near.extract(near_clusters);
                    for (const auto& c : near_clusters) {
                        pcl::PointIndices mapped;
                        for (int idx : c.indices) mapped.indices.push_back(near_indices[idx]);
                        cluster_indices.push_back(mapped);
                    }
                }
                if (!far_cloud->empty()) {
                    pcl::search::KdTree<pcl::PointXYZ>::Ptr far_tree(new pcl::search::KdTree<pcl::PointXYZ>);
                    far_tree->setInputCloud(far_cloud);
                    std::vector<pcl::PointIndices> far_clusters;
                    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_far;
                    ec_far.setClusterTolerance(cluster_tolerance_);
                    ec_far.setMinClusterSize(1);
                    ec_far.setMaxClusterSize(max_cluster_size_);
                    ec_far.setSearchMethod(far_tree);
                    ec_far.setInputCloud(far_cloud);
                    ec_far.extract(far_clusters);
                    for (const auto& c : far_clusters) {
                        pcl::PointIndices mapped;
                        for (int idx : c.indices) mapped.indices.push_back(far_indices[idx]);
                        cluster_indices.push_back(mapped);
                    }
                }

                for (const auto& idxs : cluster_indices) {
                    // Bounding box check — FSAE Driverless 2026 DD.1.3.2:
                    //   Small cones (blue/yellow/orange): 228 × 228 × 325 mm
                    //   Large orange cones (finish gate): 285 × 285 × 505 mm
                    // Filter sized to accept the large cone envelope with margin.
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
                    if ((mx_x - mn_x) > 0.4f || (mx_y - mn_y) > 0.4f || (mx_z - mn_z) > 0.6f)
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

            // 1. Range filter (X-axis passthrough)
            // Beyond 15m ahead, VLP-16 has <2 hits/cone — returns too sparse to cluster reliably.
            // Match GPU path bounds: -1m behind (side cones) to 15m ahead.
            pcl::PointCloud<pcl::PointXYZ>::Ptr range_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            {
                pcl::PassThrough<pcl::PointXYZ> pt;
                pt.setInputCloud(host_cloud);
                pt.setFilterFieldName("x");
                pt.setFilterLimits(-1.0f, 15.0f);
                pt.filter(*range_filtered);
            }

            if (range_filtered->empty()) return;

            // 2. Downsample
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(range_filtered);
            vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            vg.filter(*downsampled);

            // 3. Ground Removal
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

            // 4. Extract Objects
            pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(downsampled);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*object_cloud);

            // 4b. Z-passthrough: keep cone-body height range in the velodyne sensor frame.
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

            // 4c. Statistical Outlier Removal — dense-field only.
            // Root-caused 2026-10-08 via direct per-stage point logging: SOR computes each
            // point's mean distance to its 10 nearest neighbors, then flags it an outlier if
            // that's far above the WHOLE CLOUD's average. This cloud is ~25 dense near-field
            // points (<6m) and a literal handful beyond that (confirmed 2 real points
            // survived ground+z filtering at 6-9m and at 12-16m, every single frame) — a
            // far point's nearest neighbors are necessarily the dense near-field cluster
            // meters away, so its mean-neighbor-distance is enormous next to the crowd's, and
            // it reads as an outlier regardless of threshold (1.0 or 2.0 std-dev both zeroed
            // it out — confirmed empirically, not just in theory). Fix: only run SOR on the
            // genuinely dense sub-cloud (x<6m) and pass sparse far points through untouched;
            // they're still subject to the range filter, ground removal, z-band filter, and
            // the bbox+shape checks below, so this isn't disabling noise rejection, just not
            // applying a density assumption where it doesn't hold.
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr dense_part(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_part(new pcl::PointCloud<pcl::PointXYZ>);
                for (const auto& p : object_cloud->points) {
                    if (p.x < 6.0f) dense_part->points.push_back(p);
                    else sparse_part->points.push_back(p);
                }
                if (!dense_part->empty()) {
                    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
                    sor.setInputCloud(dense_part);
                    sor.setMeanK(10);
                    sor.setStddevMulThresh(2.0);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_sor(new pcl::PointCloud<pcl::PointXYZ>);
                    sor.filter(*filtered_sor);
                    if (!filtered_sor->empty()) dense_part = filtered_sor;
                }
                pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
                *merged = *dense_part;
                *merged += *sparse_part;
                object_cloud = merged;
            }

            // 5. Clustering with range-adaptive tolerance
            // LiDAR point density drops with distance: use tighter tolerance for near points (<10m),
            // looser tolerance for far points (>=10m) to avoid fragmenting sparse distant cones.
            std::vector<pcl::PointIndices> cluster_indices;
            if (!object_cloud->empty()) {
                // Split cloud into near/far based on X distance
                pcl::PointCloud<pcl::PointXYZ>::Ptr near_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr far_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                std::vector<int> near_indices, far_indices;

                for (size_t i = 0; i < object_cloud->points.size(); ++i) {
                    if (object_cloud->points[i].x < 10.0f) {
                        near_cloud->points.push_back(object_cloud->points[i]);
                        near_indices.push_back(i);
                    } else {
                        far_cloud->points.push_back(object_cloud->points[i]);
                        far_indices.push_back(i);
                    }
                }

                // Cluster near points with tighter tolerance (0.3m)
                if (!near_cloud->empty()) {
                    pcl::search::KdTree<pcl::PointXYZ>::Ptr near_tree(new pcl::search::KdTree<pcl::PointXYZ>);
                    near_tree->setInputCloud(near_cloud);
                    std::vector<pcl::PointIndices> near_clusters;
                    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_near;
                    ec_near.setClusterTolerance(0.3f);  // Tighter for dense near points
                    ec_near.setMinClusterSize(min_cluster_size_);
                    ec_near.setMaxClusterSize(max_cluster_size_);
                    ec_near.setSearchMethod(near_tree);
                    ec_near.setInputCloud(near_cloud);
                    ec_near.extract(near_clusters);

                    // Map back to original cloud indices and add to result
                    for (const auto& near_cluster : near_clusters) {
                        pcl::PointIndices mapped_cluster;
                        for (int near_idx : near_cluster.indices) {
                            mapped_cluster.indices.push_back(near_indices[near_idx]);
                        }
                        cluster_indices.push_back(mapped_cluster);
                    }
                }

                // Cluster far points with looser tolerance (0.6m)
                if (!far_cloud->empty()) {
                    pcl::search::KdTree<pcl::PointXYZ>::Ptr far_tree(new pcl::search::KdTree<pcl::PointXYZ>);
                    far_tree->setInputCloud(far_cloud);
                    std::vector<pcl::PointIndices> far_clusters;
                    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_far;
                    ec_far.setClusterTolerance(0.6f);   // Looser for sparse far points
                    // Min size 1, not min_cluster_size_: a VLP-16 gives only 1-2 returns
                    // per cone beyond ~5-6m (see shape-check comment below), so requiring
                    // the same minimum as the dense near field silently discards every
                    // genuine far cone before it even reaches the shape/bbox checks.
                    ec_far.setMinClusterSize(1);
                    ec_far.setMaxClusterSize(max_cluster_size_);
                    ec_far.setSearchMethod(far_tree);
                    ec_far.setInputCloud(far_cloud);
                    ec_far.extract(far_clusters);

                    // Map back to original cloud indices and add to result
                    for (const auto& far_cluster : far_clusters) {
                        pcl::PointIndices mapped_cluster;
                        for (int far_idx : far_cluster.indices) {
                            mapped_cluster.indices.push_back(far_indices[far_idx]);
                        }
                        cluster_indices.push_back(mapped_cluster);
                    }
                }
            }

            // 6. Bounding box check + shape validation + centroids — FSAE Driverless 2026 DD.1.3.2:
            //   Small cones (blue/yellow/orange): 228 x 228 x 325 mm
            //   Large orange cones (finish gate): 285 x 285 x 505 mm
            // Also verify cone shape: clusters should have MORE points at base (bottom, lower z)
            // than at tip (top, higher z) — rejects debris/merged-cones/poles.
            for (const auto& indices : cluster_indices) {
                float mn_x = FLT_MAX, mx_x = -FLT_MAX;
                float mn_y = FLT_MAX, mx_y = -FLT_MAX;
                float mn_z = FLT_MAX, mx_z = -FLT_MAX;
                for (int idx : indices.indices) {
                    const auto& p = object_cloud->points[idx];
                    mn_x = std::min(mn_x, p.x); mx_x = std::max(mx_x, p.x);
                    mn_y = std::min(mn_y, p.y); mx_y = std::max(mx_y, p.y);
                    mn_z = std::min(mn_z, p.z); mx_z = std::max(mx_z, p.z);
                }
                // Reject clusters too large to be cones (walls, barriers, car body)
                if ((mx_x - mn_x) > 0.4f || (mx_y - mn_y) > 0.4f || (mx_z - mn_z) > 0.6f)
                    continue;

                // Vertical-profile check: real cones have more points at base than tip.
                // Count points in bottom-half vs top-half of z-range.
                float z_mid = (mn_z + mx_z) * 0.5f;
                int bottom_count = 0, top_count = 0;
                for (int idx : indices.indices) {
                    if (object_cloud->points[idx].z < z_mid) {
                        bottom_count++;
                    } else {
                        top_count++;
                    }
                }
                // Reject if distribution is too uniform (suggests debris/pole instead of cone).
                // Require at least 1.5x more points at base.
                //
                // Only evaluated for clusters with >=5 points: with a VLP-16, cones past ~5-6m
                // give 1-2 points total, and the bottom/top split of a 2-point cluster is
                // mathematically forced to 1-vs-1 (one point is always the min-z, the other
                // always the max-z) — so this check rejected EVERY sparse far-field cluster
                // unconditionally, regardless of real shape, which is why recall was 0% past
                // ~6m (confirmed by live trial 2026-10-08). Below 5 points the ratio isn't
                // statistically meaningful anyway; the bounding-box size check above already
                // rejects anything too big to be a cone, so just trust it for small clusters.
                if (indices.indices.size() >= 5 && bottom_count < top_count * 1.5f) continue;

                Eigen::Vector4f centroid_vec;
                pcl::compute3DCentroid(*object_cloud, indices, centroid_vec);
                centroids_cloud->points.emplace_back(centroid_vec[0], centroid_vec[1], centroid_vec[2]);
            }
        #endif

        // /debug/gpu_objects mirrors the same centroids published on
        // /perception/cones_uncolored — i.e. cone positions after ground removal
        // + height-band filtering + clustering, not the raw pre-cluster point cloud.
        *debug_object_cloud = *centroids_cloud;

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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_centroids_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_debug_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

    double leaf_size_, ground_threshold_, cluster_tolerance_, min_intensity_;
    int min_cluster_size_, max_cluster_size_;
    std::string lidar_frame_id_;

    bool enable_motion_distortion_;
    double motion_scan_duration_sec_;
    double last_vx_, last_vy_, last_omega_;
    std::shared_ptr<perception::MotionDistortionCorrector> distortion_corrector_;
    std::shared_ptr<perception::IntensityClassifier> intensity_classifier_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
