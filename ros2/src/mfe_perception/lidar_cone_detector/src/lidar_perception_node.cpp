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

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// CUDA filters: built only with a CUDA compiler, used only if a device is present.
#if defined(PLATFORM_JETSON) || defined(PLATFORM_DESKTOP_GPU)
    #define USE_GPU_PIPELINE 1
    #include <cuda_runtime.h>
    #include <vector>
    #include "lidar_cone_detector/cuda_filter.hpp"
    #include <memory>
    #include "lidar_cone_detector/cuda_cone_pipeline.hpp"
#else
    #define USE_GPU_PIPELINE 0
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
        // Cone-body z band (sensor frame, after ground removal). Defaults fit the EUFS sim
        // VLP-16 (sensor ~1.0 m above track); set from the real mount height on hardware.
        this->declare_parameter("cone_z_min", -1.10f);
        this->declare_parameter("cone_z_max", -0.50f);
        // Ignored when built without CUDA.
        this->declare_parameter("use_gpu", true);

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
        cone_z_min_ = this->get_parameter("cone_z_min").as_double();
        cone_z_max_ = this->get_parameter("cone_z_max").as_double();
        use_gpu_ = false;
#if USE_GPU_PIPELINE
        if (this->get_parameter("use_gpu").as_bool()) {
            int device_count = 0;
            if (cudaGetDeviceCount(&device_count) == cudaSuccess && device_count > 0) {
                use_gpu_ = true;
                gpu_pipe_ = std::make_unique<GpuConePipeline>();
            } else {
                RCLCPP_WARN(this->get_logger(), "use_gpu requested but no CUDA device found; using CPU PCL");
                // Clear the sticky error from the failed query.
                cudaGetLastError();
            }
        }
#endif

        sub_raw_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/points_raw", rclcpp::SensorDataQoS(),
            std::bind(&LidarPerceptionNode::cloud_callback, this, std::placeholders::_1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ekf/output", rclcpp::SensorDataQoS(),
            std::bind(&LidarPerceptionNode::odom_callback, this, std::placeholders::_1));

        pub_centroids_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/perception/cones_uncolored", 10);

        pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/perception/cones_markers", 10);

        pub_debug_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/debug/gpu_objects", 10);

        RCLCPP_INFO(this->get_logger(), "LiDAR Perception Node Initialized. Mode: %s",
            use_gpu_ ? "GPU (CUDA: raw unpack, NaN strip, motion correction, ROI, voxel, RANSAC ground, band, clustering)" : "Standard PCL (CPU)");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_vx_ = msg->twist.twist.linear.x;
        last_vy_ = msg->twist.twist.linear.y;
        last_omega_ = msg->twist.twist.angular.z;
        distortion_corrector_->setEgoMotion(last_vx_, last_vy_, last_omega_);
    }

#if USE_GPU_PIPELINE
    // x/y/z byte offsets for direct GPU upload (float32, aligned, little-endian).
    // Returns false to fall back to the CPU conversion.
    static bool raw_xyz_offsets(const sensor_msgs::msg::PointCloud2& m, unsigned off[3]) {
        if (m.is_bigendian || m.point_step % 4) return false;
        const char* names[3] = {"x", "y", "z"};
        for (int k = 0; k < 3; k++) {
            bool found = false;
            for (const auto& f : m.fields)
                if (f.name == names[k] && f.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
                    f.count == 1 && f.offset % 4 == 0) { off[k] = f.offset; found = true; break; }
            if (!found) return false;
        }
        return m.data.size() >= static_cast<size_t>(m.width) * m.height * m.point_step;
    }
#endif

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // GPU raw path: buffer uploaded as-is; unpack, NaN strip and motion correction run on GPU.
        // The intensity filter (hardware only) still needs host clouds.
        bool gpu_raw = false;
        unsigned raw_off[3] = {0, 0, 0};
#if USE_GPU_PIPELINE
        gpu_raw = use_gpu_ && min_intensity_ <= 0.0 && raw_xyz_offsets(*msg, raw_off);
#endif
        pcl::PointCloud<pcl::PointXYZ>::Ptr host_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (!gpu_raw) {
            // Strip NaN returns (organized Velodyne clouds mark invalid points as NaN).
            pcl::fromROSMsg(*msg, *host_cloud);
            std::vector<int> nan_indices;
            pcl::removeNaNFromPointCloud(*host_cloud, *host_cloud, nan_indices);

            if (host_cloud->empty()) return;

            if (enable_motion_distortion_) {
                host_cloud = distortion_corrector_->correctMotionDistortion(host_cloud, motion_scan_duration_sec_);
            }
        }

        // Intensity cloud for the optional intensity filter.
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
        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all);
        int marker_id = 0;

        #if USE_GPU_PIPELINE
        if (use_gpu_) {
            // GPU pipeline: ROI, voxel, RANSAC ground, band and clustering run on GPU;
            // the host only does the per-cluster box checks.
            GpuConeParams gp;
            gp.leaf = static_cast<float>(leaf_size_);
            gp.ground_threshold = static_cast<float>(ground_threshold_);
            gp.cluster_tolerance = static_cast<float>(cluster_tolerance_);
            gp.min_cluster_size = min_cluster_size_;
            gp.max_cluster_size = max_cluster_size_;
            GpuConeResult res;
            if (gpu_raw) {
                GpuMotion mo;
                mo.enable = enable_motion_distortion_;
                mo.vx = static_cast<float>(last_vx_);
                mo.vy = static_cast<float>(last_vy_);
                mo.omega = static_cast<float>(last_omega_);
                mo.scan_duration = static_cast<float>(motion_scan_duration_sec_);
                gpu_pipe_->runRaw(msg->data.data(), msg->width * msg->height, msg->point_step,
                                  raw_off[0], raw_off[1], raw_off[2], mo, gp, res);
            } else {
                gpu_pipe_->run(reinterpret_cast<const float*>(host_cloud->points.data()),
                               static_cast<unsigned>(host_cloud->points.size()), gp, res);
            }
            RCLCPP_DEBUG(this->get_logger(),
                "gpu: raw=%u valid=%u roi=%u voxel=%u ground=%u band=%zu clusters=%d | ms h2d %.2f unpack %.2f roi %.2f voxel %.2f "
                "ransac %.2f band %.2f cluster %.2f d2h %.2f",
                res.raw, res.valid, res.roi, res.voxel, res.ground, res.labels.size(), res.n_clusters,
                res.t.h2d, res.t.unpack, res.t.roi, res.t.voxel, res.t.ransac, res.t.band, res.t.cluster, res.t.d2h);
            if (res.voxel < 50 || res.labels.empty()) return;

            pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            object_cloud->reserve(res.labels.size());
            for (size_t i = 0; i < res.labels.size(); i++)
                object_cloud->points.emplace_back(res.points[4*i], res.points[4*i+1], res.points[4*i+2]);
            *debug_object_cloud = *object_cloud;

            std::vector<pcl::PointIndices> cluster_indices;
            if (min_intensity_ > 0.0 && !host_cloud_i->empty()) {
                // Intensity filter must run before clustering, so clustering moves to CPU (hardware only).
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
                if (object_cloud->empty()) return;
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
                tree->setInputCloud(object_cloud);
                pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
                ec.setClusterTolerance(cluster_tolerance_);
                ec.setMinClusterSize(min_cluster_size_);
                ec.setMaxClusterSize(max_cluster_size_);
                ec.setSearchMethod(tree);
                ec.setInputCloud(object_cloud);
                ec.extract(cluster_indices);
            } else {
                cluster_indices.resize(res.n_clusters);
                for (size_t i = 0; i < res.labels.size(); i++)
                    if (res.labels[i] >= 0) cluster_indices[res.labels[i]].indices.push_back(static_cast<int>(i));
            }

            for (const auto& idxs : cluster_indices) {
                // FSAE 2026 DD.1.3.2 cone envelopes: small 228x228x325 mm, large orange 285x285x505 mm.
                // Limits below add margin around the large cone.
                float mn_x = FLT_MAX, mx_x = -FLT_MAX;
                float mn_y = FLT_MAX, mx_y = -FLT_MAX;
                float mn_z = FLT_MAX, mx_z = -FLT_MAX;
                for (int idx : idxs.indices) {
                    const auto& p = object_cloud->points[idx];
                    mn_x = std::min(mn_x, p.x); mx_x = std::max(mx_x, p.x);
                    mn_y = std::min(mn_y, p.y); mx_y = std::max(mx_y, p.y);
                    mn_z = std::min(mn_z, p.z); mx_z = std::max(mx_z, p.z);
                }
                if ((mx_x - mn_x) > 0.4f || (mx_y - mn_y) > 0.4f || (mx_z - mn_z) > 0.6f)
                    continue;

                Eigen::Vector4f c;
                pcl::compute3DCentroid(*object_cloud, idxs, c);
                centroids_cloud->points.emplace_back(c[0], c[1], c[2]);

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

        } else
        #endif
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(host_cloud);
            vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
            vg.filter(*downsampled);

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

            pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(downsampled);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*object_cloud);

            // Cone-body band in the sensor frame (limits: cone_z_min/max).
            // Lower limit keeps base points RANSAC may leave; upper excludes car body above cones.
            {
                pcl::PassThrough<pcl::PointXYZ> pt;
                pt.setInputCloud(object_cloud);
                pt.setFilterFieldName("z");
                pt.setFilterLimits(static_cast<float>(cone_z_min_), static_cast<float>(cone_z_max_));
                pcl::PointCloud<pcl::PointXYZ>::Ptr cone_body(new pcl::PointCloud<pcl::PointXYZ>);
                pt.filter(*cone_body);
                object_cloud = cone_body;
            }

            *debug_object_cloud = *object_cloud;

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

                // Same FSAE cone-size check as the GPU branch.
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
                    if ((mx_x - mn_x) > 0.4f || (mx_y - mn_y) > 0.4f || (mx_z - mn_z) > 0.6f)
                        continue;

                    Eigen::Vector4f centroid_vec;
                    pcl::compute3DCentroid(*object_cloud, indices, centroid_vec);
                    centroids_cloud->points.emplace_back(centroid_vec[0], centroid_vec[1], centroid_vec[2]);
                }
            }
        }

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

    bool use_gpu_ = false;
#if USE_GPU_PIPELINE
    // Persistent stream and device buffers.
    std::unique_ptr<GpuConePipeline> gpu_pipe_;
#endif
    double leaf_size_, ground_threshold_, cluster_tolerance_, min_intensity_;
    double cone_z_min_, cone_z_max_;
    int min_cluster_size_, max_cluster_size_;
    std::string lidar_frame_id_;

    bool enable_motion_distortion_;
    double motion_scan_duration_sec_;
    double last_vx_ = 0.0, last_vy_ = 0.0, last_omega_ = 0.0;
    std::shared_ptr<perception::MotionDistortionCorrector> distortion_corrector_;
    std::shared_ptr<perception::IntensityClassifier> intensity_classifier_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPerceptionNode>());
    rclcpp::shutdown();
    return 0;
}
