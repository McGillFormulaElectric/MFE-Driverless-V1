#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <memory>
#include <cuda_runtime.h>
#include <vector_functions.h>
#include "lidar_cone_detector/cudaSegmentation.h"
#include "lidar_cone_detector/cudaCluster.h"

using PointT = pcl::PointXYZ;

class LidarProcessor : public rclcpp::Node {
public:
    LidarProcessor() : Node("LidarConeProcessor") {
        sub_input_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar/pcl/input", 10,
            std::bind(&LidarProcessor::pointcloud_callback, this, std::placeholders::_1));

        pub_centroid_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lidar/object_centroids", 10);

        cudaStreamCreate(&stream_);
        
        seg_ = std::make_unique<cudaSegmentation>(
            SACMODEL_PLANE, SAC_RANSAC, stream_);
        
        clusterer_ = std::make_unique<cudaExtractCluster>(stream_);

        RCLCPP_INFO(this->get_logger(), "GPU Lidar started.");
    }

    ~LidarProcessor() {
        if (d_points_flat) cudaFree(d_points_flat);
        if (d_not_ground_) cudaFree(d_not_ground_);
        if (stream_) cudaStreamDestroy(stream_);
        if (d_cluster_output_) cudaFree(d_cluster_output_);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_centroid_;

    cudaStream_t stream_ = nullptr;
    float4* d_not_ground_ = nullptr;
    float* d_points_flat = nullptr;
    
    size_t points_capacity_ = 0;
    size_t not_ground_capacity_ = 0;
    size_t output_capacity_ = 0;

    float4* d_cluster_output_ = nullptr;
    std::unique_ptr<cudaSegmentation> seg_;
    std::unique_ptr<cudaExtractCluster> clusterer_;

    template<typename T>
    void ensure_gpu_buffer(T** ptr, size_t& capacity, size_t needed) {
        if (needed > capacity) {
            if (*ptr) cudaFree(*ptr);
            capacity = static_cast<size_t>(needed * 1.5);
            RCLCPP_INFO(this->get_logger(),
                "Capacity: %.2f MB, Needed: %.2f MB",
                capacity * sizeof(float4) / 1024.0 / 1024.0,
                needed * sizeof(float4) / 1024.0 / 1024.0);

            cudaError_t err = cudaMalloc(ptr, capacity * sizeof(T));
            
            if (err != cudaSuccess) {
                RCLCPP_ERROR(this->get_logger(), "CUDA MALLOC FAILED: %s", cudaGetErrorString(err));
                // Reset pointer to null to avoid using bad memory
                *ptr = nullptr;
                capacity = 0;
            }


            size_t free_bytes = 0;
            size_t total_bytes = 0;
            err = cudaMemGetInfo(&free_bytes, &total_bytes);
            if (err != cudaSuccess) {
                RCLCPP_ERROR(this->get_logger(), "cudaMemGetInfo failed: %s", cudaGetErrorString(err));
            } else {
                RCLCPP_INFO(this->get_logger(), "GPU memory: free %zu MB / total %zu MB",
                            free_bytes / (1024*1024), total_bytes / (1024*1024));
            }

        }
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        RCLCPP_INFO(this->get_logger(), "Step 1: Received %d points", (int)cloud->points.size());
        const int cloudLen = static_cast<int>(cloud->points.size());
        std::vector<float> cpuPoints(cloudLen*4);

        for (int i = 0; i < cloudLen; i++) {
            cpuPoints[4*i+0] = cloud->points[i].x; 
            cpuPoints[4*i+1] = cloud->points[i].y; 
            cpuPoints[4*i+2] = cloud->points[i].z; 
            cpuPoints[4*i+3] = 1.0f;
        }

        ensure_gpu_buffer(&d_points_flat, points_capacity_, cloudLen*4);
        if (!d_points_flat){
          RCLCPP_ERROR(this->get_logger(), "GPU Memory allocation failed");
          return;
        }
        cudaError_t err;
        err = cudaMemcpy(d_points_flat, cpuPoints.data(), cloudLen * 4 * sizeof(float), cudaMemcpyHostToDevice);
        if (err != cudaSuccess) {
          RCLCPP_INFO(this->get_logger(), "cudaMemcpy Failed: %s", cudaGetErrorString(err))
        }


        int* d_ground_indices = nullptr;
        float* d_model_coefficient = nullptr;
        cudaMalloc((void**)&d_ground_indices, cloudLen * sizeof(int));
        cudaMalloc((void**)&d_model_coefficient, 4*sizeof(float));

        segParam_t setP;
        setP.maxIterations = 50;
        setP.distanceThreshold = 0.01;
        setP.probability = 0.99;
        setP.optimizeCoefficients = true;
        seg_->set(setP);

        std::vector<int> groundIndice(cloudLen);
        std::vector<float> modelCoefficient(4);
        RCLCPP_INFO(this->get_logger(), "d_points_flat=%p, cloudLen=%d", d_points_flat, cloudLen);
        RCLCPP_INFO(this->get_logger(), "d_ground_indices=%p, d_model_coefficient=%p", 
            d_ground_indices, d_model_coefficient);


        RCLCPP_INFO(this->get_logger(), "After Memcpy");
        // Blocking call usually required here unless cuPCL handles sync internally
        seg_->segment(d_points_flat, cloudLen, d_ground_indices, d_model_coefficient);
        RCLCPP_INFO(this->get_logger(), "After segment");
        cudaMemcpy(groundIndice.data(), d_ground_indices, cloudLen * sizeof(int), cudaMemcpyDeviceToHost);
        cudaMemcpy(modelCoefficient.data(), d_model_coefficient, 4 * sizeof(float), cudaMemcpyDeviceToHost);
        cudaStreamSynchronize(stream_);
        RCLCPP_INFO(this->get_logger(), "After synch");

        std::vector<float4> not_ground_pts;
        not_ground_pts.reserve(cloudLen);

        for (int i = 0; i < cloudLen; i++) {
            if (groundIndice[i] == 0) {
                not_ground_pts.push_back(make_float4(cpuPoints[4*i+0], 
                                                    cpuPoints[4*i+1], 
                                                    cpuPoints[4*i+2], 
                                                    cpuPoints[4*i+3]));
            }
        }

        RCLCPP_INFO(this->get_logger(), "Step 2: Non-ground points: %d", (int)not_ground_pts.size());

        if (not_ground_pts.empty()) return;

        const int not_ground_size = static_cast<int>(not_ground_pts.size());
        ensure_gpu_buffer(&d_not_ground_, not_ground_capacity_, not_ground_size);
        cudaMemcpy(d_not_ground_, not_ground_pts.data(), not_ground_size * sizeof(float4), cudaMemcpyHostToDevice);

        extractClusterParam_t clusterParams;
        clusterParams.voxelX = 0.1f; 
        clusterParams.voxelY = 0.1f; 
        clusterParams.voxelZ = 0.1f;
        clusterParams.minClusterSize = 3;
        clusterParams.maxClusterSize = 1000;
        clusterParams.countThreshold = 3;

        clusterer_->set(clusterParams);

        ensure_gpu_buffer(&d_cluster_output_, output_capacity_, not_ground_size);

        std::vector<unsigned int> labels(not_ground_size);
        int cluster_count = clusterer_->extract(
                            (float*)d_not_ground_, 
                            not_ground_size, 
                            (float*) d_cluster_output_, 
                            labels.data());

        RCLCPP_INFO(this->get_logger(), "Step 3: Clusters found: %d", cluster_count);
        if (cluster_count <= 0) return;

        std::vector<pcl::PointXYZ> centroids(cluster_count, pcl::PointXYZ(0,0,0));
        std::vector<int> counts(cluster_count, 0);

        for (int i = 0; i < not_ground_size; i++) {
            int label = labels[i];
            if (label < 0 || label >= cluster_count) continue; 

            centroids[label].x += not_ground_pts[i].x;
            centroids[label].y += not_ground_pts[i].y;
            centroids[label].z += not_ground_pts[i].z;
            counts[label]++;
        }

        pcl::PointCloud<pcl::PointXYZ> out_cloud;
        for (int i = 0; i < cluster_count; i++) {
            if (counts[i] > 0) {
                centroids[i].x /= counts[i];
                centroids[i].y /= counts[i];
                centroids[i].z /= counts[i];
                out_cloud.push_back(centroids[i]);
            }
        }

        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(out_cloud, out_msg);
        out_msg.header = msg->header;
        pub_centroid_->publish(out_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessor>());
    rclcpp::shutdown();
    return 0;
}
