// Whole-frame GPU cone pipeline for lidar_perception_node. See the .cu for stage details.
// Ground fit mirrors pcl::SACSegmentation (perpendicular-plane RANSAC + refit); clustering
// mirrors pcl::EuclideanClusterExtraction; clusters outside [min, max] get label -1.
#pragma once

#include <cstdint>
#include <vector>

#include "lidar_cone_detector/cuda_filter.hpp"

struct GpuConeParams {
  float roi_x_min = -1.0f, roi_x_max = 15.0f;
  float leaf = 0.05f;                            // voxel size, m
  float ground_threshold = 0.1f;                 // RANSAC inlier distance, m
  float eps_angle_rad = 0.17453292f;             // max plane-normal tilt from +z (10 deg)
  int ransac_hypotheses = 256;
  uint32_t seed = 42;                            // fixed: same input -> same plane
  float band_min = 0.005f, band_max = 0.75f;     // height above plane kept as cone body, m
  float y_max = 6.0f;                            // |y| limit, m
  float cluster_tolerance = 0.4f;                // m
  int min_cluster_size = 3, max_cluster_size = 150;
};

struct GpuMotion {        // ego motion for distortion correction, runRaw only
  bool enable = false;
  float vx = 0, vy = 0, omega = 0;   // m/s, m/s, rad/s (sensor frame)
  float scan_duration = 0.1f;        // s
};

struct GpuConeTiming {  // ms, CUDA event timings on the pipeline stream
  float h2d = 0, unpack = 0, roi = 0, voxel = 0, ransac = 0, band = 0, cluster = 0, d2h = 0;
};

struct GpuConeResult {
  std::vector<float> points;   // band points, 4 floats each (x, y, z, pad)
  std::vector<int> labels;     // per band point: cluster id (0..n_clusters-1) or -1
  int n_clusters = 0;
  float plane[4] = {0, 0, 1, 0};
  bool plane_ok = false;
  unsigned raw = 0, valid = 0, roi = 0, voxel = 0, ground = 0;
  GpuConeTiming t;
};

class GpuConePipeline {
public:
  GpuConePipeline();
  ~GpuConePipeline();
  GpuConePipeline(const GpuConePipeline &) = delete;
  GpuConePipeline &operator=(const GpuConePipeline &) = delete;

  // n finite points, 4 floats each (pcl::PointXYZ layout).
  void run(const float *host_xyzp, unsigned n, const GpuConeParams &p, GpuConeResult &out);

  // Raw PointCloud2 bytes; x/y/z are float32 at the given byte offsets. Drops non-finite
  // points on the GPU (order kept), then motion-corrects, then runs the same stages as run().
  void runRaw(const uint8_t *data, unsigned n, unsigned point_step, unsigned off_x, unsigned off_y,
              unsigned off_z, const GpuMotion &motion, const GpuConeParams &p, GpuConeResult &out);

private:
  void process(unsigned n, const GpuConeParams &p, GpuConeResult &out);  // input in device raw buffer
  struct Impl;
  Impl *impl_;
};
