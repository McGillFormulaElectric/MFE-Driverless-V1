// Whole-frame GPU cone pipeline for lidar_perception_node:
//   ROI passthrough -> voxel grid -> RANSAC ground plane -> cone-height band -> Euclidean clustering.
//
// Everything up to the cluster labels stays on the GPU; the host gets back the cone-band
// points and one cluster label per point, and does the per-cluster bounding-box checks
// (a few dozen clusters, not worth a kernel). One instance owns its CUDA stream and
// grow-only device buffers, so steady-state frames do no allocation.
//
// Ground fit matches the CPU path's pcl::SACSegmentation setup: RANSAC over planes whose
// normal is within eps_angle of +z (SACMODEL_PERPENDICULAR_PLANE), best = most inliers
// within ground_threshold, then a least-squares refit on those inliers (what
// setOptimizeCoefficients(true) does). All hypotheses are scored in parallel.
//
// Clustering is connected components over "distance <= cluster_tolerance", the same
// relation pcl::EuclideanClusterExtraction uses; clusters outside [min_size, max_size]
// get label -1, as PCL drops them.
#pragma once

#include <cstdint>
#include <vector>

#include "lidar_cone_detector/cuda_filter.hpp"

struct GpuConeParams {
  float roi_x_min = -1.0f, roi_x_max = 15.0f;   // passthrough on x (sensor frame)
  float leaf = 0.05f;                            // voxel size, m
  float ground_threshold = 0.1f;                 // RANSAC inlier distance, m
  float eps_angle_rad = 0.17453292f;             // max plane-normal tilt from +z (10 deg)
  int ransac_hypotheses = 256;                   // all scored in parallel
  uint32_t seed = 42;                            // fixed: same input -> same plane
  float band_min = 0.005f, band_max = 0.75f;     // height above plane kept as cone body, m
  float y_max = 6.0f;                            // |y| limit, m
  float cluster_tolerance = 0.4f;                // m
  int min_cluster_size = 3, max_cluster_size = 150;
};

struct GpuConeTiming {  // milliseconds, measured with CUDA events on the pipeline stream
  float h2d = 0, roi = 0, voxel = 0, ransac = 0, band = 0, cluster = 0, d2h = 0;
};

struct GpuConeResult {
  std::vector<float> points;   // band points, 4 floats each (x, y, z, pad)
  std::vector<int> labels;     // per band point: cluster id (0..n_clusters-1) or -1
  int n_clusters = 0;
  float plane[4] = {0, 0, 1, 0};
  bool plane_ok = false;
  unsigned raw = 0, roi = 0, voxel = 0, ground = 0;
  GpuConeTiming t;
};

class GpuConePipeline {
public:
  GpuConePipeline();
  ~GpuConePipeline();
  GpuConePipeline(const GpuConePipeline &) = delete;
  GpuConePipeline &operator=(const GpuConePipeline &) = delete;

  // host_xyzp: n points, 4 floats each (pcl::PointXYZ layout), finite.
  void run(const float *host_xyzp, unsigned n, const GpuConeParams &p, GpuConeResult &out);

private:
  struct Impl;
  Impl *impl_;
};
