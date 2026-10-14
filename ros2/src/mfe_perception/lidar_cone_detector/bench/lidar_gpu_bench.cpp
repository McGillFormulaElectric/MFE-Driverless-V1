// Offline benchmark + equivalence check for lidar_perception_node's two pipelines.
//
// Input: a directory of raw float32 frames (N x 4: x, y, z, intensity), e.g. dumped
// from a rosbag of /lidar/points_raw. Both pipelines replicate the node's branch
// logic stage by stage (same PCL calls and parameters):
//   CPU branch: VoxelGrid -> RANSAC (perpendicular plane) -> extract -> z band -> cluster
//   GPU branch: H2D -> CUDA ROI passthrough -> CUDA VoxelGrid -> D2H -> RANSAC ->
//               extract -> height/Y band -> cluster -> bbox check
//
// Reports per-stage and end-to-end median / p95 latency (after warm-up), and checks:
//   (1) filter equivalence: CUDA ROI+voxel output vs PCL PassThrough+VoxelGrid output
//   (2) cone equivalence: GPU-branch downstream fed by each filter output, compared
//       against a re-run of the same downstream on the same input (RANSAC noise floor).
//
// Usage: lidar_gpu_bench <frames_dir> <stats_csv> [warmup=10] [max_frames=0]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <dirent.h>
#include <fstream>
#include <map>
#include <numeric>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <cuda_runtime.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "lidar_cone_detector/cuda_filter.hpp"
#include "lidar_cone_detector/cuda_cone_pipeline.hpp"

using Clock = std::chrono::steady_clock;
using PC = pcl::PointCloud<pcl::PointXYZ>;
using PCP = PC::Ptr;
using Vec3 = Eigen::Vector3f;

static double msSince(Clock::time_point a) {
  return std::chrono::duration<double, std::milli>(Clock::now() - a).count();
}

// Parameters: node defaults.
struct Params {
  float leaf = 0.05f;
  float ground_threshold = 0.1f;
  double cluster_tolerance = 0.4;
  int min_cluster_size = 3;
  int max_cluster_size = 150;
  float roi_x_min = -1.0f, roi_x_max = 15.0f;
  float cone_z_min = -1.10f, cone_z_max = -0.50f;
};

struct Stats {
  std::map<std::string, std::vector<double>> v;
  void add(const std::string &k, double x) { v[k].push_back(x); }
};

static double pct(std::vector<double> x, double q) {
  if (x.empty()) return 0.0;
  std::sort(x.begin(), x.end());
  size_t i = static_cast<size_t>(std::ceil(q * x.size())) - 1;
  return x[std::min(i, x.size() - 1)];
}

// ---- Node CPU branch (after the voxel stage) --------------------------------
static std::vector<Vec3> cpuDownstream(const PCP &voxel, const Params &P, Stats *s) {
  auto t = Clock::now();
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
  seg.setEpsAngle(10.0f * static_cast<float>(M_PI) / 180.0f);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(P.ground_threshold);
  seg.setInputCloud(voxel);
  seg.segment(*inliers, *coef);
  if (s) s->add("cpu.ransac", msSince(t));
  t = Clock::now();

  PCP obj(new PC);
  pcl::ExtractIndices<pcl::PointXYZ> ex;
  ex.setInputCloud(voxel);
  ex.setIndices(inliers);
  ex.setNegative(true);
  ex.filter(*obj);

  PCP body(new PC);
  pcl::PassThrough<pcl::PointXYZ> pt;
  pt.setInputCloud(obj);
  pt.setFilterFieldName("z");
  pt.setFilterLimits(P.cone_z_min, P.cone_z_max);
  pt.filter(*body);
  if (s) s->add("cpu.extract_band", msSince(t));
  t = Clock::now();

  std::vector<Vec3> out;
  if (!body->empty()) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(body);
    std::vector<pcl::PointIndices> ci;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(P.cluster_tolerance);
    ec.setMinClusterSize(P.min_cluster_size);
    ec.setMaxClusterSize(P.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(body);
    ec.extract(ci);
    for (const auto &idx : ci) {
      Eigen::Vector4f c;
      pcl::compute3DCentroid(*body, idx, c);
      out.emplace_back(c[0], c[1], c[2]);
    }
  }
  if (s) s->add("cpu.cluster", msSince(t));
  return out;
}

// ---- Node GPU branch (after the filter stages) ------------------------------
static std::vector<Vec3> gpuDownstream(const PCP &voxel, const Params &P, Stats *s) {
  auto t = Clock::now();
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr ground(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
  seg.setEpsAngle(10.0f * static_cast<float>(M_PI) / 180.0f);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(P.ground_threshold);
  seg.setMaxIterations(100);
  seg.setInputCloud(voxel);
  seg.segment(*ground, *coef);
  if (s) s->add("gpu.ransac", msSince(t));
  t = Clock::now();

  PCP obj(new PC);
  pcl::ExtractIndices<pcl::PointXYZ> ex;
  ex.setInputCloud(voxel);
  ex.setIndices(ground);
  ex.setNegative(true);
  ex.filter(*obj);

  PCP cone(new PC);
  if (coef->values.size() >= 4) {
    const float a = coef->values[0], b = coef->values[1], c = coef->values[2], d = coef->values[3];
    const float norm = std::sqrt(a * a + b * b + c * c);
    for (const auto &p : obj->points) {
      if (std::abs(p.y) > 6.0f) continue;
      const float h = std::abs((a * p.x + b * p.y + c * p.z + d) / norm);
      if (h < 0.005f || h > 0.75f) continue;
      cone->points.push_back(p);
    }
  } else {
    *cone = *obj;
  }
  if (s) s->add("gpu.extract_band", msSince(t));
  t = Clock::now();

  std::vector<Vec3> out;
  if (!cone->empty()) {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cone);
    std::vector<pcl::PointIndices> ci;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(P.cluster_tolerance);
    ec.setMinClusterSize(P.min_cluster_size);
    ec.setMaxClusterSize(P.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cone);
    ec.extract(ci);
    for (const auto &idxs : ci) {
      float mn_x = FLT_MAX, mx_x = -FLT_MAX, mn_y = FLT_MAX, mx_y = -FLT_MAX;
      float mn_z = FLT_MAX, mx_z = -FLT_MAX;
      for (int i : idxs.indices) {
        const auto &p = cone->points[i];
        mn_x = std::min(mn_x, p.x); mx_x = std::max(mx_x, p.x);
        mn_y = std::min(mn_y, p.y); mx_y = std::max(mx_y, p.y);
        mn_z = std::min(mn_z, p.z); mx_z = std::max(mx_z, p.z);
      }
      if ((mx_x - mn_x) > 0.4f || (mx_y - mn_y) > 0.4f || (mx_z - mn_z) > 0.6f) continue;
      Eigen::Vector4f c;
      pcl::compute3DCentroid(*cone, idxs, c);
      out.emplace_back(c[0], c[1], c[2]);
    }
  }
  if (s) s->add("gpu.cluster", msSince(t));
  return out;
}


// Full-GPU path (GpuConePipeline): cones from its labels, with the node's bbox check.
static std::vector<Vec3> conesFromGpu(const GpuConeResult &r) {
  std::vector<std::vector<int>> cl(r.n_clusters);
  for (size_t i = 0; i < r.labels.size(); i++) if (r.labels[i] >= 0) cl[r.labels[i]].push_back(static_cast<int>(i));
  std::vector<Vec3> out;
  for (const auto &idx : cl) {
    float mn[3] = {FLT_MAX, FLT_MAX, FLT_MAX}, mx[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
    double c[3] = {0, 0, 0};
    for (int i : idx) for (int k = 0; k < 3; k++) {
      const float v = r.points[4 * i + k];
      mn[k] = std::min(mn[k], v); mx[k] = std::max(mx[k], v); c[k] += v;
    }
    if ((mx[0] - mn[0]) > 0.4f || (mx[1] - mn[1]) > 0.4f || (mx[2] - mn[2]) > 0.6f) continue;
    out.emplace_back(c[0] / idx.size(), c[1] / idx.size(), c[2] / idx.size());
  }
  return out;
}


// GPU filter stages, written as in the node's GPU branch (per-frame allocations included).
// Returns the voxel cloud on the host. Stage times are recorded into s if given.
static PCP gpuFilters(const PCP &host, const Params &P, cudaStream_t stream, Stats *s,
                      bool *ok) {
  *ok = false;
  auto t = Clock::now();
  const int n = static_cast<int>(host->points.size());
  float *d_input = nullptr;
  checkCudaErrors(cudaMalloc(&d_input, n * sizeof(pcl::PointXYZ)));
  checkCudaErrors(cudaMemcpy(d_input, host->points.data(), n * sizeof(pcl::PointXYZ),
                             cudaMemcpyHostToDevice));
  float *d_roi = nullptr;
  checkCudaErrors(cudaMalloc(&d_roi, n * sizeof(pcl::PointXYZ)));
  unsigned int roi_count = 0;
  cudaFilter roi(stream);
  FilterParam_t rp;
  rp.type = PASSTHROUGH; rp.dim = 0;
  rp.downFilterLimits = P.roi_x_min; rp.upFilterLimits = P.roi_x_max; rp.limitsNegative = false;
  roi.set(rp);
  roi.filter(d_roi, &roi_count, d_input, n);
  cudaStreamSynchronize(stream);
  cudaFree(d_input);
  if (s) s->add("gpu.h2d_roi", msSince(t));
  t = Clock::now();

  float *d_filtered = nullptr;
  checkCudaErrors(cudaMallocManaged(&d_filtered, roi_count * sizeof(pcl::PointXYZ)));
  unsigned int filtered_count = 0;
  cudaFilter vox(stream);
  FilterParam_t vp;
  vp.type = VOXELGRID; vp.dim = 0;
  vp.voxelX = vp.voxelY = vp.voxelZ = P.leaf;
  vox.set(vp);
  vox.filter(d_filtered, &filtered_count, d_roi, roi_count);
  cudaStreamSynchronize(stream);
  cudaFree(d_roi);
  if (s) s->add("gpu.voxel", msSince(t));
  t = Clock::now();

  std::vector<float> h(static_cast<size_t>(filtered_count) * 4);
  checkCudaErrors(cudaMemcpy(h.data(), d_filtered, filtered_count * sizeof(pcl::PointXYZ),
                             cudaMemcpyDeviceToHost));
  cudaFree(d_filtered);
  PCP voxel(new PC);
  voxel->reserve(filtered_count);
  for (unsigned int i = 0; i < filtered_count; i++)
    voxel->points.emplace_back(h[i * 4], h[i * 4 + 1], h[i * 4 + 2]);
  if (s) s->add("gpu.d2h_build", msSince(t));
  *ok = true;
  return voxel;
}

// PCL CPU filters: passthrough (ROI) and voxel grid, as in the node's CPU branch.
static PCP cpuVoxel(const PCP &host, float leaf) {
  PCP out(new PC);
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(host);
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(*out);
  return out;
}

static PCP cpuRoiVoxel(const PCP &host, const Params &P) {
  PCP roi(new PC);
  pcl::PassThrough<pcl::PointXYZ> pt;
  pt.setInputCloud(host);
  pt.setFilterFieldName("x");
  pt.setFilterLimits(P.roi_x_min, P.roi_x_max);
  pt.filter(*roi);
  return cpuVoxel(roi, P.leaf);
}

// Nearest-neighbour agreement between two point sets (either as points or centroids).
struct NN {
  size_t na = 0, nb = 0, matched = 0;
  double max_nn = 0.0, p99_nn = 0.0;  // over A -> B nearest distances
  double med_match = 0.0, max_match = 0.0;
};

static NN compareSets(const std::vector<Vec3> &A, const std::vector<Vec3> &B, double tol) {
  NN r;
  r.na = A.size(); r.nb = B.size();
  if (A.empty() || B.empty()) return r;
  PCP cb(new PC); cb->reserve(B.size());
  for (const auto &b : B) cb->points.emplace_back(b.x(), b.y(), b.z());
  pcl::KdTreeFLANN<pcl::PointXYZ> kd; kd.setInputCloud(cb);
  std::vector<double> d; d.reserve(A.size());
  std::vector<double> matches;
  for (const auto &a : A) {
    pcl::PointXYZ q(a.x(), a.y(), a.z());
    std::vector<int> idx(1); std::vector<float> sq(1);
    kd.nearestKSearch(q, 1, idx, sq);
    const double dd = std::sqrt(sq[0]);
    d.push_back(dd);
    if (dd <= tol) { r.matched++; matches.push_back(dd); }
  }
  r.max_nn = *std::max_element(d.begin(), d.end());
  r.p99_nn = pct(d, 0.99);
  if (!matches.empty()) {
    r.med_match = pct(matches, 0.5);
    r.max_match = *std::max_element(matches.begin(), matches.end());
  }
  return r;
}

static std::vector<std::string> listFrames(const std::string &dir) {
  std::vector<std::string> files;
  DIR *dp = opendir(dir.c_str());
  if (!dp) return files;
  while (dirent *e = readdir(dp)) {
    std::string n = e->d_name;
    if (n.size() > 4 && n.substr(n.size() - 4) == ".f32") files.push_back(dir + "/" + n);
  }
  closedir(dp);
  std::sort(files.begin(), files.end());
  return files;
}

static PCP loadFrame(const std::string &path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  const size_t bytes = static_cast<size_t>(f.tellg());
  f.seekg(0);
  std::vector<float> buf(bytes / sizeof(float));
  f.read(reinterpret_cast<char *>(buf.data()), bytes);
  PCP pc(new PC);
  pc->points.resize(buf.size() / 4);
  for (size_t i = 0; i < pc->points.size(); i++) {
    pc->points[i].x = buf[4 * i];
    pc->points[i].y = buf[4 * i + 1];
    pc->points[i].z = buf[4 * i + 2];
  }
  std::vector<int> nan_idx;
  pcl::removeNaNFromPointCloud(*pc, *pc, nan_idx);
  return pc;
}

// Filter-stage scaling: concatenate K consecutive raw frames and time ROI+voxel only,
// CPU (PCL PassThrough + VoxelGrid, host) vs CUDA (H2D + ROI + voxel + D2H).
static void scalingBench(const std::vector<std::string> &files, const Params &P, cudaStream_t stream) {
  std::printf("\n=== (3) Filter-stage scaling (ROI + voxel only, concatenated frames) ===\n");
  std::printf("  %8s %14s %14s %9s\n", "points", "CPU median ms", "GPU median ms", "speedup");
  for (int K : {1, 4, 16, 64}) {
    if (static_cast<size_t>(K) > files.size()) break;
    PCP big(new PC);
    for (int k = 0; k < K; k++) {
      PCP f = loadFrame(files[k]);
      big->points.insert(big->points.end(), f->points.begin(), f->points.end());
    }
    std::vector<double> tc, tg;
    const int reps = 30, warm = 5;
    for (int r = 0; r < reps + warm; r++) {
      auto t0 = Clock::now();
      cpuRoiVoxel(big, P);
      double c = msSince(t0);
      bool ok = false;
      auto t1 = Clock::now();
      gpuFilters(big, P, stream, nullptr, &ok);
      double g = msSince(t1);
      if (r >= warm) { tc.push_back(c); tg.push_back(g); }
    }
    const double mc = pct(tc, 0.5), mg = pct(tg, 0.5);
    std::printf("  %8zu %14.3f %14.3f %8.2fx\n", big->points.size(), mc, mg, mc / mg);
  }
}

static void printRow(const char *name, const std::vector<double> &v) {
  if (v.empty()) return;
  std::printf("  %-18s median %9.3f ms   p95 %9.3f ms\n", name, pct(v, 0.5), pct(v, 0.95));
}

int main(int argc, char **argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <frames_dir> <stats_csv> [warmup=10] [max_frames=0]\n", argv[0]);
    return 1;
  }
  const std::string dir = argv[1];
  const std::string csv = argv[2];
  const int warmup = argc > 3 ? std::atoi(argv[3]) : 10;
  const int max_frames = argc > 4 ? std::atoi(argv[4]) : 0;
  Params P;

  auto files = listFrames(dir);
  if (files.empty()) { std::fprintf(stderr, "no .f32 frames in %s\n", dir.c_str()); return 1; }
  if (max_frames > 0 && static_cast<int>(files.size()) > max_frames) files.resize(max_frames);

  cudaDeviceProp prop{};
  cudaGetDeviceProperties(&prop, 0);
  std::printf("GPU: %s (sm_%d%d)  frames: %zu  warmup: %d  leaf: %.3f\n", prop.name, prop.major,
              prop.minor, files.size(), warmup, P.leaf);

  cudaStream_t stream;
  checkCudaErrors(cudaStreamCreate(&stream));

  Stats cpu_s, gpu_s, full_s;  // timed stages
  std::vector<double> full_total;
  GpuConePipeline pipe;
  GpuConeParams gp;   // node defaults (matches Params)
  gp.leaf = P.leaf; gp.ground_threshold = P.ground_threshold; gp.cluster_tolerance = static_cast<float>(P.cluster_tolerance);
  gp.min_cluster_size = P.min_cluster_size; gp.max_cluster_size = P.max_cluster_size;
  size_t full_frames_equal = 0, full_cnt_ref = 0, full_cnt_gpu = 0;
  std::vector<double> full_matched_frac, full_med; double full_worst = 0.0;
  std::vector<double> cpu_total, gpu_total;
  std::vector<double> filt_max_nn, filt_p99_nn, filt_cnt_cpu, filt_cnt_gpu;
  std::vector<double> filt_matched_frac;
  size_t cone_frames_equal = 0, cone_frames = 0;
  std::vector<double> cone_matched_frac, cone_noise_matched_frac, cone_med_match;
  double cone_worst_match = 0.0;
  size_t cone_cnt_cpu_sum = 0, cone_cnt_gpu_sum = 0;

  for (size_t f = 0; f < files.size(); f++) {
    PCP host = loadFrame(files[f]);
    if (host->empty()) continue;
    const bool timed = static_cast<int>(f) >= warmup;

    // --- timed: CPU branch (prep excluded from both totals; identical work) ---
    auto tc = Clock::now();
    PCP cpu_vox = cpuVoxel(host, P.leaf);
    if (timed) cpu_s.add("cpu.voxel", msSince(tc));
    cpuDownstream(cpu_vox, P, timed ? &cpu_s : nullptr);
    if (timed) cpu_total.push_back(msSince(tc));

    // --- timed: GPU branch ---
    bool ok = false;
    auto tg = Clock::now();
    PCP gpu_vox = gpuFilters(host, P, stream, timed ? &gpu_s : nullptr, &ok);
    std::vector<Vec3> cones_gpu = gpuDownstream(gpu_vox, P, timed ? &gpu_s : nullptr);
    if (timed) gpu_total.push_back(msSince(tg));

    // --- timed: full-GPU path (ROI, voxel, RANSAC, band, clustering on the GPU) ---
    GpuConeResult res;
    auto tf = Clock::now();
    pipe.run(reinterpret_cast<const float *>(host->points.data()), static_cast<unsigned>(host->points.size()), gp, res);
    std::vector<Vec3> cones_full = conesFromGpu(res);
    if (timed) {
      full_total.push_back(msSince(tf));
      full_s.add("full.h2d", res.t.h2d); full_s.add("full.roi", res.t.roi); full_s.add("full.voxel", res.t.voxel);
      full_s.add("full.ransac", res.t.ransac); full_s.add("full.band", res.t.band);
      full_s.add("full.cluster", res.t.cluster); full_s.add("full.d2h", res.t.d2h);
    }
    // untimed: same algorithm with PCL RANSAC/cluster on the same CUDA voxel cloud
    {
      NN r = compareSets(cones_gpu, cones_full, 0.05), r2 = compareSets(cones_full, cones_gpu, 0.05);
      full_cnt_ref += cones_gpu.size(); full_cnt_gpu += cones_full.size();
      if (cones_gpu.size() == cones_full.size() && r.matched == r.na && r2.matched == r2.na) full_frames_equal++;
      full_matched_frac.push_back(cones_gpu.empty() ? 1.0 : static_cast<double>(r.matched) / r.na);
      if (r.matched > 0) full_med.push_back(r.med_match);
      full_worst = std::max(full_worst, r.max_match);
    }

    // --- untimed: filter equivalence (CPU ROI+voxel vs CUDA ROI+voxel) ---
    PCP cpu_roi_vox = cpuRoiVoxel(host, P);
    {
      std::vector<Vec3> A, B;
      for (const auto &p : cpu_roi_vox->points) A.emplace_back(p.x, p.y, p.z);
      for (const auto &p : gpu_vox->points) B.emplace_back(p.x, p.y, p.z);
      NN r = compareSets(A, B, 0.001);
      NN r2 = compareSets(B, A, 0.001);
      filt_max_nn.push_back(std::max(r.max_nn, r2.max_nn));
      filt_p99_nn.push_back(std::max(r.p99_nn, r2.p99_nn));
      filt_cnt_cpu.push_back(static_cast<double>(A.size()));
      filt_cnt_gpu.push_back(static_cast<double>(B.size()));
      filt_matched_frac.push_back(A.empty() ? 1.0 : static_cast<double>(r.matched) / A.size());
    }

    // --- untimed: cone equivalence on identical downstream (GPU-branch logic) ---
    {
      std::vector<Vec3> cones_from_cpu_filter = gpuDownstream(cpu_roi_vox, P, nullptr);
      std::vector<Vec3> cones_from_cpu_filter_rerun = gpuDownstream(cpu_roi_vox, P, nullptr);
      NN noise = compareSets(cones_from_cpu_filter, cones_from_cpu_filter_rerun, 0.05);
      NN r = compareSets(cones_from_cpu_filter, cones_gpu, 0.05);
      NN r2 = compareSets(cones_gpu, cones_from_cpu_filter, 0.05);
      cone_frames++;
      if (cones_from_cpu_filter.size() == cones_gpu.size() && r.matched == r.na && r2.matched == r2.na)
        cone_frames_equal++;
      cone_cnt_cpu_sum += cones_from_cpu_filter.size();
      cone_cnt_gpu_sum += cones_gpu.size();
      cone_matched_frac.push_back(cones_from_cpu_filter.empty() ? 1.0
                                  : static_cast<double>(r.matched) / r.na);
      cone_noise_matched_frac.push_back(cones_from_cpu_filter.empty() ? 1.0
                                  : static_cast<double>(noise.matched) / noise.na);
      if (r.matched > 0) cone_med_match.push_back(r.med_match);
      cone_worst_match = std::max(cone_worst_match, r.max_match);
    }

    if ((f + 1) % 40 == 0) std::printf("  ... %zu/%zu frames\n", f + 1, files.size());
  }
  std::printf("\n=== Stage timing (timed frames: %zu) ===\n", cpu_total.size());
  std::printf("CPU branch (PCL):\n");
  for (auto &kv : cpu_s.v) printRow(kv.first.c_str(), kv.second);
  printRow("end-to-end", cpu_total);
  std::printf("GPU branch (CUDA filters + PCL downstream):\n");
  for (auto &kv : gpu_s.v) printRow(kv.first.c_str(), kv.second);
  printRow("end-to-end", gpu_total);

  std::printf("Full GPU pipeline (CUDA events per stage; end-to-end is host wall time):\n");
  for (auto &kv : full_s.v) printRow(kv.first.c_str(), kv.second);
  printRow("end-to-end", full_total);
  std::printf("\n=== (0) Full-GPU cones vs PCL RANSAC+cluster on the same CUDA voxel cloud ===\n");
  std::printf("  frames: %zu  mean cones: PCL %.2f  full-GPU %.2f\n", cone_frames,
              static_cast<double>(full_cnt_ref) / cone_frames, static_cast<double>(full_cnt_gpu) / cone_frames);
  std::printf("  frames with identical cone count and all centroids within 5 cm: %zu / %zu\n", full_frames_equal, cone_frames);
  std::printf("  matched within 5 cm (PCL -> full-GPU, mean): %.4f\n",
              std::accumulate(full_matched_frac.begin(), full_matched_frac.end(), 0.0) / full_matched_frac.size());
  if (!full_med.empty()) std::printf("  median centroid offset of matches: %.4f m  worst: %.4f m\n", pct(full_med, 0.5), full_worst);
  std::printf("\n=== (1) Filter equivalence: CUDA ROI+voxel vs PCL PassThrough+VoxelGrid ===\n");
  std::printf("  frames: %zu  mean points: CPU %.0f  GPU %.0f\n", filt_cnt_cpu.size(),
              std::accumulate(filt_cnt_cpu.begin(), filt_cnt_cpu.end(), 0.0) / filt_cnt_cpu.size(),
              std::accumulate(filt_cnt_gpu.begin(), filt_cnt_gpu.end(), 0.0) / filt_cnt_gpu.size());
  std::printf("  matched within 1 mm (CPU->GPU, mean over frames): %.4f\n",
              std::accumulate(filt_matched_frac.begin(), filt_matched_frac.end(), 0.0) /
                  filt_matched_frac.size());
  std::printf("  worst per-frame nearest-neighbour distance: %.6f m (p99 worst %.6f m)\n",
              *std::max_element(filt_max_nn.begin(), filt_max_nn.end()),
              *std::max_element(filt_p99_nn.begin(), filt_p99_nn.end()));

  std::printf("\n=== (2) Cone equivalence: same GPU-branch downstream on CPU vs CUDA filter output ===\n");
  std::printf("  frames: %zu  mean cones: CPU-filter %.2f  CUDA-filter %.2f\n", cone_frames,
              static_cast<double>(cone_cnt_cpu_sum) / cone_frames,
              static_cast<double>(cone_cnt_gpu_sum) / cone_frames);
  std::printf("  frames with identical cone count and all centroids within 5 cm: %zu / %zu\n",
              cone_frames_equal, cone_frames);
  std::printf("  matched within 5 cm (CPU-filter -> CUDA-filter, mean): %.4f\n",
              std::accumulate(cone_matched_frac.begin(), cone_matched_frac.end(), 0.0) /
                  cone_matched_frac.size());
  std::printf("  RANSAC noise floor (CPU-filter run vs rerun, mean matched): %.4f\n",
              std::accumulate(cone_noise_matched_frac.begin(), cone_noise_matched_frac.end(), 0.0) /
                  cone_noise_matched_frac.size());
  if (!cone_med_match.empty())
    std::printf("  median centroid offset of matches: %.4f m  worst: %.4f m\n",
                pct(cone_med_match, 0.5), cone_worst_match);

  scalingBench(files, P, stream);
  cudaStreamDestroy(stream);

  // CSV: one row per stage, for later analysis.
  std::ofstream out(csv);
  out << "path,stage,n,median_ms,p95_ms\n";
  for (auto &kv : cpu_s.v) out << "cpu," << kv.first << "," << kv.second.size() << "," << pct(kv.second, 0.5) << "," << pct(kv.second, 0.95) << "\n";
  for (auto &kv : gpu_s.v) out << "gpu," << kv.first << "," << kv.second.size() << "," << pct(kv.second, 0.5) << "," << pct(kv.second, 0.95) << "\n";
  out << "cpu,end_to_end," << cpu_total.size() << "," << pct(cpu_total, 0.5) << "," << pct(cpu_total, 0.95) << "\n";
  for (auto &kv : full_s.v) out << "full," << kv.first << "," << kv.second.size() << "," << pct(kv.second, 0.5) << "," << pct(kv.second, 0.95) << "\n";
  out << "full,end_to_end," << full_total.size() << "," << pct(full_total, 0.5) << "," << pct(full_total, 0.95) << "\n";
  out << "gpu,end_to_end," << gpu_total.size() << "," << pct(gpu_total, 0.5) << "," << pct(gpu_total, 0.95) << "\n";
  return 0;
}
