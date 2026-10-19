// GPU cone pipeline: unpack -> ROI -> voxel -> RANSAC ground -> height band -> clustering.
// RANSAC scores all hypotheses in parallel (grid.y = hypothesis).

#include "lidar_cone_detector/cuda_cone_pipeline.hpp"

#include <thrust/device_vector.h>
#include <thrust/scan.h>
#include <thrust/system/cuda/execution_policy.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>

namespace {

constexpr int kBlock = 256;
inline unsigned blocksFor(unsigned n) { return (n + kBlock - 1) / kBlock; }

__device__ inline float3 sub3(float4 a, float4 b) { return make_float3(a.x - b.x, a.y - b.y, a.z - b.z); }

__global__ void rawFlags(const uint8_t *data, unsigned n, unsigned step, unsigned ox, unsigned oy, unsigned oz,
                         unsigned *flags) {
  const unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const uint8_t *pt = data + static_cast<size_t>(i) * step;
  const float x = *reinterpret_cast<const float *>(pt + ox);
  const float y = *reinterpret_cast<const float *>(pt + oy);
  const float z = *reinterpret_cast<const float *>(pt + oz);
  flags[i] = isfinite(x) && isfinite(y) && isfinite(z);
}

__global__ void rawScatter(const uint8_t *data, unsigned n, unsigned step, unsigned ox, unsigned oy, unsigned oz,
                           const unsigned *flags, const unsigned *offs, float4 *out) {
  const unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n || !flags[i]) return;
  const uint8_t *pt = data + static_cast<size_t>(i) * step;
  out[offs[i]] = make_float4(*reinterpret_cast<const float *>(pt + ox), *reinterpret_cast<const float *>(pt + oy),
                             *reinterpret_cast<const float *>(pt + oz), 0.0f);
}

// Matches perception::MotionDistortionCorrector; capture time is index-based.
__global__ void motionCorrect(float4 *pts, unsigned n, float vx, float vy, float omega, float T) {
  const unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const float dt = T - (static_cast<float>(i) / static_cast<float>(n)) * T;
  float s, c;
  sincosf(omega * dt, &s, &c);
  const float4 p = pts[i];
  pts[i] = make_float4(c * p.x - s * p.y + vx * dt, s * p.x + c * p.y + vy * dt, p.z, p.w);
}

__global__ void planeHyp(const float4 *pts, const int3 *samples, int K, float cos_eps, float4 *planes) {
  int k = blockIdx.x * blockDim.x + threadIdx.x;
  if (k >= K) return;
  const int3 s = samples[k];
  const float4 a = pts[s.x], b = pts[s.y], c = pts[s.z];
  const float3 u = sub3(b, a), v = sub3(c, a);
  float3 n = make_float3(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x);
  const float len = sqrtf(n.x * n.x + n.y * n.y + n.z * n.z);
  if (len < 1e-6f) { planes[k] = make_float4(0, 0, 0, 0); return; }
  n.x /= len; n.y /= len; n.z /= len;
  if (n.z < 0) { n.x = -n.x; n.y = -n.y; n.z = -n.z; }
  if (n.z < cos_eps) { planes[k] = make_float4(0, 0, 0, 0); return; }
  planes[k] = make_float4(n.x, n.y, n.z, -(n.x * a.x + n.y * a.y + n.z * a.z));
}

__global__ void countInliers(const float4 *pts, unsigned n, const float4 *planes, float thr, unsigned *counts) {
  const int k = blockIdx.y;
  const float4 pl = planes[k];
  const unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  int in = 0;
  if (i < n && pl.z != 0.0f) {
    const float4 p = pts[i];
    in = fabsf(pl.x * p.x + pl.y * p.y + pl.z * p.z + pl.w) < thr;
  }
  const int c = __syncthreads_count(in);
  if (threadIdx.x == 0 && c) atomicAdd(&counts[k], static_cast<unsigned>(c));
}

// sums: [n, sx, sy, sz, sxx, sxy, sxz, syy, syz, szz] over inliers of plane pl
__global__ void refitSums(const float4 *pts, unsigned n, float4 pl, float thr, double *sums) {
  const unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  double v[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  if (i < n) {
    const float4 p = pts[i];
    if (fabsf(pl.x * p.x + pl.y * p.y + pl.z * p.z + pl.w) < thr) {
      const double x = p.x, y = p.y, z = p.z;
      v[0] = 1; v[1] = x; v[2] = y; v[3] = z;
      v[4] = x * x; v[5] = x * y; v[6] = x * z; v[7] = y * y; v[8] = y * z; v[9] = z * z;
    }
  }
  for (int j = 0; j < 10; j++) {
    double s = v[j];
    for (int o = 16; o > 0; o >>= 1) s += __shfl_down_sync(0xffffffffu, s, o);
    if ((threadIdx.x & 31) == 0 && s != 0.0) atomicAdd(&sums[j], s);
  }
}

__global__ void bandFlags(const float4 *pts, unsigned n, float4 pl, bool plane_ok, float thr,
                          float bmin, float bmax, float ymax, unsigned *flags) {
  const unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  if (!plane_ok) { flags[i] = 1; return; }        // no ground model: keep all, as the CPU path does
  const float4 p = pts[i];
  const float h = fabsf(pl.x * p.x + pl.y * p.y + pl.z * p.z + pl.w);
  flags[i] = (h >= thr) && (h >= bmin) && (h <= bmax) && (fabsf(p.y) <= ymax);
}

__global__ void scatter(const float4 *in, const unsigned *flags, const unsigned *offs, unsigned n, float4 *out) {
  const unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n && flags[i]) out[offs[i]] = in[i];
}

__global__ void initLabels(int *labels, unsigned n) {
  const unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) labels[i] = static_cast<int>(i);
}

// One sweep: min label over neighbours within tol (self included), tiled through shared memory.
__global__ void propagate(const float4 *pts, unsigned n, float tol2, int *labels, int *changed) {
  __shared__ float4 tp[kBlock];
  __shared__ int tl[kBlock];
  const unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  float4 p = i < n ? pts[i] : make_float4(0, 0, 0, 0);
  int best = i < n ? labels[i] : 0;
  for (unsigned base = 0; base < n; base += kBlock) {
    const unsigned j = base + threadIdx.x;
    if (j < n) { tp[threadIdx.x] = pts[j]; tl[threadIdx.x] = labels[j]; }
    __syncthreads();
    const unsigned lim = min(static_cast<unsigned>(kBlock), n - base);
    if (i < n)
      for (unsigned t = 0; t < lim; t++) {
        const float dx = p.x - tp[t].x, dy = p.y - tp[t].y, dz = p.z - tp[t].z;
        if (dx * dx + dy * dy + dz * dz <= tol2 && tl[t] < best) best = tl[t];
      }
    __syncthreads();
  }
  if (i < n && best < labels[i]) { atomicMin(&labels[i], best); *changed = 1; }
}

__global__ void jump(int *labels, unsigned n) {
  const unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) { int l = labels[i]; int ll = labels[l]; if (ll < l) labels[i] = ll; }
}

// Smallest-eigenvalue eigenvector of a symmetric 3x3 matrix (cyclic Jacobi).
void smallestEigvec(double A[3][3], double out[3]) {
  double V[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  for (int sweep = 0; sweep < 50; sweep++) {
    const double off = A[0][1] * A[0][1] + A[0][2] * A[0][2] + A[1][2] * A[1][2];
    if (off < 1e-30) break;
    for (int p = 0; p < 2; p++)
      for (int q = p + 1; q < 3; q++) {
        if (std::fabs(A[p][q]) < 1e-300) continue;
        const double th = 0.5 * std::atan2(2 * A[p][q], A[q][q] - A[p][p]);
        const double c = std::cos(th), s = std::sin(th);
        for (int k = 0; k < 3; k++) {
          const double akp = A[k][p], akq = A[k][q];
          A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < 3; k++) {
          const double apk = A[p][k], aqk = A[q][k];
          A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk;
        }
        for (int k = 0; k < 3; k++) {
          const double vkp = V[k][p], vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq;
        }
      }
  }
  int m = 0;
  for (int k = 1; k < 3; k++) if (A[k][k] < A[m][m]) m = k;
  for (int k = 0; k < 3; k++) out[k] = V[k][m];
}

}  // namespace

struct GpuConePipeline::Impl {
  cudaStream_t st;
  cudaEvent_t ev[9];
  cudaFilter *roi_f, *vox_f;
  thrust::device_vector<float4> raw, roi, vox, band;
  thrust::device_vector<uint8_t> bytes;
  thrust::device_vector<int3> samples;
  thrust::device_vector<float4> planes;
  thrust::device_vector<unsigned> counts, flags, offs;
  thrust::device_vector<double> sums;
  thrust::device_vector<int> labels, changed;
  std::vector<int3> h_samples;
  std::vector<unsigned> h_counts;
  std::vector<float4> h_planes;
  std::mt19937 rng;
};

GpuConePipeline::GpuConePipeline() : impl_(new Impl) {
  checkCudaErrors(cudaStreamCreate(&impl_->st));
  for (auto &e : impl_->ev) checkCudaErrors(cudaEventCreate(&e));
  impl_->roi_f = new cudaFilter(impl_->st);
  impl_->vox_f = new cudaFilter(impl_->st);
  impl_->sums.resize(10);
  impl_->changed.resize(1);
}

GpuConePipeline::~GpuConePipeline() {
  delete impl_->roi_f;
  delete impl_->vox_f;
  for (auto &e : impl_->ev) cudaEventDestroy(e);
  cudaStreamDestroy(impl_->st);
  delete impl_;
}

template <class T> static T *raw_ptr(thrust::device_vector<T> &v) { return thrust::raw_pointer_cast(v.data()); }
template <class T> static void grow(thrust::device_vector<T> &v, size_t n) { if (v.size() < n) v.resize(n); }

void GpuConePipeline::run(const float *host, unsigned n, const GpuConeParams &P, GpuConeResult &out) {
  Impl &s = *impl_;
  cudaStream_t st = s.st;
  out = GpuConeResult();
  out.raw = out.valid = n;
  if (n == 0) return;
  checkCudaErrors(cudaEventRecord(s.ev[0], st));
  grow(s.raw, n);
  checkCudaErrors(cudaMemcpyAsync(raw_ptr(s.raw), host, n * sizeof(float4), cudaMemcpyHostToDevice, st));
  checkCudaErrors(cudaEventRecord(s.ev[1], st));
  checkCudaErrors(cudaEventRecord(s.ev[2], st));
  process(n, P, out);
}

void GpuConePipeline::runRaw(const uint8_t *data, unsigned n, unsigned step, unsigned ox, unsigned oy, unsigned oz,
                             const GpuMotion &M, const GpuConeParams &P, GpuConeResult &out) {
  Impl &s = *impl_;
  cudaStream_t st = s.st;
  out = GpuConeResult();
  out.raw = n;
  if (n == 0) return;
  checkCudaErrors(cudaEventRecord(s.ev[0], st));
  const size_t nbytes = static_cast<size_t>(n) * step;
  grow(s.bytes, nbytes); grow(s.flags, n); grow(s.offs, n); grow(s.raw, n);
  checkCudaErrors(cudaMemcpyAsync(raw_ptr(s.bytes), data, nbytes, cudaMemcpyHostToDevice, st));
  checkCudaErrors(cudaEventRecord(s.ev[1], st));
  rawFlags<<<blocksFor(n), kBlock, 0, st>>>(raw_ptr(s.bytes), n, step, ox, oy, oz, raw_ptr(s.flags));
  thrust::exclusive_scan(thrust::cuda::par.on(st), raw_ptr(s.flags), raw_ptr(s.flags) + n, raw_ptr(s.offs));
  unsigned lo = 0, lf = 0;
  checkCudaErrors(cudaMemcpyAsync(&lo, raw_ptr(s.offs) + n - 1, sizeof(unsigned), cudaMemcpyDeviceToHost, st));
  checkCudaErrors(cudaMemcpyAsync(&lf, raw_ptr(s.flags) + n - 1, sizeof(unsigned), cudaMemcpyDeviceToHost, st));
  checkCudaErrors(cudaStreamSynchronize(st));
  const unsigned nv = lo + lf;
  rawScatter<<<blocksFor(n), kBlock, 0, st>>>(raw_ptr(s.bytes), n, step, ox, oy, oz, raw_ptr(s.flags),
                                              raw_ptr(s.offs), raw_ptr(s.raw));
  if (M.enable && nv > 0)
    motionCorrect<<<blocksFor(nv), kBlock, 0, st>>>(raw_ptr(s.raw), nv, M.vx, M.vy, M.omega, M.scan_duration);
  checkCudaErrors(cudaEventRecord(s.ev[2], st));
  out.valid = nv;
  if (nv == 0) return;
  process(nv, P, out);
}

void GpuConePipeline::process(unsigned n, const GpuConeParams &P, GpuConeResult &out) {
  Impl &s = *impl_;
  cudaStream_t st = s.st;
  grow(s.roi, n);
  FilterParam_t rp{}; rp.type = PASSTHROUGH; rp.dim = 0;
  rp.downFilterLimits = P.roi_x_min; rp.upFilterLimits = P.roi_x_max; rp.limitsNegative = false;
  s.roi_f->set(rp);
  s.roi_f->filter(raw_ptr(s.roi), &out.roi, raw_ptr(s.raw), n);
  checkCudaErrors(cudaEventRecord(s.ev[3], st));
  grow(s.vox, std::max(out.roi, 1u));
  FilterParam_t vp{}; vp.type = VOXELGRID; vp.voxelX = vp.voxelY = vp.voxelZ = P.leaf;
  s.vox_f->set(vp);
  s.vox_f->filter(raw_ptr(s.vox), &out.voxel, raw_ptr(s.roi), out.roi);
  checkCudaErrors(cudaEventRecord(s.ev[4], st));
  const unsigned m = out.voxel;

  float4 best_plane = make_float4(0, 0, 1, 0);
  if (m >= 3) {
    const int K = P.ransac_hypotheses;
    s.rng.seed(P.seed);
    std::uniform_int_distribution<int> pick(0, static_cast<int>(m) - 1);
    s.h_samples.resize(K);
    for (int k = 0; k < K; k++) {
      int a = pick(s.rng), b, c;
      do { b = pick(s.rng); } while (b == a);
      do { c = pick(s.rng); } while (c == a || c == b);
      s.h_samples[k] = make_int3(a, b, c);
    }
    grow(s.samples, K); grow(s.planes, K); grow(s.counts, K);
    checkCudaErrors(cudaMemcpyAsync(raw_ptr(s.samples), s.h_samples.data(), K * sizeof(int3), cudaMemcpyHostToDevice, st));
    checkCudaErrors(cudaMemsetAsync(raw_ptr(s.counts), 0, K * sizeof(unsigned), st));
    planeHyp<<<blocksFor(K), kBlock, 0, st>>>(raw_ptr(s.vox), raw_ptr(s.samples), K,
                                              std::cos(P.eps_angle_rad), raw_ptr(s.planes));
    countInliers<<<dim3(blocksFor(m), K), kBlock, 0, st>>>(raw_ptr(s.vox), m, raw_ptr(s.planes),
                                                           P.ground_threshold, raw_ptr(s.counts));
    s.h_counts.resize(K); s.h_planes.resize(K);
    checkCudaErrors(cudaMemcpyAsync(s.h_counts.data(), raw_ptr(s.counts), K * sizeof(unsigned), cudaMemcpyDeviceToHost, st));
    checkCudaErrors(cudaMemcpyAsync(s.h_planes.data(), raw_ptr(s.planes), K * sizeof(float4), cudaMemcpyDeviceToHost, st));
    checkCudaErrors(cudaStreamSynchronize(st));
    int bk = -1; unsigned bc = 0;
    for (int k = 0; k < K; k++)
      if (s.h_planes[k].z != 0.0f && s.h_counts[k] > bc) { bc = s.h_counts[k]; bk = k; }
    out.ground = bc;   // pre-refit inlier count, for logs
    if (bk >= 0 && bc >= 3) {
      // least-squares refit on the winner's inliers (like setOptimizeCoefficients)
      checkCudaErrors(cudaMemsetAsync(raw_ptr(s.sums), 0, 10 * sizeof(double), st));
      refitSums<<<blocksFor(m), kBlock, 0, st>>>(raw_ptr(s.vox), m, s.h_planes[bk], P.ground_threshold, raw_ptr(s.sums));
      double h[10];
      checkCudaErrors(cudaMemcpyAsync(h, raw_ptr(s.sums), sizeof(h), cudaMemcpyDeviceToHost, st));
      checkCudaErrors(cudaStreamSynchronize(st));
      const double cnt = h[0];
      const double cx = h[1] / cnt, cy = h[2] / cnt, cz = h[3] / cnt;
      double A[3][3] = {{h[4] / cnt - cx * cx, h[5] / cnt - cx * cy, h[6] / cnt - cx * cz},
                        {0, h[7] / cnt - cy * cy, h[8] / cnt - cy * cz},
                        {0, 0, h[9] / cnt - cz * cz}};
      A[1][0] = A[0][1]; A[2][0] = A[0][2]; A[2][1] = A[1][2];
      double nv[3];
      smallestEigvec(A, nv);
      if (nv[2] < 0) { nv[0] = -nv[0]; nv[1] = -nv[1]; nv[2] = -nv[2]; }
      best_plane = make_float4(nv[0], nv[1], nv[2], -(nv[0] * cx + nv[1] * cy + nv[2] * cz));
      out.plane_ok = true;
    }
  }
  out.plane[0] = best_plane.x; out.plane[1] = best_plane.y; out.plane[2] = best_plane.z; out.plane[3] = best_plane.w;
  checkCudaErrors(cudaEventRecord(s.ev[5], st));

  unsigned nb = 0;
  if (m > 0) {
    grow(s.flags, m); grow(s.offs, m); grow(s.band, m);
    bandFlags<<<blocksFor(m), kBlock, 0, st>>>(raw_ptr(s.vox), m, best_plane, out.plane_ok, P.ground_threshold,
                                               P.band_min, P.band_max, P.y_max, raw_ptr(s.flags));
    thrust::exclusive_scan(thrust::cuda::par.on(st), raw_ptr(s.flags), raw_ptr(s.flags) + m, raw_ptr(s.offs));
    unsigned lo = 0, lf = 0;
    checkCudaErrors(cudaMemcpyAsync(&lo, raw_ptr(s.offs) + m - 1, sizeof(unsigned), cudaMemcpyDeviceToHost, st));
    checkCudaErrors(cudaMemcpyAsync(&lf, raw_ptr(s.flags) + m - 1, sizeof(unsigned), cudaMemcpyDeviceToHost, st));
    checkCudaErrors(cudaStreamSynchronize(st));
    nb = lo + lf;
    scatter<<<blocksFor(m), kBlock, 0, st>>>(raw_ptr(s.vox), raw_ptr(s.flags), raw_ptr(s.offs), m, raw_ptr(s.band));
  }
  checkCudaErrors(cudaEventRecord(s.ev[6], st));

  if (nb > 0) {
    grow(s.labels, nb);
    initLabels<<<blocksFor(nb), kBlock, 0, st>>>(raw_ptr(s.labels), nb);
    const float tol2 = P.cluster_tolerance * P.cluster_tolerance;
    for (int it = 0; it < 1000; it++) {
      int changed = 0;
      checkCudaErrors(cudaMemsetAsync(raw_ptr(s.changed), 0, sizeof(int), st));
      propagate<<<blocksFor(nb), kBlock, 0, st>>>(raw_ptr(s.band), nb, tol2, raw_ptr(s.labels), raw_ptr(s.changed));
      jump<<<blocksFor(nb), kBlock, 0, st>>>(raw_ptr(s.labels), nb);
      checkCudaErrors(cudaMemcpyAsync(&changed, raw_ptr(s.changed), sizeof(int), cudaMemcpyDeviceToHost, st));
      checkCudaErrors(cudaStreamSynchronize(st));
      if (!changed) break;
    }
  }
  checkCudaErrors(cudaEventRecord(s.ev[7], st));

  out.points.resize(static_cast<size_t>(nb) * 4);
  out.labels.resize(nb);
  if (nb > 0) {
    checkCudaErrors(cudaMemcpyAsync(out.points.data(), raw_ptr(s.band), nb * sizeof(float4), cudaMemcpyDeviceToHost, st));
    checkCudaErrors(cudaMemcpyAsync(out.labels.data(), raw_ptr(s.labels), nb * sizeof(int), cudaMemcpyDeviceToHost, st));
  }
  checkCudaErrors(cudaEventRecord(s.ev[8], st));
  checkCudaErrors(cudaStreamSynchronize(st));
  checkCudaErrors(cudaGetLastError());

  // dense ids for clusters within [min, max]; others get -1 (PCL drops them)
  std::unordered_map<int, int> size, id;
  for (int l : out.labels) size[l]++;
  for (int &l : out.labels) {
    const int sz = size[l];
    if (sz < P.min_cluster_size || sz > P.max_cluster_size) { l = -1; continue; }
    auto it = id.find(l);
    if (it == id.end()) it = id.emplace(l, static_cast<int>(id.size())).first;
    l = it->second;
  }
  out.n_clusters = static_cast<int>(id.size());

  float *t[] = {&out.t.h2d, &out.t.unpack, &out.t.roi, &out.t.voxel, &out.t.ransac, &out.t.band, &out.t.cluster, &out.t.d2h};
  for (int k = 0; k < 8; k++) cudaEventElapsedTime(t[k], s.ev[k], s.ev[k + 1]);
}
