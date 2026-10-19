// CUDA PASSTHROUGH and VOXELGRID filters (see cuda_filter.hpp).
// PASSTHROUGH: flags -> exclusive scan -> scatter (stable compaction).
// VOXELGRID: voxel key -> sort_by_key -> reduce_by_key -> centroid.
// Grow-only scratch buffers: no cudaMalloc per frame.

#include "lidar_cone_detector/cuda_filter.hpp"

#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/scan.h>
#include <thrust/sort.h>
#include <thrust/reduce.h>
#include <thrust/functional.h>
#include <thrust/system/cuda/execution_policy.h>

#include <cstdint>

namespace {

constexpr int kBlock = 256;
constexpr int64_t kVoxelBias = 1 << 20;   // voxel index offset; 21 bits per axis
constexpr uint64_t kVoxelMask = (1ull << 21) - 1;

inline unsigned blocksFor(unsigned n) { return (n + kBlock - 1) / kBlock; }

__global__ void passthroughFlags(const float4 *in, unsigned n, int dim, float lo, float hi,
                                 bool negative, unsigned *flags) {
  unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const float4 p = in[i];
  const float v = (dim == 0) ? p.x : (dim == 1) ? p.y : p.z;
  const bool inside = (v >= lo) && (v <= hi);
  flags[i] = (inside != negative) ? 1u : 0u;
}

__global__ void scatterKept(const float4 *in, const unsigned *flags, const unsigned *offs,
                            unsigned n, float4 *out) {
  unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n && flags[i]) out[offs[i]] = in[i];
}

__global__ void voxelKeys(const float4 *in, unsigned n, float3 inv, uint64_t *keys, double4 *vals) {
  unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= n) return;
  const float4 p = in[i];
  // floor(p * inv_leaf) as in pcl::VoxelGrid; same partition up to a per-axis integer shift.
  const int64_t ix = static_cast<int64_t>(floorf(p.x * inv.x)) + kVoxelBias;
  const int64_t iy = static_cast<int64_t>(floorf(p.y * inv.y)) + kVoxelBias;
  const int64_t iz = static_cast<int64_t>(floorf(p.z * inv.z)) + kVoxelBias;
  keys[i] = ((static_cast<uint64_t>(ix) & kVoxelMask) << 42) |
            ((static_cast<uint64_t>(iy) & kVoxelMask) << 21) |
            (static_cast<uint64_t>(iz) & kVoxelMask);
  vals[i] = make_double4(p.x, p.y, p.z, 1.0);
}

__global__ void voxelCentroids(const double4 *sums, unsigned m, float4 *out) {
  unsigned i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= m) return;
  const double4 s = sums[i];
  const double inv = 1.0 / s.w;
  out[i] = make_float4(static_cast<float>(s.x * inv), static_cast<float>(s.y * inv),
                       static_cast<float>(s.z * inv), 1.0f);
}

struct AddDouble4 {
  __host__ __device__ double4 operator()(const double4 &a, const double4 &b) const {
    return make_double4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
  }
};

}  // namespace

struct cudaFilter::Impl {
  cudaStream_t stream;
  FilterParam_t param{};
  thrust::device_vector<unsigned> flags;
  thrust::device_vector<unsigned> offsets;
  thrust::device_vector<uint64_t> keys;
  thrust::device_vector<double4> vals;
  thrust::device_vector<uint64_t> outKeys;
  thrust::device_vector<double4> outVals;
};

cudaFilter::cudaFilter(cudaStream_t stream) : impl_(new Impl) { impl_->stream = stream; }

cudaFilter::~cudaFilter() { delete impl_; }

int cudaFilter::set(FilterParam_t param) {
  impl_->param = param;
  return 0;
}

int cudaFilter::filter(void *output, unsigned int *countLeft, void *source, unsigned int nCount) {
  Impl &s = *impl_;
  cudaStream_t st = s.stream;
  const auto *in = static_cast<const float4 *>(source);
  auto *out = static_cast<float4 *>(output);
  *countLeft = 0;
  if (nCount == 0) return 0;

  if (s.param.type == PASSTHROUGH) {
    s.flags.resize(nCount);
    s.offsets.resize(nCount);
    unsigned *flags = thrust::raw_pointer_cast(s.flags.data());
    unsigned *offs = thrust::raw_pointer_cast(s.offsets.data());
    passthroughFlags<<<blocksFor(nCount), kBlock, 0, st>>>(
        in, nCount, s.param.dim, s.param.downFilterLimits, s.param.upFilterLimits,
        s.param.limitsNegative, flags);
    thrust::exclusive_scan(thrust::cuda::par.on(st), flags, flags + nCount, offs);
    unsigned lastOff = 0, lastFlag = 0;
    checkCudaErrors(cudaMemcpyAsync(&lastOff, offs + nCount - 1, sizeof(unsigned),
                                    cudaMemcpyDeviceToHost, st));
    checkCudaErrors(cudaMemcpyAsync(&lastFlag, flags + nCount - 1, sizeof(unsigned),
                                    cudaMemcpyDeviceToHost, st));
    checkCudaErrors(cudaStreamSynchronize(st));
    scatterKept<<<blocksFor(nCount), kBlock, 0, st>>>(in, flags, offs, nCount, out);
    checkCudaErrors(cudaStreamSynchronize(st));
    checkCudaErrors(cudaGetLastError());
    *countLeft = lastOff + lastFlag;
    return 0;
  }

  const float3 inv = make_float3(1.0f / s.param.voxelX, 1.0f / s.param.voxelY, 1.0f / s.param.voxelZ);
  s.keys.resize(nCount);
  s.vals.resize(nCount);
  s.outKeys.resize(nCount);
  s.outVals.resize(nCount);
  voxelKeys<<<blocksFor(nCount), kBlock, 0, st>>>(in, nCount, inv,
                                                  thrust::raw_pointer_cast(s.keys.data()),
                                                  thrust::raw_pointer_cast(s.vals.data()));
  thrust::sort_by_key(thrust::cuda::par.on(st), s.keys.begin(), s.keys.begin() + nCount,
                      s.vals.begin());
  auto ends = thrust::reduce_by_key(thrust::cuda::par.on(st), s.keys.begin(),
                                    s.keys.begin() + nCount, s.vals.begin(), s.outKeys.begin(),
                                    s.outVals.begin(), thrust::equal_to<uint64_t>(), AddDouble4());
  const unsigned m = static_cast<unsigned>(ends.first - s.outKeys.begin());
  voxelCentroids<<<blocksFor(m), kBlock, 0, st>>>(thrust::raw_pointer_cast(s.outVals.data()), m,
                                                  out);
  checkCudaErrors(cudaStreamSynchronize(st));
  checkCudaErrors(cudaGetLastError());
  *countLeft = m;
  return 0;
}
