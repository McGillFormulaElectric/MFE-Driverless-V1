// CUDA PASSTHROUGH and VOXELGRID filters: drop-in for the cuda-pcl cudaFilter subset the node uses.
// Built from source because cuda-pcl only ships aarch64 binaries.
// Points are 4 floats (x, y, z, pad), i.e. pcl::PointXYZ. Device pointers may be device or managed memory.
#pragma once

#include <cstdio>
#include <iostream>
#include <cuda_runtime.h>

#define checkCudaErrors(status)                                       \
{                                                                     \
  cudaError_t _err = (status);                                        \
  if (_err != cudaSuccess)                                            \
  {                                                                   \
    std::cerr << "Cuda failure: " << cudaGetErrorString(_err)         \
              << " at line " << __LINE__                              \
              << " in file " << __FILE__                              \
              << " error status: " << _err << std::endl;              \
    abort();                                                          \
  }                                                                   \
}

typedef enum {
  PASSTHROUGH = 0,
  VOXELGRID = 1,
} FilterType_t;

typedef struct {
  FilterType_t type;
  // 0=x, 1=y, 2=z
  // PASSTHROUGH: keep downFilterLimits <= v <= upFilterLimits (inverted if limitsNegative)
  int dim;
  float upFilterLimits;
  float downFilterLimits;
  bool limitsNegative;
  // VOXELGRID: one centroid per occupied voxel
  float voxelX;
  float voxelY;
  float voxelZ;
} FilterParam_t;

class cudaFilter {
public:
  explicit cudaFilter(cudaStream_t stream = 0);
  ~cudaFilter();
  cudaFilter(const cudaFilter &) = delete;
  cudaFilter &operator=(const cudaFilter &) = delete;

  int set(FilterParam_t param);

  // output needs room for nCount points; countLeft (host) gets the count written.
  // PASSTHROUGH keeps input order; VOXELGRID outputs in voxel-index order and needs finite input.
  int filter(void *output, unsigned int *countLeft, void *source, unsigned int nCount);

private:
  struct Impl;
  Impl *impl_;
};
