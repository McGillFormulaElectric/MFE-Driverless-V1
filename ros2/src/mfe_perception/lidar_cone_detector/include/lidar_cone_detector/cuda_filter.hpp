// CUDA point-cloud filters (PASSTHROUGH and VOXELGRID) for lidar_perception_node.
//
// Drop-in replacement for the subset of cuda-pcl's cudaFilter API that the node
// uses. cuda-pcl only ships aarch64 (Jetson) binaries, so this implementation is
// built from source by CMake for the local GPU (see CMakeLists.txt).
//
// Point layout: each point is 4 floats (x, y, z, pad), i.e. pcl::PointXYZ.
// Device pointers may be plain device or managed memory.
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
  // type PASSTHROUGH: keep points with downFilterLimits <= v <= upFilterLimits
  //                   (inverted when limitsNegative is true)
  int dim;
  float upFilterLimits;
  float downFilterLimits;
  bool limitsNegative;
  // type VOXELGRID: one output point per occupied voxel, located at the centroid
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

  // source: nCount points (4 floats each). output: room for nCount points.
  // countLeft: host pointer, receives the number of points written to output.
  // Output order is stable for PASSTHROUGH and voxel-index order for VOXELGRID.
  // VOXELGRID expects finite input points (the caller strips NaNs).
  int filter(void *output, unsigned int *countLeft, void *source, unsigned int nCount);

private:
  struct Impl;
  Impl *impl_;
};
