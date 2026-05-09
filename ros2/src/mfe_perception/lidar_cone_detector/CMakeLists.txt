cmake_minimum_required(VERSION 3.8)
project(lidar_cone_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# eigen3 has dependencies that don't work with ament_cmake, use side package to import connection
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(pointcloud_to_laserscan REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_generate_code()

ament_auto_add_library(${PROJECT_NAME}
  DIRECTORY src
)

ament_auto_add_executable(lidar_preprocessor
  src/lidar_preprocessor.cpp
)

ament_auto_add_executable(ground_plane_removal
  src/ground_plane_removal.cpp
)

ament_auto_add_executable(file_loader
  src/file_loader.cpp
)

ament_auto_add_executable(detected_cones
  src/cone_detector.cpp
)
target_link_libraries(ground_plane_removal ${PCL_LIBRARIES})

# ---- lidar_perception_node (new unified perception pipeline) ----
ament_auto_add_executable(lidar_perception_node
  src/lidar_perception_node.cpp
)
target_link_libraries(lidar_perception_node ${PCL_LIBRARIES})

# GPU path: auto-detect CUDA toolkit + cuda-pcl, fall back to CPU PCL if either is missing.
find_package(CUDA QUIET)
if(CUDA_FOUND)
  message(STATUS "CUDA found: ${CUDA_VERSION} — looking for cuda-pcl")

  # Locate cuda-pcl headers (installed to /usr/local/include by `make install`)
  find_path(CUDA_PCL_INCLUDE_DIR cudaFilter.h
      HINTS /usr/local/include /opt/cuda-pcl/cudaFilter
      PATH_SUFFIXES cuda-pcl)

  # Locate the three cuda-pcl shared libraries
  find_library(CUDA_FILTER_LIB  cudaFilter     HINTS /usr/local/lib /opt/cuda-pcl/build)
  find_library(CUDA_SEG_LIB     cudaSegmentation HINTS /usr/local/lib /opt/cuda-pcl/build)
  find_library(CUDA_CLUSTER_LIB cudaCluster    HINTS /usr/local/lib /opt/cuda-pcl/build)

  if(CUDA_PCL_INCLUDE_DIR AND CUDA_FILTER_LIB AND CUDA_SEG_LIB AND CUDA_CLUSTER_LIB)
    message(STATUS "cuda-pcl found — GPU pipeline fully enabled")

    if(DEFINED PLATFORM_JETSON OR EXISTS "/etc/nv_tegra_release")
      set(CUDA_ARCH "87")   # Jetson Orin Nano — Ampere SM87
      target_compile_definitions(lidar_perception_node PRIVATE PLATFORM_JETSON)
      message(STATUS "Platform: Jetson (SM${CUDA_ARCH})")
    else()
      if(NOT DEFINED CUDA_ARCH)
        set(CUDA_ARCH "native")
      endif()
      target_compile_definitions(lidar_perception_node PRIVATE PLATFORM_DESKTOP_GPU)
      message(STATUS "Platform: Desktop GPU (arch=${CUDA_ARCH})")
    endif()

    set(CMAKE_CUDA_ARCHITECTURES ${CUDA_ARCH})

    target_include_directories(lidar_perception_node PRIVATE
        ${CUDA_INCLUDE_DIRS}
        ${CUDA_PCL_INCLUDE_DIR})

    target_link_libraries(lidar_perception_node
        ${CUDA_LIBRARIES}
        ${CUDA_FILTER_LIB}
        ${CUDA_SEG_LIB}
        ${CUDA_CLUSTER_LIB})
  else()
    message(WARNING "CUDA found but cuda-pcl not found — falling back to CPU PCL. "
        "Build cuda-pcl from https://github.com/NVIDIA-AI-IOT/cuda-pcl and install to /usr/local.")
  endif()
else()
  message(STATUS "CUDA not found — lidar_perception_node will use CPU PCL only")
endif()

ament_python_install_package(${PROJECT_NAME})

# install(PROGRAMS
#   scripts/cone_detector_node.py
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME cone_detector_node
# )

install(DIRECTORY
  launch
  dataset
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  lidar_preprocessor
  ground_plane_removal
  file_loader
  lidar_perception_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_auto_package()

# tests
if(AMENT_ENABLE_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(foo_gtest test/my_test.cpp)
  target_link_libraries(foo_gtest ${rclcpp_LIBRARIES} ${rmw_connext_cpp_LIBRARIES} ${std_interfaces})
endif()
