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
target_include_directories(lidar_perception_node PRIVATE
  include
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(lidar_perception_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# GPU path: CUDA filters (src/gpu, PASSTHROUGH + VOXELGRID) built for the local GPU.
# cuda-pcl is NOT used: its prebuilt libraries are aarch64 (Jetson) only.
# If no CUDA compiler is found, lidar_perception_node is built CPU-only (PCL).
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  find_package(CUDAToolkit REQUIRED)

  # Target SM. Desktop default is 120 (RTX 50-series, Blackwell); needs CUDA >= 12.8.
  # Jetson Orin Nano is SM 87. Override with -DLIDAR_CUDA_ARCH=<n>.
  if(EXISTS "/etc/nv_tegra_release")
    set(LIDAR_CUDA_ARCH_DEFAULT 87)
  else()
    set(LIDAR_CUDA_ARCH_DEFAULT 120)
  endif()
  set(LIDAR_CUDA_ARCH ${LIDAR_CUDA_ARCH_DEFAULT} CACHE STRING "CUDA SM architecture for lidar_gpu_filter")

  set(LIDAR_GPU_OK ON)
  if(LIDAR_CUDA_ARCH GREATER_EQUAL 120 AND CMAKE_CUDA_COMPILER_VERSION VERSION_LESS 12.8)
    message(WARNING "nvcc ${CMAKE_CUDA_COMPILER_VERSION} cannot target sm_${LIDAR_CUDA_ARCH} (needs CUDA >= 12.8): "
                    "building lidar_perception_node CPU-only")
    set(LIDAR_GPU_OK OFF)
  endif()
endif()

if(CMAKE_CUDA_COMPILER AND LIDAR_GPU_OK)
  add_library(lidar_gpu_filter STATIC src/gpu/cuda_filter.cu)
  target_include_directories(lidar_gpu_filter PUBLIC include)
  target_link_libraries(lidar_gpu_filter PUBLIC CUDA::cudart)
  target_compile_options(lidar_gpu_filter PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wno-pedantic>)
  set_target_properties(lidar_gpu_filter PROPERTIES
    CUDA_ARCHITECTURES ${LIDAR_CUDA_ARCH}
    POSITION_INDEPENDENT_CODE ON)

  target_link_libraries(lidar_perception_node lidar_gpu_filter)
  target_compile_definitions(lidar_perception_node PRIVATE PLATFORM_DESKTOP_GPU)

  # Offline CPU-vs-GPU equivalence + latency benchmark (not installed).
  add_executable(lidar_gpu_bench bench/lidar_gpu_bench.cpp)
  target_include_directories(lidar_gpu_bench PRIVATE ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(lidar_gpu_bench lidar_gpu_filter ${PCL_LIBRARIES})
  message(STATUS "lidar_perception_node: GPU filters enabled (CUDA ${CMAKE_CUDA_COMPILER_VERSION}, sm_${LIDAR_CUDA_ARCH})")
elseif(NOT CMAKE_CUDA_COMPILER)
  message(STATUS "No CUDA compiler found: lidar_perception_node will use CPU PCL only")
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
  detected_cones
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
