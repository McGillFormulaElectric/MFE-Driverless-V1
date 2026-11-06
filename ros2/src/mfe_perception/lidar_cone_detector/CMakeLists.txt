cmake_minimum_required(VERSION 3.8)
project(lidar_cone_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# eigen3_cmake_module: plain Eigen3 config does not work with ament_cmake.
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

target_link_libraries(ground_plane_removal ${PCL_LIBRARIES})

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

# The GPU path uses the CUDA kernels in src/gpu, built for the local GPU; no external CUDA library is needed.
# Without a CUDA compiler, lidar_perception_node is built CPU-only.
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  find_package(CUDAToolkit REQUIRED)

  # Desktop default: SM 75 (GTX 16 / RTX 20) and SM 120 (Blackwell, needs CUDA >= 12.8); Jetson Orin Nano is SM 87.
  # Override with -DLIDAR_CUDA_ARCH="<n>;<n>".
  if(EXISTS "/etc/nv_tegra_release")
    set(LIDAR_CUDA_ARCH_DEFAULT 87)
  else()
    set(LIDAR_CUDA_ARCH_DEFAULT "75;120")
  endif()
  set(LIDAR_CUDA_ARCH ${LIDAR_CUDA_ARCH_DEFAULT} CACHE STRING "CUDA SM architectures for lidar_gpu_filter")

  set(LIDAR_CUDA_ARCHS ${LIDAR_CUDA_ARCH})
  if("120" IN_LIST LIDAR_CUDA_ARCHS AND CMAKE_CUDA_COMPILER_VERSION VERSION_LESS 12.8)
    message(WARNING "nvcc ${CMAKE_CUDA_COMPILER_VERSION} cannot target sm_120 (needs CUDA >= 12.8): dropping it")
    list(REMOVE_ITEM LIDAR_CUDA_ARCHS 120)
  endif()
  set(LIDAR_GPU_OK ON)
  if(NOT LIDAR_CUDA_ARCHS)
    message(WARNING "no CUDA architecture left: building lidar_perception_node CPU-only")
    set(LIDAR_GPU_OK OFF)
  endif()
endif()

if(CMAKE_CUDA_COMPILER AND LIDAR_GPU_OK)
  add_library(lidar_gpu_filter STATIC src/gpu/cuda_filter.cu src/gpu/cuda_cone_pipeline.cu)
  target_include_directories(lidar_gpu_filter PUBLIC include)
  target_link_libraries(lidar_gpu_filter PUBLIC CUDA::cudart)
  target_compile_options(lidar_gpu_filter PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wno-pedantic>)
  set_target_properties(lidar_gpu_filter PROPERTIES
    CUDA_ARCHITECTURES "${LIDAR_CUDA_ARCHS}"
    POSITION_INDEPENDENT_CODE ON)

  target_link_libraries(lidar_perception_node lidar_gpu_filter)
  target_compile_definitions(lidar_perception_node PRIVATE PLATFORM_DESKTOP_GPU)

  message(STATUS "lidar_perception_node: GPU filters enabled (CUDA ${CMAKE_CUDA_COMPILER_VERSION}, sm ${LIDAR_CUDA_ARCHS})")
elseif(NOT CMAKE_CUDA_COMPILER)
  message(STATUS "No CUDA compiler found: lidar_perception_node will use CPU PCL only")
endif()

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  config
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
  # No license headers yet: skip copyright and cpplint checks.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_motion_distortion test/test_motion_distortion.cpp)
  target_include_directories(test_motion_distortion PRIVATE include)
  if(CMAKE_CUDA_COMPILER AND LIDAR_GPU_OK)
    target_link_libraries(test_motion_distortion lidar_gpu_filter)
    target_compile_definitions(test_motion_distortion PRIVATE LIDAR_GPU_TEST)
  endif()
endif()

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_auto_package()

