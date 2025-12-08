cmake_minimum_required(VERSION 3.8)
project(lidar_cone_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# eigen3 has dependencies that don't work with ament_cmake, use side package to import connection
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pointcloud_to_laserscan REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDA QUIET)

include_directories(include ${PCL_INCLUDE_DIRS})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "DETECTED JETSON (ARM64)")
  set(CUPCL_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64")
else()
  message(STATUS "USING NVIDIA GPU")
  set(CUPCL_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64")
endif()

# add_executable(lidar_preprocessor src/lidar_preprocessor.cpp)
# ament_target_dependencies(lidar_preprocessor rclcpp sensor_msgs pointcloud_to_laserscan)
# target_link_libraries(lidar_preprocessor ${PLATFORM_LIBS})
# 
# add_executable(ground_plane_removal src/ground_plane_removal.cpp)
# ament_target_dependencies(ground_plane_removal rclcpp sensor_msgs)
# target_link_libraries(ground_plane_removal ${PLATFORM_LIBS})

add_library(cu_segmentation SHARED IMPORTED)
set_target_properties(cu_segmentation PROPERTIES IMPORTED_LOCATION "${CUPCL_LIB_DIR}/libcudasegmentation.so")

add_library(cu_cluster SHARED IMPORTED)
set_target_properties(cu_cluster PROPERTIES IMPORTED_LOCATION "${CUPCL_LIB_DIR}/libcudacluster.so")

add_executable(file_loader src/file_loader.cpp)
ament_target_dependencies(file_loader rclcpp sensor_msgs pcl_conversions)
target_link_libraries(file_loader ${PCL_LIBRARIES})

# add_executable(detected_cones src/cone_detector.cpp)
# ament_target_dependencies(detected_cones rclcpp sensor_msgs)
# target_link_libraries(detected_cones ${PLATFORM_LIBS})

add_executable(lidar_processor src/lidar_processor.cpp)
ament_target_dependencies(lidar_processor rclcpp sensor_msgs pcl_conversions)
target_link_libraries(lidar_processor ${PCL_LIBRARIES} ${CUDA_LIBRARIES} cu_segmentation cu_cluster)





# ament_auto_find_build_dependencies()
# 
# ament_auto_generate_code()
# 
# ament_auto_add_library(${project_name}
#   directory src
# )
# 
# ament_auto_add_executable(lidar_preprocessor
#   src/lidar_preprocessor.cpp
# )
# 
# ament_auto_add_executable(ground_plane_removal
#   src/ground_plane_removal.cpp
# )
# 
# ament_auto_add_executable(file_loader
#   src/file_loader.cpp
# )
# 
# ament_auto_add_executable(detected_cones
#   src/cone_detector.cpp
# )
# target_link_libraries(ground_plane_removal ${pcl_libraries})

ament_python_install_package(${PROJECT_NAME})

 install(PROGRAMS
   scripts/cone_detector_node.py
   DESTINATION lib/${PROJECT_NAME}
   RENAME cone_detector_node
 )

install(DIRECTORY
  launch
  dataset
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  file_loader
  lidar_processor
#  lidar_preprocessor
#  ground_plane_removal
#  detected_cones
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  "${CUPCL_LIB_DIR}/libcudasegmentation.so"
  "${CUPCL_LIB_DIR}/libcudacluster.so"
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

ament_package()
