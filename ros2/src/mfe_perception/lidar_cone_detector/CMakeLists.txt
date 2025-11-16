cmake_minimum_required(VERSION 3.8)
project(lidar_cone_detector)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Core ament
find_package(ament_cmake REQUIRED)

# ROS 2 deps
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# If you publish Cone/Track:
find_package(mfe_msgs REQUIRED)

# Eigen (via helper module if you need it)
find_package(eigen3_cmake_module QUIET)
find_package(Eigen3 REQUIRED)

# PCL + conversions
find_package(pcl_conversions REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io filters segmentation visualization)

# Optional other deps
# find_package(pointcloud_to_laserscan QUIET)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ---- Executables ----
add_executable(cone_detector_node
  src/cone_detector.cpp
  src/dbscan.cpp
)
ament_target_dependencies(cone_detector_node
  rclcpp sensor_msgs geometry_msgs std_msgs pcl_conversions mfe_msgs
)
target_link_libraries(cone_detector_node ${PCL_LIBRARIES})

add_executable(lidar_preprocessor src/lidar_preprocessor.cpp)
ament_target_dependencies(lidar_preprocessor
  rclcpp sensor_msgs pcl_conversions
)
target_link_libraries(lidar_preprocessor ${PCL_LIBRARIES})

add_executable(ground_plane_removal src/ground_plane_removal.cpp)
ament_target_dependencies(ground_plane_removal
  rclcpp sensor_msgs pcl_conversions
)
target_link_libraries(ground_plane_removal ${PCL_LIBRARIES})

add_executable(file_loader src/file_loader.cpp)

ament_target_dependencies(file_loader
  rclcpp sensor_msgs pcl_conversions
)
target_link_libraries(file_loader ${PCL_LIBRARIES})
# ---- Install ----
install(TARGETS
  cone_detector_node
  lidar_preprocessor
  ground_plane_removal
  file_loader
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch dataset DESTINATION share/${PROJECT_NAME})

# ---- Tests (optional; keep minimal) ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
endif()

ament_package()
