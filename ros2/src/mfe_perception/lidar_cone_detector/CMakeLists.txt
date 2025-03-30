cmake_minimum_required(VERSION 3.8)
project(lidar_cone_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl REQUIRED)

# eigen3 has dependencies that don't work with ament_cmake, use side package to import connection
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

ament_target_dependencies(rclcpp tf2_ros tf2 tf2_eigen sensor_msgs vision_msgs geometry_msgs pcl_ros pcl pcl_conversions)
target_link_libraries(your_target ${PCL_LIBRARIES})

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

ament_package()

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
