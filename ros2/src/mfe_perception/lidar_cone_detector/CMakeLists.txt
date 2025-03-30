cmake_minimum_required(VERSION 3.8)
project(lidar_cone_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# eigen3 has dependencies that don't work with ament_cmake, use side package to import connection
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

ament_auto_generate_code()

ament_auto_add_library(lidar_cone_detector
  src/ground_plane_removal.cpp
)

ament_target_dependencies(lidar_cone_detector
  tf2  
  tf2_ros
  sensor_msgs 
  vision_msgs 
  geometry_msgs 
  pcl_ros 
  pcl_conversions
)
# target_link_libraries(lidar_cone_detector ${PCL_LIBRARIES})

install(TARGETS
  lidar_cone_detector
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