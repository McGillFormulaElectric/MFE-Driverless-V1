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
# eigen3 has dependencies that don't work with ament_cmake, use side package to import connection
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pointcloud_to_laserscan REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDA QUIET)



if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "DETECTED JETSON (ARM64)")
  add_compile_definitions(PLATFORM_JETSON)

  find_package(CUDA REQUIRED)
  set(PLATFORM_LIBS ${CUDA_LIBRARIES} ${PCL_LIBRARIES})

elseif(CUDA_FOUND)
  message(STATUS "DETECTED NVIDIA GPU")
  add_compile_definitions(PLATFORM_DESKTOP_GPU)
  set(PLATFORM_LIBS ${CUDA_LIBRARIES} ${PCL_LIBRARIES})

else()
  message(STATUS "CPU FALLBACK")
  add_compile_definitions(PLATFORM_CPU)
  set(PLATFORM_LIBS ${PCL_LIBRARIES})
endif()

include_directories(include ${PCL_INCLUDE_DIRS})

add_executable(lidar_preprocessor src/lidar_preprocessor.cpp)
ament_target_dependencies(lidar_preprocessor rclcpp sensor_msgs pointcloud_to_laserscan)
target_link_libraries(lidar_preprocessor ${PLATFORM_LIBS})

add_executable(ground_plane_removal src/ground_plane_removal.cpp)
ament_target_dependencies(ground_plane_removal rclcpp sensor_msgs)
target_link_libraries(ground_plane_removal ${PLATFORM_LIBS})

add_executable(file_loader src/file_loader.cpp)
ament_target_dependencies(file_loader rclcpp sensor_msgs)
target_link_libraries(file_loader ${PCL_LIBRARIES})

add_executable(detected_cones src/cone_detector.cpp)
ament_target_dependencies(detected_cones rclcpp sensor_msgs)
target_link_libraries(detected_cones ${PLATFORM_LIBS})

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


# tests
if(AMENT_ENABLE_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(foo_gtest test/my_test.cpp)
  target_link_libraries(foo_gtest ${rclcpp_LIBRARIES} ${rmw_connext_cpp_LIBRARIES} ${std_interfaces})
endif()

ament_package()
