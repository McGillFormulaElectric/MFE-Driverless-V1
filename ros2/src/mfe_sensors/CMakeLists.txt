cmake_minimum_required(VERSION 3.8)
project(mfe_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/camera_driver_node.cpp
  src/lidar_driver_node.cpp
  src/base_driver_node.cpp
  src/realsense_camera_component.cpp
  src/sim_camera_component.cpp
  src/velodyne_lidar_component.cpp
  src/sim_lidar_component.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>" 
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    rclcpp
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)

install(
  TARGETS     ${PROJECT_NAME}
  EXPORT      export_${PROJECT_NAME}
  ARCHIVE     DESTINATION lib
  LIBRARY     DESTINATION lib
  RUNTIME     DESTINATION bin  
)

install(
  DIRECTORY include/
  destination include/${PROJECT_NAME}
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

ament_package()
