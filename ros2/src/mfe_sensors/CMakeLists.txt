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
find_package(fmt REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pointcloud_to_laserscan REQUIRED)

add_executable(sim_lidar_node src/sim_lidar_node.cpp)

add_library(${PROJECT_NAME} SHARED
  src/camera_interface.cpp
  src/lidar_interface.cpp
  src/sensor_node.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>" 
)

target_link_libraries(sim_lidar_node
  PUBLIC
    fmt::fmt
    ${PROJECT_NAME}        # this propagates the include dirs from your library
)

ament_target_dependencies(sim_lidar_node
  PUBLIC
    rclcpp
    fmt
    rclcpp_lifecycle
    sensor_msgs
    pointcloud_to_laserscan
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    rclcpp
    fmt
    rclcpp_lifecycle
    sensor_msgs
    pointcloud_to_laserscan
)

install(
  TARGETS     ${PROJECT_NAME}
  EXPORT      export_${PROJECT_NAME}
  ARCHIVE     DESTINATION lib
  LIBRARY     DESTINATION lib
  RUNTIME     DESTINATION bin  
)

install(TARGETS sim_lidar_node DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)

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
