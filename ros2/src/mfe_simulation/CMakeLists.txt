cmake_minimum_required(VERSION 3.8)
project(mfe_simulation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_auto_find_build_dependencies()

# Install Python executable scripts
install(PROGRAMS
  mfe_simulation/carmaker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# The above installs the my_node.py script as an executable in the package's "lib" directory&#8203;:contentReference[oaicite:13]{index=13}.
# By installing to lib/my_python_pkg, ROS 2 will be able to locate it with ros2 run.

# Install launch files
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python packages (mfe_simulation/ folder with __init__.py, carmaker_interface.py, etc.)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# ament_auto_package() # Add back if needed for C++ components
