cmake_minimum_required(VERSION 3.8)
project(aruco_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclpy REQUIRED)

# Add C++ executable
add_executable(aruco_cpp_node src/aruco_cpp_node.cpp)
ament_target_dependencies(aruco_cpp_node rclcpp std_msgs)

# Install the C++ executable
install(TARGETS aruco_cpp_node
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/aruco_python_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Test setup (if needed)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks if no copyright/license
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Enable ament_python support and package definition
ament_python_install_package(${PROJECT_NAME})

# Define package as an ament package
ament_package()

