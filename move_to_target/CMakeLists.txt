cmake_minimum_required(VERSION 3.8)
project(move_to_target)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Target: move_to_target
add_executable(move_to_target src/move_to_target.cpp)
target_include_directories(move_to_target PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_to_target PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  move_to_target
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Target: move_pose
add_executable(move_pose src/move_pose.cpp)
target_include_directories(move_pose PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_pose PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  move_pose
  "moveit_ros_planning_interface"
  "rclcpp"
)

# Target: read_joint_states
add_executable(read_joint_states src/read_jointstates.cpp)
target_include_directories(read_joint_states PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(read_joint_states PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  read_joint_states
  "moveit_ros_planning_interface"
  "rclcpp"
  "sensor_msgs"
  )
  

add_executable(move_waypoints src/move_waypoints.cpp)

  # Specify the target's include directories
target_include_directories(move_waypoints PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  
  # Set required C++ standard
target_compile_features(move_waypoints PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(
  move_waypoints
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "tf2"
)

# Install all executables
install(TARGETS move_to_target move_pose read_joint_states move_waypoints
  DESTINATION lib/${PROJECT_NAME})

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
