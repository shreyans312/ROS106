cmake_minimum_required(VERSION 3.8)
project(ros2_assignment_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotPose2D.msg"
  "srv/GetRobotPosition.srv"
  DEPENDENCIES std_msgs
)

# Install Python executables from the inner package directory
install(PROGRAMS
  ros2_assignment_pkg/pose_filter_node.py
  ros2_assignment_pkg/transform_service_node.py
  ros2_assignment_pkg/test_publisher.py
  ros2_assignment_pkg/test_client.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install message and service files
install(DIRECTORY
  msg
  srv
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()