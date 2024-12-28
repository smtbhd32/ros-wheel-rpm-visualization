cmake_minimum_required(VERSION 3.5)
project(assignment)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories (if you have headers in 'include')
include_directories(include)

# Add the executable for Script A (C++ code)
add_executable(script_a_node src/script_a/script_a.cpp)

# Add the executable for Script B (C++ code)
add_executable(script_b_node src/script_b/script_b.cpp)


# Link dependencies to Script A (needs rclcpp and geometry_msgs)
ament_target_dependencies(script_a_node rclcpp geometry_msgs)

# Link dependencies to Script B (only needs rclcpp)
ament_target_dependencies(script_b_node rclcpp)

# Install the C++ executables
install(
  TARGETS script_a_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS script_b_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export package information
ament_package()
