cmake_minimum_required(VERSION 3.8)
project(servo_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)

add_executable(ur_servo_keyboard_input_node src/ur_servo_keyboard_input.cpp)
add_executable(servo_node src/servo_interface.cpp)

ament_target_dependencies(ur_servo_keyboard_input_node rclcpp geometry_msgs control_msgs)
ament_target_dependencies(servo_node rclcpp moveit_servo moveit_ros_planning_interface)

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
install(TARGETS
 ur_servo_keyboard_input_node servo_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()
