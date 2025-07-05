cmake_minimum_required(VERSION 3.8)
project(rom_add_collision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED) 
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GestureCommand.srv"
  DEPENDENCIES std_msgs
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

add_executable(add_table_node src/0_add_table_collision.cpp)
ament_target_dependencies(add_table_node rclcpp moveit_msgs geometry_msgs shape_msgs tf2 tf2_geometry_msgs)

add_executable(hello_moveit src/1_hello_moveit.cpp)
ament_target_dependencies(hello_moveit rclcpp moveit_ros_planning_interface)

add_executable(gesture_moveit src/3_gesture_moveit_service.cpp)
ament_target_dependencies(gesture_moveit rclcpp moveit_ros_planning_interface std_msgs tf2 tf2_geometry_msgs geometry_msgs moveit_core)
ament_export_dependencies(rosidl_default_runtime)
target_link_libraries(gesture_moveit
  "${PROJECT_NAME}__rosidl_typesupport_cpp" # This is crucial for linking custom messages/services
)
ament_export_dependencies(
  rosidl_default_runtime
)

add_executable(service_caller src/4_service_caller.cpp)
ament_target_dependencies(service_caller rclcpp std_msgs moveit_ros_planning_interface)
target_link_libraries(service_caller
  "${PROJECT_NAME}__rosidl_typesupport_cpp" # This is crucial for linking custom messages/services
)

add_executable(arm_controller_for_ui src/5_arm_controller_from_ui.cpp)
ament_target_dependencies(arm_controller_for_ui rclcpp moveit_ros_planning_interface tf2 tf2_geometry_msgs)

add_executable(test_controller src/6_arm_controller_from_ui_test_publisher.cpp)
ament_target_dependencies(test_controller rclcpp moveit_ros_planning_interface)

# add_executable(pose_goal_publisher src/go_pose_publisher.cpp)
# ament_target_dependencies(pose_goal_publisher
#   rclcpp
#   moveit_ros_planning_interface
#   geometry_msgs
#   moveit_core
#   moveit_visual_tools
# )

install(TARGETS
  add_table_node
  hello_moveit
  gesture_moveit
  service_caller
  arm_controller_for_ui
  test_controller
  #pose_goal_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
