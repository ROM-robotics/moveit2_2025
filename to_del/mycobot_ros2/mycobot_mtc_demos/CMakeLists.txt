cmake_minimum_required(VERSION 3.8)
project(mycobot_mtc_demos)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(
  ${moveit_task_constructor_core_INCLUDE_DIRS}
)
add_executable(cartesian src/cartesian.cpp)
ament_target_dependencies(cartesian
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_visual_tools
  tf2_eigen
)

install(TARGETS cartesian
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
