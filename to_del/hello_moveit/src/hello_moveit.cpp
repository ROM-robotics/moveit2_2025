#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("hello_moveit_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // create ros logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // next step goes here
  // create the moveit movegroup interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // set a target pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.28;
    pose.position.y = -0.2;
    pose.position.z = 0.5;
    pose.orientation.w = 1.0; // no rotation
    return pose;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]() {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const ok = static_cast<bool>(move_group_interface.plan(plan));
    return std::make_pair(ok, plan);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Failed to create a plan to the target pose");
  }

  rclcpp::shutdown();
  return 0;
}
