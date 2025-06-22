#include <cstdio>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("hello_moveit_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // create ros logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Spin up a SingleThreadedExecutor for moveitVisualTools to interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); }); 

  // create the moveit movegroup interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "panda_link0", rviz_visual_tools::RVIZ_MARKER_TOPIC,
  move_group_interface.getRobotModel()};
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create three closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0; // raise the text above the robot
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };

  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };

  auto const draw_trajectory_tool_path = [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup(
    "panda_arm")](auto const& trajectory){
      moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
    };

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
  prompt("Press 'Nexe' in the RvisVisualToolsGui window to ploan");
  draw_title("Planning");
  moveit_visual_tools.trigger();

  auto const [success, plan] = [&move_group_interface]() {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto const ok = static_cast<bool>(move_group_interface.plan(plan));
    return std::make_pair(ok, plan);
  }();

  // Execute the plan
  if(success) {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvisVisualToolsGui window to execute the plan");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } else {
    draw_title("Planning failed");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Failed to create a plan to the target pose");
  }

  rclcpp::shutdown();
  spinner.join();
  RCLCPP_INFO(logger, "MoveIt example completed successfully");
  return 0;
}
