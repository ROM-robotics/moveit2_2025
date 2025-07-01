#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h> // Optional, but highly recommended for visualization
#include <thread> // For sleep
#include <moveit/core/error_code.h> // CORRECTED: Use this for MoveItErrorCode in Humble

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_pose_goal");

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("moveit_pose_goal", node_options);

  // We spin up a single-threaded Executor for the current node.
  // This is necessary for the MoveItSimpleControllerManager to receive feedback
  // from the controllers.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create a MoveGroupInterface
  // Replace "ur5_manipulator" with the actual name of your planning group
  // (e.g., "ur_arm" or "arm" from your SRDF).
  static const std::string PLANNING_GROUP = "ur5_manipulator"; 
  moveit::planning_interface::MoveGroupInterface move_group_interface(move_group_node, PLANNING_GROUP);

  // Get the name of the planning frame
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // Get a list of all joint names in the planning group
  RCLCPP_INFO(LOGGER, "Joint names for group '%s': ", PLANNING_GROUP.c_str());
  if (move_group_interface.getRobotModel() && move_group_interface.getRobotModel()->hasJointModelGroup(PLANNING_GROUP)) {
      const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getRobotModel()->getJointModelGroup(PLANNING_GROUP);
      if (joint_model_group) {
          for (const std::string& joint_name : joint_model_group->getVariableNames()) {
              RCLCPP_INFO(LOGGER, "- %s", joint_name.c_str());
          }
      } else {
          RCLCPP_ERROR(LOGGER, "Failed to get JointModelGroup for planning group '%s'", PLANNING_GROUP.c_str());
      }
  } else {
      RCLCPP_ERROR(LOGGER, "RobotModel or JointModelGroup '%s' not found for planning frame.", PLANNING_GROUP.c_str());
  }


  // (Optional) Initialize MoveItVisualTools for debugging and visualization
  // Only if you included moveit_visual_tools in your CMakeLists.txt and package.xml
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(
      move_group_node, "world", PLANNING_GROUP, move_group_interface.getRobotModel());
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  // Define a target pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.28;
  target_pose.position.y = -0.2;
  target_pose.position.z = 0.5;

  move_group_interface.setPoseTarget(target_pose);

  // Plan to the target pose
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(LOGGER, "Planning successful! Executing plan...");
    // Execute the planned path
    moveit::core::MoveItErrorCode execute_result = move_group_interface.execute(my_plan);
    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(LOGGER, "Execution successful!");
    }
    else
    {
      // CORRECTED: The message() method is part of the MoveItErrorCode object itself.
      RCLCPP_ERROR(LOGGER, "Execution failed: %s", execute_result.message().c_str());
    }
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning failed!");
  }

  // You can optionally add a short delay before shutting down
  std::this_thread::sleep_for(std::chrono::seconds(5));

  rclcpp::shutdown();
  return 0;
}