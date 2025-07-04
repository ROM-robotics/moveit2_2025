#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp> // Include for geometry_msgs::msg::Pose
#include <tf2/LinearMath/Quaternion.h> // For converting RPY to Quaternion
#include <tf2/LinearMath/Matrix3x3.h> // For converting Quaternion to RPY for logging (optional)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // For converting between tf2 and geometry_msgs
#include <cmath> // Required for M_PI

// Define a logger for the node
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_pose_goal_args");

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "moveit_pose_goal_args", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();
  
  if (argc != 7) {
    RCLCPP_ERROR(LOGGER, "Usage: %s <x> <y> <z> <roll_deg> <pitch_deg> <yaw_deg>", argv[0]);
    RCLCPP_ERROR(LOGGER, "Example: %s 0.2 -0.2 1.2 0.0 0.0 0.0 (for identity orientation)", argv[0]);
    RCLCPP_ERROR(LOGGER, "Example: %s 0.2 -0.2 1.2 0.0 180.0 90.0 ", argv[0]);
    rclcpp::shutdown();
    return 1; // Indicate an error, 
  }

  geometry_msgs::msg::Pose target_pose;
  double roll_deg, pitch_deg, yaw_deg; // Euler angles in degrees
  double roll_rad, pitch_rad, yaw_rad; // Euler angles in radians for tf2 conversion

  try {
    target_pose.position.x = std::stod(argv[1]);
    target_pose.position.y = std::stod(argv[2]);
    target_pose.position.z = std::stod(argv[3]);
    roll_deg = std::stod(argv[4]);
    pitch_deg = std::stod(argv[5]);
    yaw_deg = std::stod(argv[6]);
  } catch (const std::invalid_argument& e) {
    RCLCPP_ERROR(LOGGER, "Invalid argument: %s. Please ensure all arguments are valid numbers.", e.what());
    rclcpp::shutdown();
    return 1;
  } catch (const std::out_of_range& e) {
    RCLCPP_ERROR(LOGGER, "Argument out of range: %s. Please ensure numbers are within valid double range.", e.what());
    rclcpp::shutdown();
    return 1;
  }
  
  roll_rad = roll_deg * M_PI / 180.0;
  pitch_rad = pitch_deg * M_PI / 180.0;
  yaw_rad = yaw_deg * M_PI / 180.0;
  
  tf2::Quaternion q;
  q.setRPY(roll_rad, pitch_rad, yaw_rad);
  target_pose.orientation.w = q.w();
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();

  RCLCPP_INFO(LOGGER, "Target Pose set to:");
  RCLCPP_INFO(LOGGER, "  Position: (x=%.3f, y=%.3f, z=%.3f)",
              target_pose.position.x, target_pose.position.y, target_pose.position.z);
  RCLCPP_INFO(LOGGER, "  Orientation (RPY): (roll=%.3f deg, pitch=%.3f deg, yaw=%.3f deg)",
              roll_deg, pitch_deg, yaw_deg);
  RCLCPP_INFO(LOGGER, "  Orientation (Quaternion): (w=%.3f, x=%.3f, y=%.3f, z=%.3f)",
              target_pose.orientation.w, target_pose.orientation.x,
              target_pose.orientation.y, target_pose.orientation.z);
              
  static const std::string PLANNING_GROUP = "ur5_manipulator";
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);
  
  RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  move_group_interface.setPoseTarget(target_pose);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(LOGGER, "Planning successful! Executing plan...");
    moveit::core::MoveItErrorCode execute_result = move_group_interface.execute(my_plan);
    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(LOGGER, "Execution successful!");
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Execution failed");
    }
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning failed!");
  }
  
  rclcpp::shutdown();
  return 0;
}
