#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

#include <rom_add_collision/srv/gesture_command.hpp> 

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_gesture_service");

static const std::string PLANNING_GROUP = "ur5_manipulator";

class GestureMoveService : public rclcpp::Node
{
public:
  GestureMoveService() : Node("moveit_gesture_service", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    gesture_service_ = this->create_service<rom_add_collision::srv::GestureCommand>(
        "gesture_command_service",
        std::bind(&GestureMoveService::handle_gesture_command, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(LOGGER, "GestureMoveService initialized. Waiting for service calls on /gesture_command_service.");
  }

  void initializeMoveGroup()
  {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this()), PLANNING_GROUP);
    RCLCPP_INFO(LOGGER, "MoveGroupInterface initialized. Planning frame: %s", move_group_interface_->getPlanningFrame().c_str());
  }

private:
  // Service callback function
  void handle_gesture_command(
      const std::shared_ptr<rom_add_collision::srv::GestureCommand::Request> request,
      std::shared_ptr<rom_add_collision::srv::GestureCommand::Response> response)
  {
    RCLCPP_INFO(LOGGER, "Received gesture command request: '%s'", request->command.c_str());

    if (!is_running_)
    {
      is_running_ = true;
      RCLCPP_INFO(LOGGER, "Processing command...");

      bool command_executed = false;
      if (request->command == "1")
      {
        RCLCPP_INFO(LOGGER, "Executing 'move_to_defined_pose 1' command.");
        move_to_defined_pose(0.2, 0.2, 1.2, 0.0, 180.0, 0.0);
        command_executed = true;
      }
      else if (request->command == "2")
      {
        RCLCPP_INFO(LOGGER, "Executing 'move_to_defined_pose 2' command.");
        move_to_defined_pose(-0.2, 0.2, 1.2, 0.0, 180.0, 0.0);
        command_executed = true;
      }
      else if (request->command == "3")
      {
        RCLCPP_INFO(LOGGER, "Executing 'move_to_defined_pose 3' command.");
        move_to_defined_pose(-0.2, -0.2, 1.2, 0.0, 180.0, 0.0);
        command_executed = true;
      }
      else if (request->command == "4")
      {
        RCLCPP_INFO(LOGGER, "Executing 'move_to_defined_pose 4' command.");
        move_to_defined_pose(0.2, -0.2, 1.2, 0.0, 180.0, 0.0);
        command_executed = true;
      }
      else if (request->command == "5")
      {
        RCLCPP_INFO(LOGGER, "Executing 'Return Home' command.");
        move_to_defined_pose(0.078, 0.027, 1.724, 47.118, 8.183, -1.366);
        command_executed = true;
      }
      else
      {
        RCLCPP_INFO(LOGGER, "Unknown command received. Ignoring.");
        response->success = false;
        response->message = "Unknown command";
      }

      is_running_ = false;
      RCLCPP_INFO(LOGGER, "Command processing complete.");

      if (command_executed) 
      {
        response->success = true;
        response->message = "Command executed successfully";
      } 
      else if (response->message.empty()) 
      { 
        response->success = false;
        response->message = "Failed to execute command";
      }
    }
    else
    {
      RCLCPP_WARN(LOGGER, "Node is currently processing another command. Please wait.");
      response->success = false;
      response->message = "Node is busy";
    }
  }

  void move_to_defined_pose(double x, double y, double z, double roll_x, double pitch_y, double yaw_z)
  {
    geometry_msgs::msg::Pose target_pose;

    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;

    double roll_rad = roll_x * M_PI / 180.0;
    double pitch_rad = pitch_y * M_PI / 180.0;
    double yaw_rad = yaw_z * M_PI / 180.0;

    tf2::Quaternion q;
    q.setRPY(roll_rad, pitch_rad, yaw_rad);

    target_pose.orientation.w = q.w();
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();

    RCLCPP_INFO(LOGGER, "Setting target pose to:");
    RCLCPP_INFO(LOGGER, "  Position: (x=%.3f, y=%.3f, z=%.3f)",
                target_pose.position.x, target_pose.position.y, target_pose.position.z);
    RCLCPP_INFO(LOGGER, "  Orientation (RPY): (roll=%.3f deg, pitch=%.3f deg, yaw=%.3f deg)",
                roll_x, pitch_y, yaw_z);
    RCLCPP_INFO(LOGGER, "  Orientation (Quaternion): (w=%.3f, x=%.3f, y=%.3f, z=%.3f)",
                target_pose.orientation.w, target_pose.orientation.x,
                target_pose.orientation.y, target_pose.orientation.z);

    move_group_interface_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(LOGGER, "Planning successful! Executing plan...");

      moveit::core::MoveItErrorCode execute_result = move_group_interface_->execute(my_plan);

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
  }
  
  rclcpp::Service<rom_add_collision::srv::GestureCommand>::SharedPtr gesture_service_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  bool is_running_ = false;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GestureMoveService>(); 
  node->initializeMoveGroup();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}