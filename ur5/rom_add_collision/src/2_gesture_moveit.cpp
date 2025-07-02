#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <std_msgs/msg/string.hpp> 
#include <cmath> 

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_gesture_subscriber");

static const std::string PLANNING_GROUP = "ur5_manipulator"; 

class GestureMoveNode : public rclcpp::Node
{
public:
  GestureMoveNode() : Node("moveit_gesture_subscriber", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    gesture_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "/gesture_command",
        1, // QoS history depth
        std::bind(&GestureMoveNode::gesture_command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(LOGGER, "GestureMoveNode initialized. Waiting for commands on /gesture_command topic.");
  }

  void initializeMoveGroup()
  {
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this()), PLANNING_GROUP);
    RCLCPP_INFO(LOGGER, "MoveGroupInterface initialized. Planning frame: %s", move_group_interface_->getPlanningFrame().c_str());
  }

private:
  void gesture_command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(LOGGER, "Received gesture command: '%s'", msg->data.c_str());

    if(!is_running_)
    {
        RCLCPP_INFO(LOGGER, "Processing command...");
        if (msg->data == "1")
        {
            is_running_ = true;
            RCLCPP_INFO(LOGGER, "Executing 'move_to_defined_pose 1' command.");
            move_to_defined_pose(0.2, 0.2, 1.2, 0.0, 180.0, 0.0); 
            is_running_ = false;
        }
        else if (msg->data == "2")
        {
            is_running_ = true;
            RCLCPP_INFO(LOGGER, "Executing 'move_to_defined_pose 2' command.");
            move_to_defined_pose(-0.2, 0.2, 1.2, 0.0, 180.0, 0.0);
            is_running_ = false;
        }
        else if (msg->data == "3")
        {
            is_running_ = true;
            RCLCPP_INFO(LOGGER, "Executing 'move_to_defined_pose 3' command.");
            move_to_defined_pose(-0.2, -0.2, 1.2, 0.0, 180.0, 0.0);
            is_running_ = false;
        }
        else if (msg->data == "4")
        {
            is_running_ = true;
            RCLCPP_INFO(LOGGER, "Executing 'move_to_defined_pose 4' command.");
            move_to_defined_pose(0.2, -0.2, 1.2, 0.0, 180.0, 0.0);
            is_running_ = false;
        }
        else
        {
            RCLCPP_INFO(LOGGER, "Unknown command received. Ignoring.");
        }
        RCLCPP_INFO(LOGGER, "Command processing complete.");
    }
    else
    {
        RCLCPP_WARN(LOGGER, "Node is currently processing another command. Please wait.");
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
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gesture_subscriber_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  bool is_running_ = false;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<GestureMoveNode>();
  node->initializeMoveGroup();
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
