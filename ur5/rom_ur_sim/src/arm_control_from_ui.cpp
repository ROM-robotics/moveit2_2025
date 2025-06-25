#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Helper function to plan and execute a motion
// In C++, plan() and execute() are typically called on the MoveGroupInterface object directly.
// We'll integrate this logic into the class methods.

class Controller : public rclcpp::Node, public std::enable_shared_from_this<Controller>
{
public:
  Controller()
  : Node("commander_from_ui"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)),
    panda_arm_(std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur5_manipulator")),
    panda_hand_(std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "robotiq_gripper"))
  {
    RCLCPP_INFO(this->get_logger(), "Initializing CommanderFromUI Node...");

    // Initialize MoveIt2 interfaces
    // The MoveGroupInterface constructor handles the necessary ROS2 setup
    // for interacting with MoveIt services.

    // Get basic information about the robot
    RCLCPP_INFO(this->get_logger(), "Robot Planning Frame: %s", panda_arm_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End Effector Link: %s", panda_arm_->getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(), "Available Planning Groups: ");
    for (const auto& group_name : panda_arm_->getJointModelGroupNames()) {
        RCLCPP_INFO(this->get_logger(), "  - %s", group_name.c_str());
    }

    // Set a planning time (in seconds)
    panda_arm_->setPlanningTime(10.0);
    panda_hand_->setPlanningTime(10.0);

    // Initialize the subscription
    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/target_point",
        10,
        std::bind(&Controller::listener_callback, this, std::placeholders::_1));

    // Define fixed heights and initial angles
    height_ = 0.18;
    pick_height_ = 0.126;
    carrying_height_ = 0.3;
    init_angle_ = -0.3825;

    RCLCPP_INFO(this->get_logger(), "CommanderFromUI Node initialized.");
  }

  ~Controller()
  {
    // Destructor for cleanup, if necessary
  }

private:
  // ROS2 Objects
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // MoveIt2 Objects
  moveit::planning_interface::MoveGroupInterface::SharedPtr panda_arm_;
  moveit::planning_interface::MoveGroupInterface::SharedPtr panda_hand_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Node parameters/constants
  double height_;
  double pick_height_;
  double carrying_height_;
  double init_angle_;

  // Helper to plan and execute
  bool plan_and_execute(moveit::planning_interface::MoveGroupInterface::SharedPtr move_group, double sleep_time = 0.0)
  {
    RCLCPP_INFO(this->get_logger(), "Planning trajectory for group: %s", move_group->getName().c_str());
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Executing plan for group: %s", move_group->getName().c_str());
      move_group->execute(my_plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed for group: %s", move_group->getName().c_str());
    }

    if (sleep_time > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
    }
    return success;
  }

  // Function to move the arm to a specified pose goal
  bool move_to(double x, double y, double z, double xo, double yo, double zo, double wo)
  {
    geometry_msgs::msg::PoseStamped pose_goal;
    pose_goal.header.frame_id = "base_link"; // Assuming base_link is the robot's base frame
    pose_goal.header.stamp = this->now();
    pose_goal.pose.position.x = x;
    pose_goal.pose.position.y = y;
    pose_goal.pose.position.z = z;
    pose_goal.pose.orientation.x = xo;
    pose_goal.pose.orientation.y = yo;
    pose_goal.pose.orientation.z = zo;
    pose_goal.pose.orientation.w = wo;

    RCLCPP_INFO(this->get_logger(), "Moving arm to pose: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    panda_arm_->setPoseTarget(pose_goal, "tool0"); // Target the 'tool0' link (end-effector)

    bool success = plan_and_execute(panda_arm_, 3.0);
    panda_arm_->clearPoseTargets(); // Clear targets after execution
    return success;
  }

  // Function for gripper action
  bool gripper_action(const std::string& action)
  {
    panda_hand_->setStartStateToCurrentState(); // Set current state as start state for planning

    std::map<std::string, double> joint_values;
    if (action == "open") {
      joint_values["robotiq_85_left_knuckle_joint"] = 0.0000;
      RCLCPP_INFO(this->get_logger(), "Opening gripper.");
    } else if (action == "close") {
      joint_values["robotiq_85_left_knuckle_joint"] = 0.65;
      RCLCPP_INFO(this->get_logger(), "Closing gripper.");
    } else {
      RCLCPP_WARN(this->get_logger(), "No such gripper action: %s", action.c_str());
      return false;
    }

    // Set joint target for the gripper
    panda_hand_->setJointValueTarget(joint_values);
    return plan_and_execute(panda_hand_, 3.0);
  }

  // Callback for the target_point subscription
  void listener_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 3) {
      RCLCPP_ERROR(this->get_logger(), "Received target_point message with insufficient data (expected at least 3 elements).");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Received target point: x=%.2f, y=%.2f, angle_offset=%.2f",
                msg->data[0], msg->data[1], msg->data[2]);

    // Pick and Place sequence
    // 1. Move to above pick location
    if (!move_to(msg->data[0], msg->data[1], height_, 1.0, init_angle_ + msg->data[2], 0.0, 0.0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to above pick location.");
        return;
    }

    // 2. Open gripper
    if (!gripper_action("open")) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open gripper.");
        return;
    }

    // 3. Move to pick location
    if (!move_to(msg->data[0], msg->data[1], pick_height_, 1.0, init_angle_ + msg->data[2], 0.0, 0.0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to pick location.");
        return;
    }

    // 4. Close gripper
    if (!gripper_action("close")) {
        RCLCPP_ERROR(this->get_logger(), "Failed to close gripper.");
        return;
    }

    // 5. Move to carrying height
    if (!move_to(msg->data[0], msg->data[1], carrying_height_, 1.0, init_angle_ + msg->data[2], 0.0, 0.0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to carrying height.");
        return;
    }

    // 6. Move to drop off location (fixed for this example)
    if (!move_to(0.3, -0.3, carrying_height_, 1.0, init_angle_ + msg->data[2], 0.0, 0.0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to move to drop-off location.");
        return;
    }

    // 7. Open gripper to release
    if (!gripper_action("open")) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open gripper at drop-off.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Pick and Place sequence completed for current target.");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // We use a MultiThreadedExecutor to ensure MoveIt2 background services
  // and the node's callbacks can run concurrently.
  rclcpp::executors::MultiThreadedExecutor executor;
  auto controller_node = std::make_shared<Controller>();
  executor.add_node(controller_node);

  // Spin the executor in a separate thread
  std::thread([&]() { executor.spin(); }).detach();

  // Keep the main thread alive to allow the executor thread to run
  // and to ensure rclcpp::ok() is checked.
  rclcpp::Rate rate(2); // 2 Hz
  while (rclcpp::ok()) {
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}