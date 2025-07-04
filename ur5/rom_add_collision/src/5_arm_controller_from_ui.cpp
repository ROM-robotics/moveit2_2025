#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// Include TF2 for quaternion operations
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h" // Not strictly needed for this, but useful
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 

// MoveIt 2 includes
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_state/conversions.h"
#include "moveit_msgs/msg/display_robot_state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("arm_controller_from_ui")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/target_point", 10, std::bind(&Controller::listener_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Commander node has been initialized.");

        pick_height_ = 0.126;   // down from height_
        carrying_height_ = 0.3; // up after picking, and place height

        
        height_ = 0.23;         // tool0 link is 23 centimeters from base_link ( 18 cm is collide to bolt )
        pick_height_ = 0.17;    // to pick lower height to tool0 link
        init_angle_ = 1.3201;      // for adjust z-axis of tool0 
    }

    void initialize_move_groups()
    {
        arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur5_manipulator");
        hand_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "robotiq_gripper");

        arm_move_group_->setPoseReferenceFrame("base_link");
        arm_move_group_->setEndEffectorLink("tool0");

        RCLCPP_INFO(this->get_logger(), "MoveGroupInterfaces initialized.");
    }


private:
    void plan_and_execute(std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group, double sleep_time = 0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Planning trajectory");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        //bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        bool success = (move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Executing plan");
            move_group->execute(my_plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
        }

        if (sleep_time > 0.0)
        {
            rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(sleep_time * 1000)));
        }
    }
    
    void move_to(double x, double y, double z, double xo, double yo, double zo, double wo)
    {
        geometry_msgs::msg::PoseStamped pose_goal;
        pose_goal.header.frame_id = "base_link";
        pose_goal.pose.position.x = x;
        pose_goal.pose.position.y = y;
        pose_goal.pose.position.z = z;
        pose_goal.pose.orientation.x = xo;
        pose_goal.pose.orientation.y = yo;
        pose_goal.pose.orientation.z = zo;
        pose_goal.pose.orientation.w = wo;

        arm_move_group_->setPoseTarget(pose_goal);
        plan_and_execute(arm_move_group_, 3.0);
    }

    // Function for a gripper action
    void gripper_action(const std::string& action)
    {
        // moveit::core::RobotState current_state = hand_move_group_->getCurrentState();
        // std::vector<double> joint_group_positions;
        // current_state.copyJointGroupPositions(hand_move_group_->getName(), joint_group_positions);

        // if (action == "open")
        // {
        //     joint_group_positions[0] = 0.03; // Assuming panda_finger_joint1 is the first joint
        // }
        // else if (action == "close")
        // {
        //     joint_group_positions[0] = 0.001; // Assuming panda_finger_joint1 is the first joint
        // }
        // else
        // {
        //     RCLCPP_INFO(this->get_logger(), "No such action: %s", action.c_str());
        //     return;
        // }

        // hand_move_group_->setJointValueTarget(joint_group_positions);
        // plan_and_execute(hand_move_group_, 3.0);
    }
    // Function for a gripper action
    // void gripper_action(const std::string& action)
    // {
    //     if (!hand_move_group_) {
    //         RCLCPP_ERROR(this->get_logger(), "Hand MoveGroupInterface is not initialized! Cannot perform gripper action.");
    //         return;
    //     }
        
    //     moveit::core::RobotStatePtr current_state = hand_move_group_->getCurrentState();
    //     if (!current_state) {
    //         RCLCPP_ERROR(this->get_logger(), "Could not get current robot state for gripper.");
    //         return;
    //     }
        
    //     std::vector<double> joint_group_positions;
    //     current_state->copyJointGroupPositions(hand_move_group_->getName(), joint_group_positions);

    //     if (joint_group_positions.empty()) {
    //         RCLCPP_ERROR(this->get_logger(), "No joint positions found for gripper group. Check SRDF/URDF.");
    //         return;
    //     }

    //     // IMPORTANT: This block was commented out in your provided code, causing the warning.
    //     // Uncommenting it will resolve the 'unused parameter' warning.
    //     if (action == "open")
    //     {
    //         joint_group_positions[0] = 0.03; // Example open value
    //     }
    //     else if (action == "close")
    //     {
    //         joint_group_positions[0] = 0.001; // Example close value (almost closed)
    //     }
    //     else
    //     {
    //         RCLCPP_INFO(this->get_logger(), "No such action: %s", action.c_str());
    //         return;
    //     }

    //     hand_move_group_->setJointValueTarget(joint_group_positions);
    //     plan_and_execute(hand_move_group_, 3.0);
    // }

    void listener_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received data: [%f, %f, %f]", msg->data[0], msg->data[1], msg->data[2]);

        // Move to initial height
        //move_to(msg->data[0], msg->data[1], height_, 1.0, init_angle_ + msg->data[2], 0.0, 0.0); 
        double final_yaw_angle = init_angle_ + msg->data[2]; // msg->data[2] is the angle from Python (in radians)

        tf2::Quaternion q_yaw_from_bolt;
        q_yaw_from_bolt.setRPY(0, 3.14, 1.57); // y=180, z=90 , it's ok
        q_yaw_from_bolt.setRPY(0, M_PI, final_yaw_angle);

        // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
        geometry_msgs::msg::Quaternion final_ros_orientation = tf2::toMsg(q_yaw_from_bolt);
        
        move_to(msg->data[0], msg->data[1], height_, final_ros_orientation.x, final_ros_orientation.y, final_ros_orientation.z, final_ros_orientation.w);
        
        // gripper_action("open");
        
        // move_to(msg->data[0], msg->data[1], pick_height_, 1.0, init_angle_ + msg->data[2], 0.0, 0.0);
        
        // gripper_action("close");
        
        // move_to(msg->data[0], msg->data[1], carrying_height_, 1.0, init_angle_ + msg->data[2], 0.0, 0.0);
        
        // move_to(0.3, -0.3, carrying_height_, 1.0, init_angle_ + msg->data[2], 0.0, 0.0);
        
        // gripper_action("open");
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_move_group_;

    double height_;
    double pick_height_;
    double carrying_height_;
    double init_angle_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto controller_node = std::make_shared<Controller>();
    controller_node->initialize_move_groups();

    RCLCPP_INFO(controller_node->get_logger(), "Controller node is ready to receive commands.");
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controller_node);

    RCLCPP_INFO(controller_node->get_logger(), "Executor is set up with the controller node.");
    
    std::thread([&executor]() { executor.spin(); }).detach();

    RCLCPP_INFO(controller_node->get_logger(), "Controller node is spinning.");

    while (rclcpp::ok()) 
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }
    
    rclcpp::shutdown();

    return 0;
}