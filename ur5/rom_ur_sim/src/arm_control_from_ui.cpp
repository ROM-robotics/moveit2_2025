#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread> // For std::this_thread::sleep_for

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"

using namespace std::chrono_literals;

/**
 * @brief Controller class for commanding a UR5 manipulator and Robotiq gripper
 * using MoveIt 2 C++ API (moveit_cpp).
 *
 * This class subscribes to a `/target_point` topic and performs a pick-and-place
 * sequence based on the received coordinates.
 * 
 * @author ROM DYNAMICS 
 * @date 2025-06-26
 */
class Controller : public rclcpp::Node
{
public:
    Controller()  : Node("commander_from_ui")
    {
        // IMPORTANT: Declare and get parameters for MoveItCpp.
        // MoveItCpp relies on these parameters to find the robot description
        // (e.g., URDF and SRDF) and the configured planning pipelines.
        // These parameters are typically loaded from a MoveIt configuration package.
        this->declare_parameter<std::string>("robot_description", "");
        this->declare_parameter<std::string>("planning_pipelines", "");
        
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/target_point", 10, std::bind(&Controller::listener_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /target_point topic.");

        // Instantiate MoveItCpp instance.
        // `shared_from_this()` provides a shared_ptr to the current Node instance,
        // which MoveItCpp needs to access ROS 2 services and parameters.
        moveit_cpp_ = std::make_shared<moveit_cpp::MoveItCpp>(shared_from_this());
        RCLCPP_INFO(this->get_logger(), "MoveItCpp instance created successfully.");
        

        ur5_arm_ = std::make_shared<moveit_cpp::PlanningComponent>("ur5_manipulator", moveit_cpp_);
        ur5_hand_ = std::make_shared<moveit_cpp::PlanningComponent>("robotiq_gripper", moveit_cpp_);
        RCLCPP_INFO(this->get_logger(), "Planning components for 'ur5_manipulator' and 'robotiq_gripper' created.");
        
        robot_model_ = moveit_cpp_->getRobotModel();
        if (!robot_model_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get robot model from MoveItCpp.");
            rclcpp::shutdown(); // Shutdown if we can't get the robot model
        }
        
        pose_goal_.header.frame_id = "base_link";

        // Define heights and initial angle specific to your robot's setup.
        height_ = 0.18;         // General height for movement
        pick_height_ = 0.126;   // Height for picking up an object
        carrying_height_ = 0.3; // Height for carrying an object
        init_angle_ = -0.3825;  // Initial orientation angle offset
    }

private:
    /**
     * @brief Moves the end-effector to a specified Cartesian pose.
     * @param x X-coordinate of the target position.
     * @param y Y-coordinate of the target position.
     * @param z Z-coordinate of the target position.
     * @param xo X-component of the quaternion orientation.
     * @param yo Y-component of the quaternion orientation.
     * @param zo Z-component of the quaternion orientation.
     * @param wo W-component of the quaternion orientation.
     */
    void move_to(double x, double y, double z, double xo, double yo, double zo, double wo)
    {
        pose_goal_.pose.position.x = x;
        pose_goal_.pose.position.y = y;
        pose_goal_.pose.position.z = z;
        pose_goal_.pose.orientation.x = xo;
        pose_goal_.pose.orientation.y = yo;
        pose_goal_.pose.orientation.z = zo;
        pose_goal_.pose.orientation.w = wo;

        // Set the goal state for the arm planning component using the pose.
        // "tool0" is assumed to be the name of the end-effector link.
        ur5_arm_->setGoal(pose_goal_, "tool0");
        RCLCPP_INFO(this->get_logger(), "Planning arm movement to x: %.2f, y: %.2f, z: %.2f", x, y, z);

        // Plan the trajectory. `plan()` returns a PlanSolution object.
        auto plan_solution = ur5_arm_->plan();
        
        if (plan_solution)
        {
            RCLCPP_INFO(this->get_logger(), "Executing arm plan.");
            ur5_arm_->execute();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Arm planning failed.");
        }
        // Pause for a moment to allow execution to complete and for visualization to update.
        std::this_thread::sleep_for(3s);
    }

    void gripper_action(const std::string& action)
    {
        ur5_hand_->setStartStateToCurrentState();

        // Create a Constraints message to specify the joint goal.
        moveit_msgs::msg::Constraints goal_constraints;
        moveit_msgs::msg::JointConstraint joint_constraint;
        // The name of the joint to control for the gripper (e.g., "robotiq_85_left_knuckle_joint").
        joint_constraint.joint_name = "robotiq_85_left_knuckle_joint";
        joint_constraint.weight = 1.0; // Weight of this constraint (1.0 means it's fully considered).

        if (action == "open")
        {
            joint_constraint.position = 0.0000; // Joint position for opening the gripper.
            joint_constraint.tolerance_above = 0.01; // Tolerance above the target position.
            joint_constraint.tolerance_below = 0.01; // Tolerance below the target position.
        }
        else if (action == "close")
        {
            joint_constraint.position = 0.65; // Joint position for closing the gripper.
            joint_constraint.tolerance_above = 0.01;
            joint_constraint.tolerance_below = 0.01;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No such gripper action: %s. Skipping.", action.c_str());
            return;
        }

        // Add the joint constraint to the overall goal constraints.
        goal_constraints.joint_constraints.push_back(joint_constraint);

        // Set the goal for the hand planning component using the joint constraints.
        //ur5_hand_->setGoal(goal_constraints);
        ur5_hand_->setGoal(std::vector<moveit_msgs::msg::Constraints>{goal_constraints});
        RCLCPP_INFO(this->get_logger(), "Planning gripper action: %s", action.c_str());

        auto plan_solution = ur5_hand_->plan();

        if (plan_solution)
        {
            RCLCPP_INFO(this->get_logger(), "Executing gripper plan.");
            ur5_hand_->execute();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Gripper planning failed.");
        }
        std::this_thread::sleep_for(3s);
    }
    
    void listener_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received target point: [%f, %f, %f]", msg->data[0], msg->data[1], msg->data[2]);

        // Sequence of movements for pick and place:
        // 1. Move to a safe height above the target.
        move_to(msg->data[0], msg->data[1], height_, 1.0, init_angle_ + msg->data[2], 0.0, 0.0);

        // 2. Open the gripper.
        gripper_action("open");

        // 3. Move down to the pick height to grasp the object.
        move_to(msg->data[0], msg->data[1], pick_height_, 1.0, init_angle_ + msg->data[2], 0.0, 0.0);

        // 4. Close the gripper to grasp the object.
        gripper_action("close");

        // 5. Move up to a carrying height with the object.
        move_to(msg->data[0], msg->data[1], carrying_height_, 1.0, init_angle_ + msg->data[2], 0.0, 0.0);

        // 6. Move to a predefined drop-off location.
        move_to(0.3, -0.3, carrying_height_, 1.0, init_angle_ + msg->data[2], 0.0, 0.0);

        // 7. Open the gripper to release the object.
        gripper_action("open");

        RCLCPP_INFO(this->get_logger(), "Pick and place sequence completed for current target.");
    }

    // Private member variables
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_;
    std::shared_ptr<moveit_cpp::PlanningComponent> ur5_arm_;
    std::shared_ptr<moveit_cpp::PlanningComponent> ur5_hand_;
    moveit::core::RobotModelConstPtr robot_model_; 

    geometry_msgs::msg::PoseStamped pose_goal_;

    double height_;
    double pick_height_;
    double carrying_height_;
    double init_angle_;
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto controller_node = std::make_shared<Controller>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controller_node);
    
    std::thread executor_thread([&executor]() { executor.spin(); });
    
    rclcpp::Rate loop_rate(2); // Loop at 2 Hz
    while (rclcpp::ok()) {
        loop_rate.sleep();
    }
    
    rclcpp::shutdown();
    executor_thread.join();

    return 0;
}