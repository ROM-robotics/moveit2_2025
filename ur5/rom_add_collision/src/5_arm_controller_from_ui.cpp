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


// MoveIt 2 Core includes (added explicitly for robustness)
// These define foundational classes like RobotModel and RobotState, which trajectory processing depends on.
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"

// MoveIt 2 Planning Interface includes
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_state/conversions.h" // Already there, but good to keep grouped

// MoveIt 2 Messages includes
#include "moveit_msgs/msg/display_robot_state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"

// MoveIt 2 Trajectory processing includes
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_parameterization.h>

#define UNUSED(x) (void)(x)

static const std::string GRIPPER_JOINT_NAME = "robotiq_85_left_knuckle_joint";
static const double OPEN_GRIPPER_VALUE = 0.0;  
static const double CLOSE_GRIPPER_VALUE = 0.55; // open = 0.00008, close = 0.791

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("arm_controller_from_ui")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/target_point", 10, std::bind(&Controller::listener_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Commander node has been initialized.");

        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    }

    void initialize_move_groups()
    {
        arm_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "ur5_manipulator");
        hand_move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "robotiq_gripper");

        arm_move_group_->setPoseReferenceFrame("base_link");
        arm_move_group_->setEndEffectorLink("tool0");

        RCLCPP_INFO(this->get_logger(), "MoveGroupInterfaces initialized.");

        // --- ADDED: Move to 'HOME' position ---
        RCLCPP_INFO(this->get_logger(), "Attempting to move arm to 'HOME' position...");
        arm_move_group_->setNamedTarget("home");
        if (plan_and_execute(arm_move_group_, 3.0)) 
        {
            RCLCPP_INFO(this->get_logger(), "Successfully moved arm to 'HOME' position.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to move arm to 'HOME' position. Check 'HOME' state definition and initial robot state.");
        }
        // --- END ADDED ---
    }


private:
    bool plan_and_execute(std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group, double sleep_time = 0.0)
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
        return success;
    }
    
    bool move_to(const geometry_msgs::msg::PoseStamped& pose_goal)
    {
        arm_move_group_->setPoseTarget(pose_goal);
        bool success = plan_and_execute(arm_move_group_, 3.0);
        return success;
    }

    bool move_to_vector(geometry_msgs::msg::PoseStamped pose_goal)
    {
        std::vector<geometry_msgs::msg::Pose> approach_waypoints1;
        pose_goal.pose.position.z -= 0.035;
        approach_waypoints1.push_back(pose_goal.pose);

        pose_goal.pose.position.z -= 0.035;
        approach_waypoints1.push_back(pose_goal.pose);

        moveit_msgs::msg::RobotTrajectory trajectory_approach1;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;

        double fraction = arm_move_group_->computeCartesianPath(approach_waypoints1, eef_step, jump_threshold, trajectory_approach1);

        arm_move_group_->execute(trajectory_approach1);

        rclcpp::sleep_for(std::chrono::seconds(3));


        return true;
    }

    // Function for a gripper action
    void gripper_action(const std::string& action)
    {
        if (action == "open")
        {
            hand_move_group_->setNamedTarget("open");
        }
        else if (action == "close")
        {
            hand_move_group_->setNamedTarget("close");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No such action: %s", action.c_str());
            return;
        }
        plan_and_execute(hand_move_group_, 3.0);
    }

    bool gripper_close()
    {
        // // GRIPPER ACQUISITION
        //     moveit::core::RobotStatePtr current_gripper_state = hand_move_group_->getCurrentState();
            
        //     const moveit::core::JointModelGroup* joint_model_group_gripper = current_gripper_state->getJointModelGroup(hand_move_group_->getName()); // Or use a static const PLANNING_GROUP_GRIPPER

        //     if (!joint_model_group_gripper) {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to get joint model group for gripper.");
        //         return false;
        //     }
            
        //     std::vector<double> current_gripper_joint_positions;
        //     current_gripper_state->copyJointGroupPositions(joint_model_group_gripper, current_gripper_joint_positions);
            
        //     const std::vector<std::string>& joint_names = joint_model_group_gripper->getVariableNames();
        //     auto it = std::find(joint_names.begin(), joint_names.end(), GRIPPER_JOINT_NAME);

        //     if (it != joint_names.end()) {
        //         int index = std::distance(joint_names.begin(), it);
        //         RCLCPP_INFO(this->get_logger(), "Gripper joint '%s' position BEFORE closing: %f",
        //                     GRIPPER_JOINT_NAME.c_str(), current_gripper_joint_positions[index]);
        //     } else {
        //         RCLCPP_ERROR(this->get_logger(), "Gripper joint '%s' not found in the gripper group.", GRIPPER_JOINT_NAME.c_str());
        //         // You might still proceed, but it's good to know if the joint isn't found.
        //     } 
        // // END GRIPPER ACQUISITION

        std::map<std::string, double> gripper_joint_targets_close;
        gripper_joint_targets_close[GRIPPER_JOINT_NAME] = CLOSE_GRIPPER_VALUE;

        hand_move_group_->setJointValueTarget(gripper_joint_targets_close);
        moveit::planning_interface::MoveGroupInterface::Plan gripper_plan_close;
        bool gripper_close_success = (hand_move_group_->plan(gripper_plan_close) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (gripper_close_success) {
            hand_move_group_->execute(gripper_plan_close);
            RCLCPP_INFO(this->get_logger(), "Gripper closed, object grasped.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan or execute gripper close action.");
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
        return gripper_close_success;
    }


    void listener_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received data: [%f, %f, %f]", msg->data[0], msg->data[1], msg->data[2]);

        double yaw_angle_of_target_bolt = msg->data[2] + 0.0; // Adjust this value if needed for collision avoidance
        double target_bolt_fit_to_tool0 = init_angle_ + msg->data[2];

        tf2::Quaternion q_yaw_from_bolt;
        q_yaw_from_bolt.setRPY(0, M_PI, target_bolt_fit_to_tool0);

        // Define bolt properties (adjust these based on your actual bolt and setup)
        const std::string world_frame = "base_link"; 
        const std::string bolt_id = "target_bolt";
        const double bolt_radius = 0.02; // 4cm diameter of clylinder
        const double bolt_height = 0.07; // 7cm length 
        const double bolt_center_z = bolt_radius;

        geometry_msgs::msg::PoseStamped pose_goal;
        pose_goal.header.frame_id = world_frame;
        pose_goal.pose.position.x = msg->data[0];
        pose_goal.pose.position.y = msg->data[1];
        pose_goal.pose.position.z = height_ + robotiq_2f_85_gripper_height; // ( 0.0700 + 0.1628 = 23 cm height tool0 from base_link )
        pose_goal.pose.orientation.x = q_yaw_from_bolt.x();
        pose_goal.pose.orientation.y = q_yaw_from_bolt.y();
        pose_goal.pose.orientation.z = q_yaw_from_bolt.z();
        pose_goal.pose.orientation.w = q_yaw_from_bolt.w();

        // Define your gripper's link name and touch links
        const std::string gripper_palm_link = "robotiq_85_base_link"; 
        const std::vector<std::string> gripper_finger_links = {"robotiq_85_left_knuckle_link", "robotiq_85_left_finger_link", "robotiq_85_right_knuckle_link", "robotiq_85_right_finger_link" };

        // --- 0. Add the bolt as a collision object before any movement towards it ---
        add_bolt_to_planning_scene(bolt_id, pose_goal.pose.position.x, pose_goal.pose.position.y, bolt_center_z, bolt_radius, bolt_height, world_frame, 0, M_PI, yaw_angle_of_target_bolt);


        // --- 1. Move to initial height (pre-grasp) ---
        RCLCPP_INFO(this->get_logger(), "Attempting to move to initial height...");
        if (  !move_to(pose_goal)  )
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to move to initial height (pre-grasp). Aborting pick operation.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Reached initial height.");


        // --- 2. Open Gripper ---
        RCLCPP_INFO(this->get_logger(), "Attempting to open gripper...");
        gripper_action("open");
        RCLCPP_INFO(this->get_logger(), "Gripper opened.");



        // // --- 3. Move down to pick height ---
        RCLCPP_INFO(this->get_logger(), "Attempting to move to pick height...");
        if (  !move_to_vector(pose_goal)  )
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to move to pick height. Check collision objects and pick_height_!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Reached pick height.");


        
        // --- 4. Close Gripper (Grasp) and Attach Bolt ---
        RCLCPP_INFO(this->get_logger(), "Attempting to close gripper and attach bolt...");
        bool gripper_close_status = gripper_close();
        if (!gripper_close_status)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to close gripper. Aborting pick operation.");
            return;
        }
        // After the gripper is closed, attach the bolt to the gripper
        attach_bolt_to_gripper(bolt_id, gripper_palm_link, gripper_finger_links);
        RCLCPP_INFO(this->get_logger(), "Gripper closed and bolt attached.");

        /*
        
        // --- 5. Move up to carrying height ---
        RCLCPP_INFO(this->get_logger(), "Attempting to move to carrying height...");
        if (  !move_to(msg->data[0], msg->data[1], carrying_height_, final_ros_orientation.x, final_ros_orientation.y, final_ros_orientation.z, final_ros_orientation.w)  )
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to move to carrying height with attached bolt.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Reached carrying height.");



        // --- 6. Move to a safe 'place' position (e.g., above drop-off) ---
        RCLCPP_INFO(this->get_logger(), "Attempting to move to drop-off pre-position...");
        if (!move_to(0.3, -0.3, carrying_height_, final_ros_orientation.x, final_ros_orientation.y,final_ros_orientation.z, final_ros_orientation.w))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to move to drop-off pre-position.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Reached drop-off pre-position.");



        // --- 7. Open Gripper to release and Detach Bolt ---
        RCLCPP_INFO(this->get_logger(), "Attempting to open gripper for release and detach bolt...");
        gripper_action("open");
        // After opening the gripper, detach the bolt. It will be re-added to the world scene.
        detach_bolt_from_gripper(bolt_id, gripper_palm_link);
        RCLCPP_INFO(this->get_logger(), "Gripper opened and bolt detached.");

        // (Optional) Move away from the released object if necessary
        // Example: move up slightly after release
        // if (!move_to(0.3, -0.3, carrying_height_ + 0.05, final_ros_orientation.x, final_ros_orientation.y, final_ros_orientation.z, final_ros_orientation.w)) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to move away after release.");
        //     return;
        // }
        */
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> hand_move_group_;

    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

    void add_bolt_to_planning_scene(const std::string& bolt_id, double x, double y, double z, double radius, double height, const std::string& frame_id, double roll, double pitch, double yaw);
    void attach_bolt_to_gripper(const std::string& bolt_id, const std::string& gripper_link_name, const std::vector<std::string>& touch_links);
    void detach_bolt_from_gripper(const std::string& bolt_id, const std::string& gripper_link_name);

    double table_height_ = 1.02;
    double robotiq_2f_85_gripper_height = 0.1628;       // ROBOTIQ GRIPPER DIMENSION 162.8 x 148.6 x 61 mm, 0.1628 x 0.1486 x 0.061 m
    double height_ = 0.0700;                            // gripper finger top  is 7 centimeters from base_link 

    double pick_height_ = 0.165;    // 17 cm ( tool0 link from base_link ), for picking bolt
    double carrying_height_ = 0.3; // up after picking, and place height
    double init_angle_ = 1.3201;   // for adjust z-axis of tool0 
};

void Controller::add_bolt_to_planning_scene(const std::string& bolt_id, double x, double y, double z, double radius, double height, const std::string& frame_id, double roll, double pitch, double yaw)
{
    if (!planning_scene_interface_) {
        RCLCPP_ERROR(this->get_logger(), "PlanningSceneInterface not initialized. Cannot add bolt.");
        return;
    }

    moveit_msgs::msg::CollisionObject bolt_object;
    bolt_object.header.frame_id = frame_id;
    bolt_object.id = bolt_id;
    
    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.dimensions.resize(2);
    cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = height;
    cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = radius;
    
    geometry_msgs::msg::Pose bolt_pose;
    bolt_pose.position.x = x;
    bolt_pose.position.y = y;
    bolt_pose.position.z = z;

    tf2::Quaternion tmp_from_bolt;
        tmp_from_bolt.setRPY(yaw, 1.5, 0);
        UNUSED(roll);
        UNUSED(pitch);
        geometry_msgs::msg::Quaternion final_ros_orientation = tf2::toMsg(tmp_from_bolt);

    bolt_pose.orientation = final_ros_orientation;
    
    bolt_object.primitives.push_back(cylinder);
    bolt_object.primitive_poses.push_back(bolt_pose);
    
    bolt_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(bolt_object);

    planning_scene_interface_->addCollisionObjects(collision_objects);
    RCLCPP_INFO(this->get_logger(), "Added bolt '%s' to the planning scene at [%.2f, %.2f, %.2f].", bolt_id.c_str(), x, y, z);
}

void Controller::attach_bolt_to_gripper(const std::string& bolt_id, const std::string& gripper_link_name, const std::vector<std::string>& touch_links)
{
    if (!planning_scene_interface_) {
        RCLCPP_ERROR(this->get_logger(), "PlanningSceneInterface not initialized. Cannot attach bolt.");
        return;
    }

    // First, remove the object from the world collision scene
    // It must not be in the world scene when being attached
    moveit_msgs::msg::CollisionObject remove_from_world;
    remove_from_world.id = bolt_id;
    remove_from_world.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface_->applyCollisionObject(remove_from_world);
    RCLCPP_INFO(this->get_logger(), "Removed bolt '%s' from world collision scene before attaching.", bolt_id.c_str());


    // Now, create the AttachedCollisionObject message
    moveit_msgs::msg::AttachedCollisionObject attached_bolt;
    attached_bolt.link_name = gripper_link_name;
    attached_bolt.object.id = bolt_id;
    attached_bolt.object.operation = moveit_msgs::msg::CollisionObject::ADD; // Add as an attached object

    // Specify which links of the robot are allowed to touch the attached object
    // (e.g., your gripper fingers)
    attached_bolt.touch_links = touch_links;

    planning_scene_interface_->applyAttachedCollisionObject(attached_bolt);
    RCLCPP_INFO(this->get_logger(), "Attached bolt '%s' to gripper link '%s'.", bolt_id.c_str(), gripper_link_name.c_str());
}


void Controller::detach_bolt_from_gripper(const std::string& bolt_id, const std::string& gripper_link_name)
{
    if (!planning_scene_interface_) {
        RCLCPP_ERROR(this->get_logger(), "PlanningSceneInterface not initialized. Cannot detach bolt.");
        return;
    }

    moveit_msgs::msg::AttachedCollisionObject detached_bolt;
    detached_bolt.link_name = gripper_link_name;
    detached_bolt.object.id = bolt_id;
    detached_bolt.object.operation = moveit_msgs::msg::CollisionObject::REMOVE; // Remove as an attached object

    planning_scene_interface_->applyAttachedCollisionObject(detached_bolt);
    RCLCPP_INFO(this->get_logger(), "Detached bolt '%s' from gripper link '%s'. It is now back in the world scene.", bolt_id.c_str(), gripper_link_name.c_str());
}

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