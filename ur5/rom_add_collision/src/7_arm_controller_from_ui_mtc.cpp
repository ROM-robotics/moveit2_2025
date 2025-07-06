#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// TF2 for quaternion operations
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// MoveIt 2 Core includes
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_state/conversions.h"

// MoveIt Task Constructor includes
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/cost_terms.h>

#define UNUSED(x) (void)(x)

namespace mtc = moveit::task_constructor;

// It's good practice to derive from enable_shared_from_this if you use shared_from_this()
class Controller : public rclcpp::Node, public std::enable_shared_from_this<Controller>
{
public:
    Controller() : Node("arm_controller_mtc")
    {
        // Declare and get parameters for robot groups and end-effector links
        this->declare_parameter<std::string>("arm_group_name", "ur5_manipulator");
        this->declare_parameter<std::string>("hand_group_name", "robotiq_gripper");
        this->declare_parameter<std::string>("eef_link", "tool0");
        this->declare_parameter<std::string>("world_frame", "world");

        arm_group_name_ = this->get_parameter("arm_group_name").as_string();
        hand_group_name_ = this->get_parameter("hand_group_name").as_string();
        eef_link_ = this->get_parameter("eef_link").as_string();
        world_frame_ = this->get_parameter("world_frame").as_string();

        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/target_point", 10, std::bind(&Controller::listener_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Controller node has been initialized. Waiting for /target_point messages.");

        // Define heights and initial angle
        carrying_height_ = 0.3; // up after picking, and place height
        height_ = 0.23;         // pre-grasp height
        pick_height_ = 0.165;   // grasp height
        init_angle_ = M_PI;     // assuming tool0 points down with M_PI pitch (0 roll, 0 yaw initially relative to base_link)
    }

private:
    void listener_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received data: [%f, %f, %f]", msg->data[0], msg->data[1], msg->data[2]);

        // Calculate final orientation for the arm's end-effector
        // The rotation about Z is added to the initial angle (M_PI for tool0 pointing down)
        tf2::Quaternion q_end_effector_orientation;
        q_end_effector_orientation.setRPY(0, init_angle_, msg->data[2]); // Roll, Pitch, Yaw
        geometry_msgs::msg::Quaternion final_ros_orientation = tf2::toMsg(q_end_effector_orientation);

        // Define bolt properties (adjust these based on your actual bolt and setup)
        const std::string bolt_id = "target_bolt";
        const double bolt_radius = 0.02;
        const double bolt_height = 0.065;

        // Assuming table is at Z=0. If not, adjust bolt_z_on_table and bolt_center_z.
        const double bolt_center_z = 0.0325; // Assuming table at Z=0, bolt_height/2

        // Define gripper finger links for allowing collisions
        const std::vector<std::string> gripper_finger_links = {
            "robotiq_85_left_knuckle_link",
            "robotiq_85_left_finger_link",
            "robotiq_85_right_knuckle_link",
            "robotiq_85_right_finger_link"};

        // Create the MTC task
        mtc::Task task;
        task.stages()->setName("Pick and Place Bolt");
        task.loadRobotModel(shared_from_this()); // Keep this as it's the correct way for the Node object

        // Set properties for the task (e.g., planning groups, end-effector)
        task.setProperty("group", arm_group_name_);
        task.setProperty("eef", hand_group_name_);
        task.setProperty("ik_frame", eef_link_);

        // Solvers for different stages
        auto pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->get_node_base_interface(), "ompl");
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        auto joint_interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();


        // --- Stage 1: Current State ---
        // This stage provides the initial robot state and current planning scene.
        mtc::Stage* current_state_ptr = nullptr;
        {
            auto stage = std::make_unique<mtc::stages::CurrentState>("current_state");
            current_state_ptr = stage.get();
            task.add(std::move(stage));
        }

        // --- Stage 2: Open Gripper (Pre-Pick) ---
        // Moves the gripper to the "open" pose.
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open_gripper", joint_interpolation_planner);
            stage->setGroup(hand_group_name_);
            stage->setGoal("open");
            task.add(std::move(stage));
        }

        // --- Stage 3: Move to Pre-Grasp Pose ---
        // This is a free motion to a pose above the object.
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_pre_grasp", pipeline_planner);
            stage->setGroup(arm_group_name_);
            stage->setGoal(createPoseStamped(msg->data[0], msg->data[1], height_, final_ros_orientation));
            stage->properties().configureInitFrom(mtc::Stage::PREDECESSOR, {"group"}); // FIX: Revert to mtc::Stage::PREDECESSOR
            task.add(std::move(stage));
        }

        // --- Stage 4: Add Collision Object (Bolt) ---
        // Adds the bolt to the planning scene. This is a scene modification.
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("add_bolt");

            // Create the collision object
            moveit_msgs::msg::CollisionObject bolt_object;
            bolt_object.header.frame_id = world_frame_;
            bolt_object.id = bolt_id;

            shape_msgs::msg::SolidPrimitive cylinder;
            cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            cylinder.dimensions.resize(2);
            cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = bolt_height;
            cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = bolt_radius;

            geometry_msgs::msg::Pose bolt_pose;
            bolt_pose.position.x = msg->data[0];
            bolt_pose.position.y = msg->data[1];
            bolt_pose.position.z = bolt_center_z; // Object's center Z, assuming base at Z=0
            
            // For the bolt's orientation in the world, use a quaternion representing its upright pose with the given yaw
            tf2::Quaternion bolt_q;
            bolt_q.setRPY(0, 0, msg->data[2]); // Assuming bolt stands upright (0 roll, 0 pitch)
            bolt_pose.orientation = tf2::toMsg(bolt_q);

            bolt_object.primitives.push_back(cylinder);
            bolt_object.primitive_poses.push_back(bolt_pose);
            bolt_object.operation = moveit_msgs::msg::CollisionObject::ADD;
            stage->addObject(bolt_object);
            task.add(std::move(stage));
        }

        // --- Pick Sequence (Serial Container) ---
        // This container holds all stages related to the picking action.
        mtc::Stage* attach_object_stage = nullptr; // Pointer to be forwarded for place
        {
            auto pick = std::make_unique<mtc::SerialContainer>("pick_bolt");
            task.properties().exposeTo(pick->properties(), {"group", "eef", "ik_frame"});

            // --- Approach (Cartesian Path) ---
            // Move straight down to the pick height.
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
                stage->setGroup(arm_group_name_);
                stage->setIKFrame(eef_link_);
                stage->setDirection(createVector3d(0.0, 0.0, pick_height_ - height_, eef_link_)); // FIX: Pass eef_link_ as frame_id
                pick->add(std::move(stage));
            }

            // --- Allow Contact (Gripper and Object) ---
            // Temporarily allows collision between gripper and object during grasp.
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow_contact_gripper_object");
                stage->allowCollisions(bolt_id, gripper_finger_links, true);
                pick->add(std::move(stage));
            }

            // --- Close Gripper (Grasp) ---
            // Moves the gripper to the "close" pose.
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("close_gripper", joint_interpolation_planner);
                stage->setGroup(hand_group_name_);
                stage->setGoal("close");
                pick->add(std::move(stage));
            }

            // --- Attach Object to Gripper ---
            // Modifies the planning scene to attach the bolt to the end-effector.
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach_object");
                stage->attachObject(bolt_id, eef_link_, gripper_finger_links);
                attach_object_stage = stage.get(); // Save pointer for later use in place
                pick->add(std::move(stage));
            }

            // --- Lift (Cartesian Path) ---
            // Move straight up after grasping.
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner);
                stage->setGroup(arm_group_name_);
                stage->setIKFrame(eef_link_);
                stage->setDirection(createVector3d(0.0, 0.0, carrying_height_ - pick_height_, eef_link_)); // FIX: Pass eef_link_ as frame_id
                pick->add(std::move(stage));
            }
            task.add(std::move(pick));
        }

        // --- Stage 5: Move to Place Location ---
        // Free motion to a safe position above the drop-off location.
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_place_location", pipeline_planner);
            stage->setGroup(arm_group_name_);
            // Example place location: 0.3m in X, -0.3m in Y, at carrying height
            stage->setGoal(createPoseStamped(0.3, -0.3, carrying_height_, final_ros_orientation));
            stage->properties().configureInitFrom(mtc::Stage::PREDECESSOR, {"group"}); // FIX: Revert to mtc::Stage::PREDECESSOR
            task.add(std::move(stage));
        }

        // --- Place Sequence (Serial Container) ---
        // This container holds all stages related to the placing action.
        {
            auto place = std::make_unique<mtc::SerialContainer>("place_bolt");
            task.properties().exposeTo(place->properties(), {"group", "eef", "ik_frame"});

            // --- Lower (Cartesian Path) ---
            // Move straight down to the actual drop-off height.
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("lower", cartesian_planner);
                stage->setGroup(arm_group_name_);
                stage->setIKFrame(eef_link_);
                // Assuming drop height is pick_height_ for symmetry or adjust as needed
                stage->setDirection(createVector3d(0.0, 0.0, pick_height_ - carrying_height_, eef_link_)); // FIX: Pass eef_link_ as frame_id
                place->add(std::move(stage));
            }

            // --- Open Gripper (Release) ---
            {
                auto stage = std::make_unique<mtc::stages::MoveTo>("open_gripper_release", joint_interpolation_planner);
                stage->setGroup(hand_group_name_);
                stage->setGoal("open");
                place->add(std::move(stage));
            }

            // --- Detach Object from Gripper ---
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach_object");
                stage->detachObject(bolt_id, eef_link_);
                place->add(std::move(stage));
            }

            // --- Forbid Contact (Gripper and Object) ---
            // Restores collision checking between gripper and object.
            {
                auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid_contact_gripper_object");
                stage->allowCollisions(bolt_id, gripper_finger_links, false);
                place->add(std::move(stage));
            }

            // --- Retreat (Cartesian Path) ---
            // Move straight up after releasing.
            {
                auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
                stage->setGroup(arm_group_name_);
                stage->setIKFrame(eef_link_);
                stage->setDirection(createVector3d(0.0, 0.0, 0.05, eef_link_)); // FIX: Pass eef_link_ as frame_id
                place->add(std::move(stage));
            }
            task.add(std::move(place));
        }

        // --- Stage 6: Move to Home Position ---
        // Returns the arm to a safe home position.
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_home", pipeline_planner);
            stage->setGroup(arm_group_name_);
            stage->setGoal("home");
            stage->properties().configureInitFrom(mtc::Stage::PREDECESSOR, {"group"}); // FIX: Revert to mtc::Stage::PREDECESSOR
            task.add(std::move(stage));
        }

        // --- Planning and Execution ---
        try
        {
            task.init();
        }
        catch (mtc::InitStageException& e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), e.what()); // Use e.what() to get string description
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Starting task planning...");
        if (task.plan())
        {
            RCLCPP_INFO(this->get_logger(), "Task planning succeeded. Executing task...");
            task.publishAllSolutions(task.solutions()); // Publish all found solutions to RViz for debugging
            auto result = task.execute(*task.solutions().front()); // Execute the first found solution
            if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "Task execution succeeded!");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Task execution failed! Error code: %d", result.val);
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Task planning failed!");
        }
    }

    // Helper function to create a PoseStamped message
    geometry_msgs::msg::PoseStamped createPoseStamped(double x, double y, double z, const geometry_msgs::msg::Quaternion& orientation)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = world_frame_;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        pose.pose.orientation = orientation;
        return pose;
    }

    // Modified Helper function to create a Vector3d message (now Vector3Stamped)
    geometry_msgs::msg::Vector3Stamped createVector3d(double x, double y, double z, const std::string& frame_id)
    {
        geometry_msgs::msg::Vector3Stamped vec_stamped;
        vec_stamped.header.frame_id = frame_id;
        vec_stamped.vector.x = x;
        vec_stamped.vector.y = y;
        vec_stamped.vector.z = z;
        return vec_stamped;
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;

    // MoveIt Task Constructor related members
    std::string arm_group_name_;
    std::string hand_group_name_;
    std::string eef_link_;
    std::string world_frame_;

    double height_;
    double pick_height_;
    double carrying_height_;
    double init_angle_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true); // Allow declaring parameters from CLI/launch file
    auto controller_node = std::make_shared<Controller>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(controller_node);

    RCLCPP_INFO(controller_node->get_logger(), "Executor is set up with the controller node.");

    // Spin the executor in a separate thread to allow the main thread to continue
    std::thread([&executor]() { executor.spin(); }).detach();

    RCLCPP_INFO(controller_node->get_logger(), "Controller node is spinning.");

    // Keep the main thread alive until shutdown
    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    rclcpp::shutdown();

    return 0;
}