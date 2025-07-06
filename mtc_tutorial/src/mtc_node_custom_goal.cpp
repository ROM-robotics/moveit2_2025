#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.h>
#endif

// Include for Eigen to convert Euler angles to Quaternion
#include <Eigen/Geometry>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

  MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options) : node_{ std::make_shared<rclcpp::Node>("mtc_node", options) }
  {
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
  {
    return node_->get_node_base_interface();
  }

  void MTCTaskNode::setupPlanningScene()
  {
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "base_link";
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { 0.1, 0.02 }; // 10cm, 2cm

    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.2;
    pose.position.y = 0.2;
    pose.orientation.w = 1.0;
    object.pose = pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
  }

  void MTCTaskNode::doTask()
  {
    task_ = createTask();

    try
    {
      task_.init();
    }
    catch (mtc::InitStageException& e)
    {
      RCLCPP_ERROR_STREAM(LOGGER, e);
      return;
    }

    if (!task_.plan(20))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
      return;
    }
    task_.introspection().publishSolution(*task_.solutions().front());

    auto result = task_.execute(*task_.solutions().front());
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
      return;
    }

    return;
  }

  mtc::Task MTCTaskNode::createTask()
  {
    mtc::Task task;
    task.stages()->setName("demo task");
    task.loadRobotModel(node_);

    const auto& arm_group_name = "ur5_manipulator";
    const auto& hand_group_name = "robotiq_gripper";
    const auto& hand_frame = "tool0";
    //const auto& hand_frame = "robotiq_85_left_knuckle_link"; // If you use this, ensure your URDF/SRDF defines IK for it

    // Set task properties
    task.setProperty("group", arm_group_name);
    task.setProperty("eef", hand_group_name);
    task.setProperty("ik_frame", hand_frame); // This is crucial for Cartesian planning

  // Disable warnings for this line, as it's a variable that's set but not used in this example
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator
  #pragma GCC diagnostic pop

    // current state ကို အရင်သွား
    auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage_state_current.get();
    task.add(std::move(stage_state_current));

    // planner ၃ ခု ဆောက်
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(1.0);
    cartesian_planner->setMaxAccelerationScalingFactor(1.0);
    cartesian_planner->setStepSize(.01);

    auto stage_open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
    stage_open_hand->setGroup(hand_group_name);
    stage_open_hand->setGoal("open");
    task.add(std::move(stage_open_hand));

    // Move arm to a CUSTOM CARTESIAN POSITION
    auto stage_move_to_custom_pose = std::make_unique<mtc::stages::MoveTo>("move to custom pose", sampling_planner);
    stage_move_to_custom_pose->setGroup(arm_group_name);
    stage_move_to_custom_pose->setIKFrame(hand_frame); // Explicitly set the IK frame

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link"; // The frame relative to which the pose is defined
    target_pose.pose.position.x = 0.147;         
    target_pose.pose.position.y = -0.574;        
    target_pose.pose.position.z = 0.230;  
    
    // For a downward orientation:
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    // // Example: Orientation for the tool0 frame
    // // Convert RPY (Roll, Pitch, Yaw) to Quaternion
    // // Roll (rotation around X), Pitch (rotation around Y), Yaw (rotation around Z)
    // // All angles in radians
    // Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()) * // 90 degrees around X (Roll)
    //                        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * // 0 degrees around Y (Pitch)
    //                        Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ());   // 45 degrees around Z (Yaw)

    target_pose.pose.orientation.x = q.x();
    target_pose.pose.orientation.y = q.y();
    target_pose.pose.orientation.z = q.z();
    target_pose.pose.orientation.w = q.w();

    stage_move_to_custom_pose->setGoal(target_pose);
    task.add(std::move(stage_move_to_custom_pose));

    // Close the gripper
    auto stage_close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage_close_hand->setGroup(hand_group_name);
    stage_close_hand->setGoal("close"); // Assuming "close" is a defined joint state for your gripper
    task.add(std::move(stage_close_hand));

    return task;
  }

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}