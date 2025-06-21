#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
//#include <moveit/task_constructor/solvers/planning_pipeline.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

using namespace moveit::task_constructor;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("mtc_cartesian_demo", options);

  Task task("mycobot_mtc_cartesian");
  task.loadRobotModel(node);

  auto current = std::make_unique<stages::CurrentState>("current");
  task.add(std::move(current));

  auto pipeline = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>(node);
  auto move_to = std::make_unique<stages::MoveTo>("approach", pipeline);
  move_to->setGroup("arm");

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.20;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.15;
  pose.pose.orientation.w = 1.0;

  move_to->setGoal(pose);
  task.add(std::move(move_to));

  if (!task.plan()) {
    RCLCPP_ERROR(node->get_logger(), "Planning failed");
    return 1;
  }

  task.introspection().publishSolution(*task.solutions().front());
  task.execute(*task.solutions().front());

  rclcpp::shutdown();
  return 0;
}
