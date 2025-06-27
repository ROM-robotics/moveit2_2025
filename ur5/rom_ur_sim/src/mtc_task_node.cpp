#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/stages/current_state.h>
#include <moveit_task_constructor/stages/fixed_state.h>
#include <moveit_task_constructor/stages/move_to.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_interface/planning_interface.h>

using namespace moveit::task_constructor;

class MTCSimpleNode : public rclcpp::Node {
public:
  MTCSimpleNode() : Node("mtc_task_node") {
    task_ = std::make_shared<Task>("simple_task");
    task_->loadRobotModel(this->get_name());

    // Build a simple task pipeline
    auto current_state = std::make_unique<stages::CurrentState>("current state");
    task_->add(std::move(current_state));

    auto move_to = std::make_unique<stages::MoveTo>("move to home", pipeline_stage());
    move_to->setGroup("manipulator");
    move_to->setGoal("home");
    task_->add(std::move(move_to));

    if (!task_->init()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize task");
    }

    task_->publishSolution(*task_->solutions().front());
  }

private:
  std::shared_ptr<Task> task_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MTCSimpleNode>());
  rclcpp::shutdown();
  return 0;
}
