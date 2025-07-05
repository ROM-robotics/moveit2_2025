#include <rclcpp/rclcpp.hpp>                 
//#include <moveit_msgs/msg/CollisionObject.hpp> 
//#include <shape_msgs/msg/SolidPrimitive.hpp> 
//#include <geometry_msgs/msg/Pose.hpp>       
#include <chrono>                            
#include <thread>                            
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>    

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h" // Not strictly needed for this, but useful
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 

class AddCollisionObjectNode : public rclcpp::Node
{
public:
  AddCollisionObjectNode()
  : Node("add_collision_object_node") 
  {
    RCLCPP_INFO(this->get_logger(), "Initializing AddCollisionObjectNode...");
    
    collision_object_publisher_ = this->create_publisher<moveit_msgs::msg::CollisionObject>("collision_object", 10);
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    publishTable();
    //publishBolt1();

    RCLCPP_INFO(this->get_logger(), "AddCollisionObjectNode initialized and table published.");
  }

private:
  void publishTable()
  {
    moveit_msgs::msg::CollisionObject table_object;
    
    table_object.header.frame_id = "world";
    table_object.header.stamp = this->now();
    
    table_object.id = "work_table";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    
    primitive.dimensions.resize(3); 
    primitive.dimensions[0] = 0.8;  
    primitive.dimensions[1] = 1.5;  
    primitive.dimensions[2] = 1.015; 
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.29; 
    pose.position.y = 0.0;
    
    pose.position.z = primitive.dimensions[2] / 2.0;
    
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.7071;
    pose.orientation.w = 0.7071;
    
    table_object.primitives.push_back(primitive);
    table_object.primitive_poses.push_back(pose);
    table_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    
    RCLCPP_INFO(this->get_logger(), "Attempting to publish table collision object '%s'...", table_object.id.c_str());
    collision_object_publisher_->publish(table_object);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    collision_object_publisher_->publish(table_object);
    RCLCPP_INFO(this->get_logger(), "Table collision object published successfully.");
  }

  void publishBolt1()
  {
    moveit_msgs::msg::CollisionObject bolt_object;
    
    bolt_object.header.frame_id = "world";
    bolt_object.header.stamp = this->now();
    
    bolt_object.id = "bolt1";
    
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER; 

    primitive.dimensions.resize(2);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.05; 
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.01; 
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.5; 
    pose.position.y = 0.35; 
    pose.position.z = 1.05; 

    double roll = 0.0;
    double pitch = 1.5708; // Approximately PI/2 radians (90 degrees)
    double yaw = 0.0;
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(roll, pitch, yaw); 
    
    tf_quat.normalize();
    
    pose.orientation = tf2::toMsg(tf_quat);
    
    bolt_object.primitives.push_back(primitive);
    bolt_object.primitive_poses.push_back(pose);
    
    bolt_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    
    RCLCPP_INFO(this->get_logger(), "Attempting to publish bolt1 collision object '%s'...", bolt_object.id.c_str());

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    collision_object_publisher_->publish(bolt_object);
    RCLCPP_INFO(this->get_logger(), "Bolt1 collision object published successfully.");
  }
  
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr collision_object_publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<AddCollisionObjectNode>();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


/*
TABLE POSE IN WORLD SDF
<model name="table">
      <static>true</static>
      <include>
        <uri>file:///home/mr_robot/devel_ws/install/rom_ur_sim/share/rom_ur_sim/worlds/table/</uri>
      </include>
      <pose>0.29 0 0 0 0 1.5708</pose>
    </model>


TABLE SIZE IN TABLE MODEL

Width: 1.5 meters (x-axis)
Depth: 0.8 meters (y-axis) <======
Thickness: 0.03 meters (z-axis)
Top surface height: 1.0 + 0.03/2 = 1.015 m <======
Bottom surface height: 1.0 - 0.03/2 = 0.985 m

height = 1.015
width = 1.5
depth = 0.8

*/

