#include <rclcpp/rclcpp.hpp>                 
//#include <moveit_msgs/msg/CollisionObject.hpp> 
//#include <shape_msgs/msg/SolidPrimitive.hpp> 
//#include <geometry_msgs/msg/Pose.hpp>       
#include <chrono>                            
#include <thread>                            
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>        

class AddCollisionObjectNode : public rclcpp::Node
{
public:
  AddCollisionObjectNode()
  : Node("add_collision_object_node") 
  {
    RCLCPP_INFO(this->get_logger(), "Initializing AddCollisionObjectNode...");
    
    collision_object_publisher_ = this->create_publisher<moveit_msgs::msg::CollisionObject>("collision_object", 10);

    // Give some time for the publisher to establish connection with MoveIt's subscribers.
    // This is a good practice to ensure the message is received, especially at startup.
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    publishTable();

    RCLCPP_INFO(this->get_logger(), "AddCollisionObjectNode initialized and table published.");
  }

private:
  void publishTable()
  {
    moveit_msgs::msg::CollisionObject table_object;
    
    table_object.header.frame_id = "world";
    table_object.header.stamp = this->now();

    // --- Set the Object ID ---
    // A unique ID for this collision object. MoveIt! uses this ID to identify,
    // modify, or remove the object later.
    table_object.id = "work_table";

    // --- Define the Shape ---
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX; // Specify the shape type as a BOX

    // Set the dimensions of the box (length, width, height) in meters.
    // Adjust these values to match the size of your simulated table in Ignition Gazebo.
    primitive.dimensions.resize(3); // Resize to hold 3 dimensions (x, y, z for a box)
    primitive.dimensions[0] = 0.8;  
    primitive.dimensions[1] = 1.5;  
    primitive.dimensions[2] = 1.015; 

    // --- Define the Pose (Position and Orientation) ---
    // This pose defines the *center* of the primitive relative to the `frame_id` ("world").
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0.29; // X position of the table's center
    pose.position.y = 0.0; // Y position of the table's center

    // Z position: If the table's total height is 0.8m and its base is on the ground (z=0),
    // its center will be at half its height (0.4m).
    pose.position.z = primitive.dimensions[2] / 2.0;

    // Orientation as a quaternion. Setting w=1.0 and x,y,z=0.0 means no rotation (identity).
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.7071;
    pose.orientation.w = 0.7071;

    // Add the defined primitive shape and its pose to the collision object message.
    table_object.primitives.push_back(primitive);
    table_object.primitive_poses.push_back(pose);

    // --- Set the Operation ---
    // This specifies what action to perform with this object in the planning scene.
    // `ADD` means to add this new object. Other options include `REMOVE`, `APPEND`, `MOVE`.
    table_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // --- Publish the Collision Object ---
    RCLCPP_INFO(this->get_logger(), "Attempting to publish table collision object '%s'...", table_object.id.c_str());
    collision_object_publisher_->publish(table_object);

    // It's often helpful to publish multiple times or with a small delay
    // to ensure reliable delivery, especially for static objects at startup.
    // MoveIt!'s PlanningSceneMonitor will process these updates.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    collision_object_publisher_->publish(table_object);
    RCLCPP_INFO(this->get_logger(), "Table collision object published successfully.");
  }

  // Declare the publisher for CollisionObject messages
  rclcpp::Publisher<moveit_msgs::msg::CollisionObject>::SharedPtr collision_object_publisher_;
};

// Main function to run the ROS 2 node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // Initialize ROS 2
  // Create an instance of our AddCollisionObjectNode
  auto node = std::make_shared<AddCollisionObjectNode>();
  // Spin the node, keeping it alive and processing callbacks until shutdown
  rclcpp::spin(node);
  rclcpp::shutdown(); // Shutdown ROS 2
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

