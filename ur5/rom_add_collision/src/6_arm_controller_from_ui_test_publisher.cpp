#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher()
    : Node("simple_publisher")
    {
        // Create the publisher
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/target_point", 10);
        RCLCPP_INFO(this->get_logger(), "SimplePublisher node has been initialized. Preparing to publish once to /target_point.");
    }

    void publish_single_message()
    {
        auto message = std_msgs::msg::Float64MultiArray();
        
        double x_val = -0.2;
        double y_val = -0.2;
        double angle_val = 1.0; 

        message.data.push_back(x_val);
        message.data.push_back(y_val);
        message.data.push_back(angle_val);

        RCLCPP_INFO(this->get_logger(), "Publishing once: [%f, %f, %f]", message.data[0], message.data[1], message.data[2]);
        publisher_->publish(message);
        
        RCLCPP_INFO(this->get_logger(), "Message published. Keeping node alive briefly for propagation.");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimplePublisher>();
    
    rclcpp::Rate loop_rate(1);
    loop_rate.sleep();
    
    node->publish_single_message();
    
    auto start_time = node->now();
    auto end_time = start_time + rclcpp::Duration(2, 0); 
    
    while (rclcpp::ok() && node->now() < end_time) 
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }
    
    RCLCPP_INFO(node->get_logger(), "Publisher node finished its operation and is shutting down.");
    rclcpp::shutdown();
    
    return 0;
}
