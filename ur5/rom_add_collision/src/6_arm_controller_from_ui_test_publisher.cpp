#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher()
    : Node("simple_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/target_point", 10);
        RCLCPP_INFO(this->get_logger(), "SimplePublisher node has been initialized. Preparing to publish once to /target_point.");

        publish_single_message();
    }

private:
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
        
        RCLCPP_INFO(this->get_logger(), "Message published. Signaling for shutdown.");
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimplePublisher>();
    
    rclcpp::spin_some(node);
    
    if (rclcpp::ok()) { 
        rclcpp::shutdown();
    }
    
    return 0;
}