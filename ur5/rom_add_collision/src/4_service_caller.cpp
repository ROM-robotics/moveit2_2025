#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> 
#include <rom_add_collision/srv/gesture_command.hpp>
#include <chrono> 

using namespace std::chrono_literals;
class GestureSubscriber : public rclcpp::Node
{
public:
    GestureSubscriber() : Node("gesture_subscriber")
    {
        // Create a subscriber to the 'gesture_command' topic.
        // QoS history depth is set to 1, meaning only the latest message is kept.
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "gesture_command",
            rclcpp::QoS(1), // QoS history depth: Keep only the latest message
            std::bind(&GestureSubscriber::gesture_callback, this, std::placeholders::_1));

        // Create a service client for the 'gesture_command_service'.
        // This client will be used to send requests to the service server.
        client_ = this->create_client<rom_add_collision::srv::GestureCommand>("gesture_command_service");

        RCLCPP_INFO(this->get_logger(), "Gesture Subscriber Node has been started.");
        RCLCPP_INFO(this->get_logger(), "Waiting for 'gesture_command_service' to be available...");

        // Wait for the service to be available before proceeding.
        // This is important to ensure the client can connect to the server.
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
        RCLCPP_INFO(this->get_logger(), "'gesture_command_service' is available.");
    }

private:
    void gesture_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Gesture Command: '%s'", msg->data.c_str());

        auto request = std::make_shared<rom_add_collision::srv::GestureCommand::Request>();
        if (msg->data == "Pick Object 1") { request->command = "1"; }
        else if (msg->data == "Pick Object 2") { request->command = "2"; }
        else if (msg->data == "Pick Object 3") { request->command = "3"; }
        else if (msg->data == "Pick Object 4") { request->command = "4"; }
        else if (msg->data == "Stop") { request->command = "5"; }
        else  { request->command = "100"; } // not calling

        if (sending_request_== false && request->command != "100")
        {
            sending_request_ = true;
            RCLCPP_INFO(this->get_logger(), "Calling 'gesture_command_service' with command: '%s'", request->command.c_str());

            client_->async_send_request(request,
                [this](rclcpp::Client<rom_add_collision::srv::GestureCommand>::SharedFuture future) {
                    auto response = future.get(); // Get the response from the future
                    if (response->success) {
                        RCLCPP_INFO(this->get_logger(), "Service call successful: '%s'", response->message.c_str());
                        sending_request_ = false;
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Service call failed: '%s'", response->message.c_str());
                        sending_request_ = false;
                    }
                });
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Ignoring unknown gesture command: '%s'", msg->data.c_str());
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::Client<rom_add_collision::srv::GestureCommand>::SharedPtr client_; 
    bool sending_request_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GestureSubscriber>());

    rclcpp::shutdown();
    return 0;
}
