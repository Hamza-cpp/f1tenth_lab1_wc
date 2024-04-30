#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class RelayNode : public rclcpp::Node
{
public:
    RelayNode() : Node("my_relay_node_in_cpp")
    {
        subscriber_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive", 10, std::bind(&RelayNode::driveCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive_relay", 10);

        RCLCPP_INFO(this->get_logger(), "Relay Node has been started, gog go go!");
    }

private:
    void driveCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        auto new_msg = std::make_shared<ackermann_msgs::msg::AckermannDriveStamped>();
        new_msg->drive.speed = msg->drive.speed * 3;
        new_msg->drive.steering_angle = msg->drive.steering_angle * 3;
        publisher_->publish(*new_msg);
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto relay_node = std::make_shared<RelayNode>();
    rclcpp::spin(relay_node);
    rclcpp::shutdown();
    return 0;
}
