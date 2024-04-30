#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

/**
 * @brief A class representing a talker node.
 * 
 * The TalkerNode class is a subclass of rclcpp::Node and is responsible for publishing
 * AckermannDriveStamped messages to the "drive" topic. It retrieves the values of the
 * "d" and "v" parameters and uses them to populate the AckermannDriveStamped message.
 * The node also creates a timer that triggers the publish_drive() function at a specified
 * interval of 1ms.
 */
class TalkerNode : public rclcpp::Node
{
public:
    TalkerNode() : Node("my_talker_node_in_cpp")
    {
        d_param = this->declare_parameter("d", 0.0);
        v_param = this->declare_parameter("v", 0.0);
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        timer_period = std::chrono::milliseconds(1);
        timer_ = this->create_wall_timer(timer_period, std::bind(&TalkerNode::publish_drive, this));
        RCLCPP_INFO(this->get_logger(), "Talker Node has been started, gog go go!");
        RCLCPP_INFO(this->get_logger(), "Parameters received: v=%f, d=%f", v_param, d_param);
    }

private:
    double d_param;
    double v_param;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    std::chrono::milliseconds timer_period;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_drive()
    {
        auto msg = std::make_shared<ackermann_msgs::msg::AckermannDriveStamped>();
        msg->drive.speed = v_param ? v_param : 0.0;
        msg->drive.steering_angle = d_param ? d_param : 0.0;
        publisher_->publish(*msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto talker_node = std::make_shared<TalkerNode>();
    rclcpp::spin(talker_node);
    rclcpp::shutdown();
    return 0;
}