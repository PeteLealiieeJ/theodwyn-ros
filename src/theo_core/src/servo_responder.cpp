#include "servo_responder.hpp"

using namespace Theodwyn;

ServoResponderNode::ServoResponderNode()
    : Node("servo_responder_node")
{
    declare_parameter<double>("update_timeout", DEFAULT_update_timeout_);
    declare_parameter<std::string>("servo_cmd_topic", std::string(DEFAULT_servocmd_topic_));
    declare_parameter<std::string>("servo_add_topic", std::string(DEFAULT_servoadd_topic_));

    get_parameter("update_timeout", update_timeout_seconds_);
    std::string servo_cmd_topic, servo_add_topic;
    get_parameter("servo_cmd_topic", servo_cmd_topic);
    get_parameter("servo_add_topic", servo_add_topic);

    // Setup pub/sub+
    subscription_ = create_subscription<theo_msgs::msg::TheoServoCmd>(
        servo_cmd_topic,
        10,
        std::bind(
            &ServoResponderNode::cmdCallback,
            this,
            std::placeholders::_1));

    publisher_ = create_publisher<theo_msgs::msg::TheoServoAddition>(
        servo_add_topic,
        10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ServoResponderNode::timerCallback, this));

    last_vel_update_time_ = get_clock()->now();
    return;
};

void ServoResponderNode::cmdCallback(const theo_msgs::msg::TheoServoCmd::SharedPtr msg)
{
    (last_vel_cmd_).pan_angular_velocity = msg->pan_vel;
    (last_vel_cmd_).tilt_angular_velocity = msg->tilt_vel;
    (last_vel_cmd_).expired = false;
    last_vel_update_time_ = get_clock()->now();
};

void ServoResponderNode::publish_pantilt_command(double dt)
{
    theo_msgs::msg::TheoServoAddition add_msg_out;
    add_msg_out.pan_angle = (last_vel_cmd_.pan_angular_velocity) * dt;
    add_msg_out.tilt_angle = (last_vel_cmd_.tilt_angular_velocity) * dt;
    publisher_->publish(add_msg_out);
};

void ServoResponderNode::timerCallback()
{

    if ((last_vel_cmd_).expired)
        return;

    double dt = rclcpp::Duration((get_clock()->now()) - (last_vel_update_time_)).nanoseconds() * 1e-9;
    if (dt > (update_timeout_seconds_))
        last_vel_cmd_.expired = true;
    else
        publish_pantilt_command(dt);
};

// -------------------------------------------------------------------------------------------------------------
// Main
// -------------------------------------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ServoResponderNode> node = std::make_shared<ServoResponderNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
