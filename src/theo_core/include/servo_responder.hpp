#ifndef THEODWYNROS_THEOCORE_SERVORESPONDER_
#define THEODWYNROS_THEOCORE_SERVORESPONDER_

#include "rclcpp/rclcpp.hpp"
#include "theo_msgs/msg/theo_servo_cmd.hpp"
#include "theo_msgs/msg/theo_servo_addition.hpp"

namespace Theodwyn
{
    class ServoResponderNode : public rclcpp::Node
    {
    public:
        ServoResponderNode();

        struct PantiltVelocityData
        {
            double pan_angular_velocity = 0.0;
            double tilt_angular_velocity = 0.0;
            bool expired = true;
        };

    private:
        // Functional Parameter Defaults
        static constexpr double DEFAULT_update_timeout_ = 0.1;
        static constexpr std::string_view DEFAULT_servocmd_topic_ = "servocmd_topic";
        static constexpr std::string_view DEFAULT_servoadd_topic_ = "servoadd_topic";

        // Functional Members
        rclcpp::Time last_vel_update_time_;
        double update_timeout_seconds_;
        PantiltVelocityData last_vel_cmd_;

        // Functional Methods
        void publish_pantilt_command(double);

        // ROS members
        rclcpp::Subscription<theo_msgs::msg::TheoServoCmd>::SharedPtr subscription_;
        rclcpp::Publisher<theo_msgs::msg::TheoServoAddition>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        // ROS Callbacks
        void cmdCallback(const theo_msgs::msg::TheoServoCmd::SharedPtr);
        void timerCallback();
    };

}

#endif