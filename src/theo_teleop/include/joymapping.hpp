#ifndef THEODWYNROS_THEOTELEOP_JOYMAPPING_
#define THEODWYNROS_THEOTELEOP_JOYMAPPING_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "theo_msgs/msg/theo_servo_cmd.hpp"

#include <vector>
#include <cstdint>
#include <string>

namespace Theodwyn
{
    class JoyMapping : public rclcpp::Node
    {
    public:
        enum class TeleopMode : int8_t
        {
            On,
            Off
        };
        JoyMapping();

    private:
        // ROS members
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr authority_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_publisher_;
        rclcpp::Publisher<theo_msgs::msg::TheoServoCmd>::SharedPtr pantilt_publisher_;

        // ROS Callbacks
        void mappingCallback(const sensor_msgs::msg::Joy::SharedPtr);
        void authorityCallback(const std_msgs::msg::Bool::SharedPtr);

        // Functional Parameters
        double chassis_max_linspeed_;
        double chassis_max_angspeed_;
        double pantilt_maxspeed_;
        double settings_update_waitime_;
        bool speed_locked_;
        rclcpp::Time last_settings_update_;
        TeleopMode mode_;

        // Functional Parameter Defaults
        static constexpr double DEFAULT_chassis_max_linspeed_ = 1.0;
        static constexpr double DEFAULT_chassis_max_angspeed_ = 1.0;
        static constexpr double DEFAULT_pantilt_maxspeed_ = 1.57;
        static constexpr double DEFAULT_settings_update_waitime_ = 0.5;
        static constexpr double DEFAULT_speed_locked_ = true;
        static constexpr std::string_view DEFAULT_joy_topic_ = "joy";
        static constexpr std::string_view DEFAULT_joyauthority_topic_ = "joy_authority";
        static constexpr std::string_view DEFAULT_velcmd_topic_ = "velcmd_topic";
        static constexpr std::string_view DEFAULT_servocmd_topic_ = "servocmd_topic";
    };
}

#endif