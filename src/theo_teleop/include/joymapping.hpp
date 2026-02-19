#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "theo_msgs/msg/theo_servo_state.hpp"
#include <vector>
#include <cstdint>

enum class TeleopMode : int {
    On,
    Off
};

class JoyMapping : public rclcpp::Node {
    public:
        double chassis_max_linspeed     = 1.0;
        double chassis_max_angspeed     = 1.0;
        double pantilt_maxspeed         = 1.0;
        double settings_update_waitime  = 0.5;
        rclcpp::Time last_settings_update;
        TeleopMode mode_;

        JoyMapping();

    private:
        void mappingCallback( const sensor_msgs::msg::Joy::SharedPtr );
        void authorityCallback( const std_msgs::msg::Bool::SharedPtr ); // <-- 

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr authority_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_publisher_;
        rclcpp::Publisher<theo_msgs::msg::TheoServoState>::SharedPtr pantilt_publisher_;
};
