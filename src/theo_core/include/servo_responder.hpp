#include "rclcpp/rclcpp.hpp"
#include "theo_msgs/msg/theo_servo_cmd.hpp"
#include "theo_msgs/msg/theo_servo_addition.hpp"


struct PantiltVelocityData
{
    double pan_angular_velocity     = 0.0;
    double tilt_angular_velocity    = 0.0;
    bool   expired                  = true;
};


class ServoResponderNode : public rclcpp::Node{

    public:
        ServoResponderNode();

    private:
        rclcpp::Time last_vel_update_time_;
        double update_timeout_seconds_;
        PantiltVelocityData last_vel_cmd_;
        rclcpp::Subscription<theo_msgs::msg::TheoServoCmd>::SharedPtr subscription_;
        rclcpp::Publisher<theo_msgs::msg::TheoServoAddition>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        void cmdCallback( const theo_msgs::msg::TheoServoCmd::SharedPtr );
        void publish_pantilt_command( double );
        void timerCallback();
    
};
