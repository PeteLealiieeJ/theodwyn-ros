#ifndef THEODWYNROS_THEOCORE_SERVORESPONDER_
#define THEODWYNROS_THEOCORE_SERVORESPONDER_

#include "rclcpp/rclcpp.hpp"
#include "theo_msgs/msg/theo_servo_cmd.hpp"
#include "theo_msgs/msg/theo_servo_addition.hpp"

namespace Theodwyn
{
    /**
     * @brief Node that integrates pan/tilt velocity commands into incremental angle
     *        additions for the servo.
     *
     * While a recent command exists, each timer tick publishes velocity * elapsed time.
     * Commands older than the update timeout are expired and nothing is published.
     */
    class ServoResponderNode : public rclcpp::Node
    {
    public:
        /// @brief Declares parameters and creates the subscription, publisher, and 10 ms timer.
        ServoResponderNode();

        /// @brief Most recent pan/tilt velocity command.
        struct PantiltVelocityData
        {
            double pan_angular_velocity = 0.0;  ///< Pan rate [rad/s]
            double tilt_angular_velocity = 0.0; ///< Tilt rate [rad/s]
            bool expired = true;                ///< True if no valid (recent) command is held
        };

    private:
        /// Default command timeout [s].
        static constexpr double DEFAULT_update_timeout_ = 0.1;
        /// Default topic for incoming servo velocity commands.
        static constexpr std::string_view DEFAULT_servocmd_topic_ = "servocmd_topic";
        /// Default topic for outgoing servo angle additions.
        static constexpr std::string_view DEFAULT_servoadd_topic_ = "servoadd_topic";

        /// Time the last velocity command was received.
        rclcpp::Time last_vel_update_time_;
        /// Seconds after which a velocity command is considered stale.
        double update_timeout_seconds_;
        /// Last received velocity command.
        PantiltVelocityData last_vel_cmd_;

        /**
         * @brief Publishes the angle increment (velocity * dt) for pan and tilt.
         * @param dt Seconds since the command was last updated.
         */
        void publish_pantilt_command(double);

        /// Subscription to servo velocity commands.
        rclcpp::Subscription<theo_msgs::msg::TheoServoCmd>::SharedPtr subscription_;
        /// Publisher of servo angle additions.
        rclcpp::Publisher<theo_msgs::msg::TheoServoAddition>::SharedPtr publisher_;
        /// Timer driving timerCallback().
        rclcpp::TimerBase::SharedPtr timer_;

        /// @brief Stores the incoming velocity command and its receive time.
        void cmdCallback(const theo_msgs::msg::TheoServoCmd::SharedPtr);

        /// @brief Expires stale commands, otherwise publishes the angle increment.
        void timerCallback();
    };

}

#endif
