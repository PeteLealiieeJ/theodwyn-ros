#ifndef THEODWYNROS_THEOCORE_MIXER_
#define THEODWYNROS_THEOCORE_MIXER_

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "theo_msgs/msg/theo_mechanum_cmd.hpp"

namespace Theodwyn
{
    /**
     * @brief Node that converts body velocity commands (Twist) into four normalized
     *        mecanum wheel throttles.
     */
    class MixerNode : public rclcpp::Node
    {
    public:
        /// @brief Declares parameters, builds the mixing matrix, and creates the subscription and publisher.
        MixerNode();

    private:
        /// Default wheel radius (r_wheel) [m].
        static constexpr double DEFAULT_r_wheel_ = 1.;
        /// Default half-length lx of the wheel base [m].
        static constexpr double DEFAULT_lx_ = 1.;
        /// Default half-width ly of the wheel base [m].
        static constexpr double DEFAULT_ly_ = 1.;
        /// Default wheel angular rate corresponding to full throttle [rad/s].
        static constexpr double DEFAULT_omega_ref_ = 1.;
        /// Default direction flip for wheel 0.
        static constexpr bool DEFAULT_flipdir_0_ = false;
        /// Default direction flip for wheel 1.
        static constexpr bool DEFAULT_flipdir_1_ = false;
        /// Default direction flip for wheel 2.
        static constexpr bool DEFAULT_flipdir_2_ = false;
        /// Default direction flip for wheel 3.
        static constexpr bool DEFAULT_flipdir_3_ = false;
        /// Default topic for incoming velocity commands.
        static constexpr std::string_view DEFAULT_velcmd_topic_ = "velcmd_topic";
        /// Default topic for outgoing throttle commands.
        static constexpr std::string_view DEFAULT_thrtlcmd_topic_ = "thrtlcmd_topic";

        /// 4x3 matrix mapping [vx, vy, wz] to the four wheel throttles (includes scaling and direction flips).
        Eigen::Matrix<double, 4, 3> MixMat;

        /// Subscription to velocity commands.
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        /// Publisher of wheel throttle commands.
        rclcpp::Publisher<theo_msgs::msg::TheoMechanumCmd>::SharedPtr publisher_;

        /**
         * @brief Mixes the commanded linear x/y and angular z velocity into wheel throttles and publishes them.
         * @param msg_in Velocity command.
         */
        void mixingCallback(const geometry_msgs::msg::Twist::SharedPtr) const;
    };
}

#endif
