#ifndef THEODWYNROS_THEOCORE_MIXER_
#define THEODWYNROS_THEOCORE_MIXER_

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "theo_msgs/msg/theo_mechanum_cmd.hpp"

namespace Theodwyn
{
    class MixerNode : public rclcpp::Node
    {
    public:
        MixerNode();

    private:
        // Functional Parameter Defaults
        static constexpr double DEFAULT_r_wheel_ = 1.;
        static constexpr double DEFAULT_lx_ = 1.;
        static constexpr double DEFAULT_ly_ = 1.;
        static constexpr double DEFAULT_omega_ref_ = 1.;
        static constexpr bool DEFAULT_flipdir_0_ = false;
        static constexpr bool DEFAULT_flipdir_1_ = false;
        static constexpr bool DEFAULT_flipdir_2_ = false;
        static constexpr bool DEFAULT_flipdir_3_ = false;
        static constexpr std::string_view DEFAULT_velcmd_topic_ = "velcmd_topic";
        static constexpr std::string_view DEFAULT_thrtlcmd_topic_ = "thrtlcmd_topic";

        // Functional Members
        Eigen::Matrix<double, 4, 3> MixMat;

        // ROS members
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        rclcpp::Publisher<theo_msgs::msg::TheoMechanumCmd>::SharedPtr publisher_;

        // ROS Callbacks
        void mixingCallback(const geometry_msgs::msg::Twist::SharedPtr) const;
    };
}

#endif