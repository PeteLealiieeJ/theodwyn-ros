#include "mixer.hpp"

using namespace Theodwyn;

MixerNode::MixerNode()
    : Node("mixer_node")
{

    // Setup Mixing matrix

    double r_wheel, lx, ly, omega_ref;
    bool flipdir_0, flipdir_1, flipdir_2, flipdir_3;

    declare_parameter<double>("r_wheel", DEFAULT_r_wheel_);
    declare_parameter<double>("lx", DEFAULT_lx_);
    declare_parameter<double>("ly", DEFAULT_ly_);
    declare_parameter<double>("omega_ref", DEFAULT_omega_ref_);
    declare_parameter<bool>("flipdir_0", DEFAULT_flipdir_0_);
    declare_parameter<bool>("flipdir_1", DEFAULT_flipdir_1_);
    declare_parameter<bool>("flipdir_2", DEFAULT_flipdir_2_);
    declare_parameter<bool>("flipdir_3", DEFAULT_flipdir_3_);
    declare_parameter<std::string>("vel_cmd_topic", std::string(DEFAULT_velcmd_topic_));
    declare_parameter<std::string>("throttle_cmd_topic", std::string(DEFAULT_thrtlcmd_topic_));

    get_parameter("r_wheel", r_wheel);
    get_parameter("lx", lx);
    get_parameter("ly", ly);
    get_parameter("omega_ref", omega_ref);
    get_parameter("flipdir_0", flipdir_0);
    get_parameter("flipdir_1", flipdir_1);
    get_parameter("flipdir_2", flipdir_2);
    get_parameter("flipdir_3", flipdir_3);
    std::string vel_cmd_topic, throttle_cmd_topic;
    get_parameter("vel_cmd_topic", vel_cmd_topic);
    get_parameter("throttle_cmd_topic", throttle_cmd_topic);

    const double c = lx + ly;
    const double alpha_0 = (flipdir_0) ? -1. : 1.;
    const double alpha_1 = (flipdir_1) ? -1. : 1.;
    const double alpha_2 = (flipdir_2) ? -1. : 1.;
    const double alpha_3 = (flipdir_3) ? -1. : 1.;
    Eigen::Matrix<double, 4, 1> _Fvec{alpha_0, alpha_1, alpha_2, alpha_3};
    Eigen::Matrix<double, 4, 3> _Mu{
        {1., -1., -c},
        {1., 1., -c},
        {1., 1., c},
        {1., -1., c}};

    MixMat = (1. / (r_wheel * omega_ref)) * _Fvec.asDiagonal() * _Mu;

    // Setup pub/sub+
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        vel_cmd_topic,
        10,
        std::bind(
            &MixerNode::mixingCallback,
            this,
            std::placeholders::_1));

    publisher_ = create_publisher<theo_msgs::msg::TheoMechanumCmd>(
        throttle_cmd_topic,
        10);
}

void MixerNode::mixingCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg_in) const
{
    Eigen::Matrix<double, 3, 1> twist_in{
        msg_in->linear.x,
        msg_in->linear.y,
        msg_in->angular.z};
    Eigen::Matrix<double, 4, 1> throttles_out = MixMat * twist_in;

    theo_msgs::msg::TheoMechanumCmd msg_out;
    msg_out.throttle_0 = throttles_out(0);
    msg_out.throttle_1 = throttles_out(1);
    msg_out.throttle_2 = throttles_out(2);
    msg_out.throttle_3 = throttles_out(3);

    publisher_->publish(msg_out);
}

// ----------------------------------------------------------
// Main
// ----------------------------------------------------------

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MixerNode>());
    rclcpp::shutdown();
    return 0;
}
