#include "joymapping.hpp"

using namespace Theodwyn;

JoyMapping::JoyMapping()
    : Node("joymapping_node"), mode_(TeleopMode::On)
{
    declare_parameter<double>("chassis_max_linspeed", DEFAULT_chassis_max_linspeed_);
    declare_parameter<double>("chassis_max_angspeed", DEFAULT_chassis_max_angspeed_);
    declare_parameter<double>("pantilt_maxspeed", DEFAULT_pantilt_maxspeed_);
    declare_parameter<double>("settings_update_waitime", DEFAULT_settings_update_waitime_);
    declare_parameter<bool>("speed_locked", DEFAULT_speed_locked_);
    declare_parameter<std::string>("joy_topic", std::string(DEFAULT_joy_topic_));
    declare_parameter<std::string>("joy_authority_topic", std::string(DEFAULT_joyauthority_topic_));
    declare_parameter<std::string>("vel_cmd_topic", std::string(DEFAULT_velcmd_topic_));
    declare_parameter<std::string>("servo_cmd_topic", std::string(DEFAULT_servocmd_topic_));

    // Pull down configuration parameters
    get_parameter("chassis_max_linspeed", chassis_max_linspeed_);
    get_parameter("chassis_max_angspeed", chassis_max_angspeed_);
    get_parameter("pantilt_maxspeed", pantilt_maxspeed_);
    get_parameter("settings_update_waitime", settings_update_waitime_);
    get_parameter("speed_locked", speed_locked_);
    std::string joy_topic, joy_authority_topic, vel_cmd_topic, servo_cmd_topic;
    get_parameter("joy_topic", joy_topic);
    get_parameter("joy_authority_topic", joy_authority_topic);
    get_parameter("vel_cmd_topic", vel_cmd_topic);
    get_parameter("servo_cmd_topic", servo_cmd_topic);

    // Settings Update Time
    last_settings_update_ = get_clock()->now();

    // Joystick Publisher
    joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
        joy_topic,
        10,
        std::bind(
            &JoyMapping::mappingCallback,
            this,
            std::placeholders::_1));

    // Authority topic shared with Autonomous functionality
    authority_subscription_ = create_subscription<std_msgs::msg::Bool>(
        joy_authority_topic,
        10,
        std::bind(
            &JoyMapping::authorityCallback,
            this,
            std::placeholders::_1));

    // Chassis Subscriber
    chassis_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        vel_cmd_topic,
        10);

    // Pantilt Subscriber
    pantilt_publisher_ = create_publisher<theo_msgs::msg::TheoServoCmd>(
        servo_cmd_topic,
        10);
};

void JoyMapping::mappingCallback(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (mode_ == TeleopMode::Off)
        return;
    // -> Joystick doesn't have authority (move lower to enable authority override)

    std::vector<float> axes = msg->axes;
    std::vector<int32_t> buttons = msg->buttons;

    // ------------------------------------
    // Assignments
    // ------------------------------------
    // Buttons
    // ------------------------------------
    //  0  -
    //  1  -
    //  2  -
    //  3  -
    //  4  -
    //  5  -
    //  6  -
    //  8  -
    //  9  - Chassis Bwz+
    //  10 - Chassis Bwz-
    //  11 - Chassis Max Speed +
    //  12 - Chassis Max Speed -
    //  13 - Pan-Tilt Max Speed -
    //  14 - Pan-Tilt Max Speed +
    //  15 -
    //  16 -
    //  17 -
    //  18 -
    //  19 -
    //  20 -
    // ------------------------------------
    // Axis
    // ------------------------------------
    //  0  - Chassis Bvx
    //  1  - Chassis Bvy
    //  2  - PT Bvx
    //  3  - PT Bvy
    //  4  -
    //  5  -
    // ------------------------------------

    geometry_msgs::msg::Twist chassis_msg_out;
    theo_msgs::msg::TheoServoCmd pantilt_msg_out;

    // // -> Chassis Movements
    // Left Stick for Chassis Translation
    chassis_msg_out.linear.x = chassis_max_linspeed_ * axes[1];
    chassis_msg_out.linear.y = chassis_max_linspeed_ * axes[0];
    // Bumpers for Angular Velocity
    chassis_msg_out.angular.z = chassis_max_angspeed_ * double(buttons[6] - buttons[7]);

    // // // -> Pantilt Movements
    // // Right Stick
    pantilt_msg_out.pan_vel = pantilt_maxspeed_ * axes[2];
    pantilt_msg_out.tilt_vel = pantilt_maxspeed_ * axes[3];

    // // Settings Changes
    if (!speed_locked_)
    {
        if (abs(axes[7]) > 0.1 || abs(axes[6]) > 0.1)
        {

            rclcpp::Time time = get_clock()->now();

            if ((time - last_settings_update_).seconds() > settings_update_waitime_)
            {

                if (axes[7] > 0.1)
                { // -> Max Chassis Speed Increases
                    chassis_max_linspeed_ *= 1.1;
                    chassis_max_angspeed_ *= 1.1;
                }
                if (axes[7] < -0.1)
                { // -> Max Chassis Speed Decreases
                    chassis_max_linspeed_ *= 0.9;
                    chassis_max_angspeed_ *= 0.9;
                }
                if (axes[6] < -0.1)
                { // -> Max Chassis Speed Increases
                    pantilt_maxspeed_ *= 1.1;
                    pantilt_maxspeed_ *= 1.1;
                }
                if (axes[6] > 0.1)
                { // -> Max Chassis Speed Decreases
                    pantilt_maxspeed_ *= 0.9;
                    pantilt_maxspeed_ *= 0.9;
                }
                last_settings_update_ = time;

                RCLCPP_INFO(
                    get_logger(),
                    "CMLS: %.2f, CMAS: %.2f, PTMS: %.2f \r",
                    chassis_max_linspeed_,
                    chassis_max_angspeed_,
                    pantilt_maxspeed_);
            }
        }
    }

    chassis_publisher_->publish(chassis_msg_out);
    pantilt_publisher_->publish(pantilt_msg_out);
}

void JoyMapping::authorityCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool has_authority = msg->data;
    if (has_authority)
        mode_ = TeleopMode::On;
    else
        mode_ = TeleopMode::Off;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyMapping>());
    rclcpp::shutdown();
    return 0;
}
