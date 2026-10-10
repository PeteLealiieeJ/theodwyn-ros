#include "pid_controller.hpp"

using namespace Theodwyn;

// -------------------------------------------------------------------------------------------------------------
// Constructer / Destructor
// -------------------------------------------------------------------------------------------------------------

PIDControllerNode::PIDControllerNode()
    : BrokerClientNode("pid_controller_node", BrokerInfo::BroadcastId::Robotic, "external/broker_exchange", "external/broker_robotic_topic"),
      mode_(ControllerMode::Idle)
{

    // setup ros parameters and file handle
    declare_parameter<double>("p_linear_gain", DEFAULT_p_linear_gain_);
    declare_parameter<double>("i_linear_gain", DEFAULT_i_linear_gain_);
    declare_parameter<double>("d_linear_gain", DEFAULT_d_linear_gain_);
    declare_parameter<double>("p_angular_gain", DEFAULT_p_angular_gain_);
    declare_parameter<double>("i_angular_gain", DEFAULT_i_angular_gain_);
    declare_parameter<double>("d_angular_gain", DEFAULT_d_angular_gain_);
    declare_parameter<double>("p_servo_gain", DEFAULT_p_servo_gain_);
    declare_parameter<double>("i_servo_gain", DEFAULT_i_servo_gain_);
    declare_parameter<double>("d_servo_gain", DEFAULT_d_servo_gain_);
    declare_parameter<double>("proximity_met_range_meters", DEFAULT_proximity_met_range_meters_);
    declare_parameter<double>("proximity_met_direction_degrees", DEFAULT_proximity_met_direction_degrees_);
    declare_parameter<double>("chassis_linear_error_integrator_threshold", DEFAULT_chassis_linear_error_integrator_threshold_);
    declare_parameter<double>("chassis_angular_error_integrator_threshold", DEFAULT_chassis_angular_error_integrator_threshold_);
    declare_parameter<double>("servo_error_integrator_threshold", DEFAULT_servo_error_integrator_threshold_);
    declare_parameter<double>("valid_error_update_dt", DEFAULT_valid_error_update_dt_);
    declare_parameter<std::vector<double>>("rpy_degrees_C0_C", {DEFAULT_rpy_C0_C[0], DEFAULT_rpy_C0_C[1], DEFAULT_rpy_C0_C[2]});
    declare_parameter<std::string>("trajectory_topic", std::string(DEFAULT_trajectory_topic_));
    declare_parameter<std::string>("vicon_chassis_topic", std::string(DEFAULT_viconchassis_topic_));
    declare_parameter<std::string>("vicon_cam_topic", std::string(DEFAULT_viconcam_topic_));
    declare_parameter<std::string>("joy_authority_topic", std::string(DEFAULT_joyauthority_topic_));
    declare_parameter<std::string>("auto_authority_topic", std::string(DEFAULT_autoauthority_topic_));
    declare_parameter<std::string>("vel_cmd_topic", std::string(DEFAULT_velcmd_topic_));
    declare_parameter<std::string>("servo_cmd_topic", std::string(DEFAULT_servocmd_topic_));

    double proximity_met_direction_degrees;
    std::vector<double> rpy_C0_C;
    get_parameter("p_linear_gain", p_linear_gain_);
    get_parameter("i_linear_gain", i_linear_gain_);
    get_parameter("d_linear_gain", d_linear_gain_);
    get_parameter("p_angular_gain", p_angular_gain_);
    get_parameter("i_angular_gain", i_angular_gain_);
    get_parameter("d_angular_gain", d_angular_gain_);
    get_parameter("p_servo_gain", p_servo_gain_);
    get_parameter("i_servo_gain", i_servo_gain_);
    get_parameter("d_servo_gain", d_servo_gain_);
    get_parameter("proximity_met_range_meters", proximity_met_range_meters_);
    get_parameter("proximity_met_direction_degrees", proximity_met_direction_degrees);
    get_parameter("chassis_linear_error_integrator_threshold", chassis_linear_error_integrator_threshold_);
    get_parameter("chassis_angular_error_integrator_threshold", chassis_angular_error_integrator_threshold_);
    get_parameter("servo_error_integrator_threshold", servo_error_integrator_threshold_);
    get_parameter("valid_error_update_dt", valid_error_update_dt_);
    get_parameter("rpy_degrees_C0_C", rpy_C0_C);
    std::string trajectory_topic, vicon_chassis_topic, vicon_cam_topic, joy_authority_topic, auto_authority_topic, vel_cmd_topic, servo_cmd_topic;
    declare_parameter<std::string>("trajectory_topic", trajectory_topic);
    declare_parameter<std::string>("vicon_chassis_topic", vicon_chassis_topic);
    declare_parameter<std::string>("vicon_cam_topic", vicon_cam_topic);
    declare_parameter<std::string>("joy_authority_topic", joy_authority_topic);
    declare_parameter<std::string>("auto_authority_topic", auto_authority_topic);
    declare_parameter<std::string>("vel_cmd_topic", vel_cmd_topic);
    declare_parameter<std::string>("servo_cmd_topic", servo_cmd_topic);

    double deg2rad_ = (3.14159265358979323846 / 180.0);
    proximity_met_direction_radians_ = deg2rad_ * proximity_met_direction_degrees;
    R_C0_C_.setRPY(
        deg2rad_ * rpy_C0_C[0],
        deg2rad_ * rpy_C0_C[1],
        deg2rad_ * rpy_C0_C[2]);

    // setup service/publishers/subscribers

    reference_subscription_ = create_subscription<theo_msgs::msg::TheoWaypoint>(
        trajectory_topic,
        10,
        std::bind(
            &PIDControllerNode::waypointCallback,
            this,
            std::placeholders::_1));

    vicon_chassis_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        vicon_chassis_topic,
        10,
        std::bind(
            &PIDControllerNode::chassisMocapCallback,
            this,
            std::placeholders::_1));

    vicon_servo_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        vicon_cam_topic,
        10,
        std::bind(
            &PIDControllerNode::servoMocapCallback,
            this,
            std::placeholders::_1));

    rclcpp::QoS authority_qos_profile(rclcpp::KeepLast(1));
    authority_qos_profile.transient_local();
    authority_qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    teleop_authority_publisher_ = create_publisher<std_msgs::msg::Bool>(
        joy_authority_topic,
        authority_qos_profile);

    authority_subscription_ = create_subscription<std_msgs::msg::Bool>(
        auto_authority_topic,
        10,
        std::bind(
            &PIDControllerNode::authorityCallback,
            this,
            std::placeholders::_1));

    chassis_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
        vel_cmd_topic,
        10);

    servo_publisher_ = create_publisher<theo_msgs::msg::TheoServoCmd>(
        servo_cmd_topic,
        10);

    last_yaw_ = 0.0;

    return;
}

PIDControllerNode::~PIDControllerNode()
{
    reset_idle(); // return authority to proper place :0
};

// -------------------------------------------------------------------------------------------------------------
// Controller Handling/Broadcast
// -------------------------------------------------------------------------------------------------------------

void PIDControllerNode::change_teleop_authority_to(bool mode)
{
    std_msgs::msg::Bool msg;
    msg.data = mode;
    teleop_authority_publisher_->publish(msg); // return authority to teleop
}

void PIDControllerNode::switch_mode_(ControllerMode mode_in)
{
    reset_signal_errors_();
    switch (mode_in)
    {
    case ControllerMode::Idle:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Controller switching to Idle Mode");
        mode_ = mode_in;
        break;

    case ControllerMode::Responding:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Controller switching to Responding Mode");
        mode_ = mode_in;
        break;

    case ControllerMode::Active:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Controller switching to Active Mode");
        mode_ = mode_in;
        break;

    default:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "User requested switch to unrecognized mode ... Skipping");
        break;
    }
};

void PIDControllerNode::grab_n_expire_waypoint_data(WaypointData &waypoint_in)
{
    waypoint_in = last_waypoint_;
    if (waypoint_in.expired)
        return;
    else
        (last_waypoint_).expired = true;
};

void PIDControllerNode::grab_n_expire_chassis_mocap_data(MocapData &mocap_in)
{
    mocap_in = last_chassis_mocap_;
    if (mocap_in.expired)
        return;
    else
        (last_chassis_mocap_).expired = true;
};

void PIDControllerNode::grab_n_expire_servo_mocap_data(MocapData &mocap_in)
{
    mocap_in = last_servo_mocap_;
    if (mocap_in.expired)
        return;
    else
        (last_servo_mocap_).expired = true;
};

Eigen::Matrix2d PIDControllerNode::get_last_rotm() const
{
    double cos_yaw = std::cos(last_yaw_);
    double sin_yaw = std::sin(last_yaw_);
    return Eigen::Matrix2d{
        {cos_yaw, sin_yaw},
        {-sin_yaw, cos_yaw}};
};

void PIDControllerNode::reset_signal_errors_()
{
    double time_now = get_clock()->now().seconds();
    Error3D chassis_error_reset;
    Error2D servo_error_reset;
    chassis_error_reset.receive_time = time_now;
    servo_error_reset.receive_time = time_now;
    update_chassis_errors_(chassis_error_reset);
    update_servo_errors_(servo_error_reset);
    unwind_accumulated_errors();
};

void PIDControllerNode::update_chassis_errors_(const Error3D &chassis_error_in)
{
    last_chassis_errors_ = chassis_error_in;
};

void PIDControllerNode::update_servo_errors_(const Error2D &servo_error_in)
{
    last_servo_errors_ = servo_error_in;
};

void PIDControllerNode::unwind_accumulated_errors()
{
    chassis_error_iterm_ = Eigen::Vector3d::Zero();
    servo_error_iterm_ = Eigen::Vector2d::Zero();
};

Eigen::Vector3d PIDControllerNode::add_n_grab_chassis_error_i_term_(const Eigen::Vector3d &err_in, double &dt)
{
    chassis_error_iterm_ += (err_in * dt);
    if (chassis_error_iterm_(0) > chassis_linear_error_integrator_threshold_)
    {
        chassis_error_iterm_(0) = chassis_linear_error_integrator_threshold_;
    };
    if (chassis_error_iterm_(1) > chassis_linear_error_integrator_threshold_)
    {
        chassis_error_iterm_(1) = chassis_linear_error_integrator_threshold_;
    };
    if (chassis_error_iterm_(2) > chassis_angular_error_integrator_threshold_)
    {
        chassis_error_iterm_(2) = chassis_angular_error_integrator_threshold_;
    };

    return (chassis_error_iterm_);
};

Eigen::Vector2d PIDControllerNode::add_n_grab_servo_error_i_term_(const Eigen::Vector2d &err_in, double &dt)
{
    servo_error_iterm_ += (err_in * dt);

    if (servo_error_iterm_(0) > servo_error_integrator_threshold_)
    {
        servo_error_iterm_(0) = servo_error_integrator_threshold_;
    };
    if (servo_error_iterm_(1) > servo_error_integrator_threshold_)
    {
        servo_error_iterm_(1) = servo_error_integrator_threshold_;
    };

    return (servo_error_iterm_);
};

PIDControllerNode::Error3D PIDControllerNode::grab_n_update_chassis_errors(const WaypointData &waypoint_in, const MocapData &mocap_in)
{

    Error3D chassis_errors;
    chassis_errors.receive_time = mocap_in.receive_time;

    // Determines lienar error from mocap and waypoint difference
    Eigen::Vector2d linear_p_error = waypoint_in.chassis_position.segment(0, 2) - mocap_in.position.segment(0, 2);
    double angular_p_error = angles::normalize_angle(waypoint_in.chassis_rpy(2) - mocap_in.rpy(2));
    chassis_errors.p_error.segment(0, 2) = linear_p_error;
    chassis_errors.p_error(2) = angular_p_error;

    // Approximate Integral and Derivative terms
    double dt = mocap_in.receive_time - (last_chassis_errors_.receive_time);
    if (dt < (valid_error_update_dt_) && dt > 0)
    {
        chassis_errors.i_error = add_n_grab_chassis_error_i_term_(chassis_errors.p_error, dt);
        chassis_errors.d_error = (chassis_errors.p_error - last_chassis_errors_.p_error) / dt;
    }

    update_chassis_errors_(chassis_errors);

    return chassis_errors;
};

PIDControllerNode::Error2D PIDControllerNode::grab_n_update_servo_errors(const WaypointData &waypoint_in, const PanTiltData &pantilt_in)
{

    Error2D servo_errors;
    servo_errors.receive_time = pantilt_in.receive_time;

    // Determines lienar error from mocap and waypoint difference
    servo_errors.p_error(0) = waypoint_in.servo_pan_tilt_angle(0) - pantilt_in.pan_angle;
    servo_errors.p_error(1) = waypoint_in.servo_pan_tilt_angle(1) - pantilt_in.tilt_angle;

    // Approximate Integral and Derivative terms
    double dt = pantilt_in.receive_time - (last_servo_errors_.receive_time);
    if (dt < (valid_error_update_dt_) && dt > 0)
    {
        servo_errors.i_error = add_n_grab_servo_error_i_term_(servo_errors.p_error, dt);
        servo_errors.d_error = (servo_errors.p_error - last_servo_errors_.p_error) / dt;
    }

    update_servo_errors_(servo_errors);

    return servo_errors;
};

void PIDControllerNode::process_on_active()
{

    // copy waypoint and mocap data
    WaypointData current_waypoint;
    MocapData current_chassis_mocap;
    grab_n_expire_waypoint_data(current_waypoint);
    grab_n_expire_chassis_mocap_data(current_chassis_mocap);
    if (current_waypoint.expired)
        return;

    publish_chassis_control_command(current_waypoint, current_chassis_mocap);
    if (current_waypoint.servo_enabled)
    {
        MocapData current_servo_mocap;
        grab_n_expire_servo_mocap_data(current_servo_mocap);
        PanTiltData current_pantilt_mocap = get_pantilt_from_mocap(current_chassis_mocap, current_servo_mocap);
        publish_servo_control_command(current_waypoint, current_pantilt_mocap);
    }

    return;
};

bool PIDControllerNode::process_on_responding()
{

    // copy waypoint and mocap data
    WaypointData current_waypoint;
    MocapData current_chassis_mocap;
    MocapData current_servo_mocap;
    PanTiltData current_pantilt_mocap;
    grab_n_expire_waypoint_data(current_waypoint);
    grab_n_expire_chassis_mocap_data(current_chassis_mocap);

    if (!current_waypoint.expired)
    {
        if (!current_chassis_mocap.expired)
        {
            publish_chassis_control_command(current_waypoint, current_chassis_mocap);
            if (current_waypoint.servo_enabled)
            {
                MocapData current_servo_mocap;
                grab_n_expire_servo_mocap_data(current_servo_mocap);
                if (!current_servo_mocap.expired)
                {
                    current_pantilt_mocap = get_pantilt_from_mocap(current_chassis_mocap, current_servo_mocap);
                    publish_servo_control_command(current_waypoint, current_pantilt_mocap);
                }
                else
                    return false;
            }
        }
        else
            return false;
    }
    else
        return false;

    return check_configuration(current_waypoint, current_chassis_mocap, current_pantilt_mocap);
};

bool PIDControllerNode::check_configuration(
    const WaypointData &current_waypoint, const MocapData &current_chassis_mocap, const PanTiltData &current_pantilt_mocap) const
{

    if (
        !(
            ((current_waypoint.chassis_position - current_chassis_mocap.position).segment(0, 2)).norm() <
            proximity_met_range_meters_))
        return false;

    if (
        !(
            abs(angles::normalize_angle(current_waypoint.chassis_rpy(2) - current_chassis_mocap.rpy(2))) <
            proximity_met_direction_radians_))
        return false;

    if (current_waypoint.servo_enabled)
    {

        if (
            !(
                abs(angles::normalize_angle(current_waypoint.servo_pan_tilt_angle(0) - current_pantilt_mocap.pan_angle)) <
                proximity_met_direction_radians_))
            return false;

        if (
            !(
                abs(angles::normalize_angle(current_waypoint.servo_pan_tilt_angle(1) - current_pantilt_mocap.tilt_angle)) <
                proximity_met_direction_radians_))
            return false;
    }

    return true;
}

PIDControllerNode::PanTiltData PIDControllerNode::get_pantilt_from_mocap(const MocapData &chassis_mocap, const MocapData &servo_mocap) const
{

    PanTiltData ptout;
    if (chassis_mocap.expired || servo_mocap.expired)
    {
        ptout.expired = true;
        return ptout;
    }

    // Get Rotation Matrices from Mocap System
    tf2::Matrix3x3 R_V_C;
    tf2::Matrix3x3 R_V_B;
    R_V_C.setRPY(servo_mocap.rpy(0), servo_mocap.rpy(1), servo_mocap.rpy(2));
    R_V_B.setRPY(chassis_mocap.rpy(0), chassis_mocap.rpy(1), chassis_mocap.rpy(2));

    // Get Rotation Matrix for Body ( Chassis) -> Camera Reference Point
    tf2::Matrix3x3 R_C0_B = (R_C0_C_)*R_V_C.transpose() * R_V_B;

    // Extract Pan-Tilt Angles
    ptout.pan_angle = std::atan2(R_C0_B[1][0], R_C0_B[0][0]);
    ptout.tilt_angle = std::asin(-R_C0_B[2][0]);

    ptout.receive_time = chassis_mocap.receive_time;
    ptout.expired = false;

    return ptout;
}

void PIDControllerNode::publish_chassis_control_command(const WaypointData &waypoint_in, const MocapData &mocap_in)
{
    // first pull feedforward commands
    Eigen::Vector2d linear_command = waypoint_in.chassis_linear_velocity.segment(0, 2);
    double angular_command = waypoint_in.chassis_angular_velocity(2);

    // pull feedback terms if they are available
    if (!mocap_in.expired)
    {
        last_yaw_ = mocap_in.rpy(2);
        Error3D chassis_errors = grab_n_update_chassis_errors(waypoint_in, mocap_in);
        linear_command += p_linear_gain_ * chassis_errors.p_error.segment(0, 2) + i_linear_gain_ * chassis_errors.i_error.segment(0, 2) + d_linear_gain_ * chassis_errors.d_error.segment(0, 2);

        angular_command += p_angular_gain_ * chassis_errors.p_error(2) + i_angular_gain_ * chassis_errors.i_error(2) + d_angular_gain_ * chassis_errors.d_error(2);
    }

    Eigen::Vector2d linear_command_out = get_last_rotm() * linear_command;

    // package and output
    geometry_msgs::msg::Twist msg_out;
    msg_out.linear.x = linear_command_out(0);
    msg_out.linear.y = linear_command_out(1);
    msg_out.angular.z = angular_command;
    chassis_publisher_->publish(msg_out);

    return;
};

void PIDControllerNode::publish_servo_control_command(const WaypointData &waypoint_in, const PanTiltData &pantilt_in)
{
    // first pull feedforward commands
    Eigen::Vector2d command = waypoint_in.servo_pan_tilt_velocity;

    // pull feedback terms if they are available
    if (!pantilt_in.expired)
    {
        Error2D servo_errors = grab_n_update_servo_errors(waypoint_in, pantilt_in);
        command += p_servo_gain_ * servo_errors.p_error + i_servo_gain_ * servo_errors.i_error + d_servo_gain_ * servo_errors.d_error;
    }

    // package and output
    theo_msgs::msg::TheoServoCmd msg_out;
    msg_out.pan_vel = command(0);
    msg_out.tilt_vel = command(1);
    servo_publisher_->publish(msg_out);

    return;
};

void PIDControllerNode::reset_idle()
{
    switch_mode_(ControllerMode::Idle);
    change_teleop_authority_to(true);
}

void PIDControllerNode::authorityCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bool has_authority = msg->data;
    if (!has_authority)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Autonomous Controller has Lost Authoity. Returning to Idle.");
        reset_idle();
    }
}

void PIDControllerNode::waypointCallback(const theo_msgs::msg::TheoWaypoint::SharedPtr msg_ptr)
{
    WaypointData new_data;

    new_data.receive_time = get_clock()->now().seconds();

    new_data.chassis_position(0) = msg_ptr->chassis_state.pose.position.x;
    new_data.chassis_position(1) = msg_ptr->chassis_state.pose.position.y;
    new_data.chassis_position(2) = msg_ptr->chassis_state.pose.position.z;

    tf2::Quaternion tmp_quat(
        msg_ptr->chassis_state.pose.orientation.x,
        msg_ptr->chassis_state.pose.orientation.y,
        msg_ptr->chassis_state.pose.orientation.z,
        msg_ptr->chassis_state.pose.orientation.w);
    tf2::Matrix3x3 tmp_mat(tmp_quat);
    double roll, pitch, yaw;
    tmp_mat.getRPY(roll, pitch, yaw);
    new_data.chassis_rpy(0) = roll;
    new_data.chassis_rpy(1) = pitch;
    new_data.chassis_rpy(2) = yaw;

    new_data.chassis_linear_velocity(0) = msg_ptr->chassis_state.twist.linear.x;
    new_data.chassis_linear_velocity(1) = msg_ptr->chassis_state.twist.linear.y;
    new_data.chassis_linear_velocity(2) = msg_ptr->chassis_state.twist.linear.z;

    new_data.chassis_angular_velocity(0) = msg_ptr->chassis_state.twist.angular.x;
    new_data.chassis_angular_velocity(1) = msg_ptr->chassis_state.twist.angular.y;
    new_data.chassis_angular_velocity(2) = msg_ptr->chassis_state.twist.angular.z;

    // if the servo is enabled, indicate it in the data
    if (msg_ptr->servo_enabled)
    {
        new_data.servo_enabled = true;
        new_data.servo_pan_tilt_angle(0) = msg_ptr->servo_state.angle.pan_angle;
        new_data.servo_pan_tilt_angle(1) = msg_ptr->servo_state.angle.tilt_angle;
        new_data.servo_pan_tilt_velocity(0) = msg_ptr->servo_state.vel.pan_vel;
        new_data.servo_pan_tilt_velocity(1) = msg_ptr->servo_state.vel.tilt_vel;
    }
    else
        new_data.servo_enabled = false;

    new_data.expired = false;
    last_waypoint_ = new_data;
};

void PIDControllerNode::chassisMocapCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_ptr)
{
    MocapData new_data;

    new_data.receive_time = get_clock()->now().seconds();

    new_data.position(0) = msg_ptr->pose.position.x;
    new_data.position(1) = msg_ptr->pose.position.y;
    new_data.position(2) = msg_ptr->pose.position.z;

    tf2::Quaternion tmp_quat(
        msg_ptr->pose.orientation.x,
        msg_ptr->pose.orientation.y,
        msg_ptr->pose.orientation.z,
        msg_ptr->pose.orientation.w);
    tf2::Matrix3x3 tmp_mat(tmp_quat);
    double roll, pitch, yaw;
    tmp_mat.getRPY(roll, pitch, yaw);
    new_data.rpy(0) = roll;
    new_data.rpy(1) = pitch;
    new_data.rpy(2) = yaw;

    new_data.expired = false;

    last_chassis_mocap_ = new_data;
};

void PIDControllerNode::servoMocapCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg_ptr)
{
    MocapData new_data;

    new_data.receive_time = get_clock()->now().seconds();

    new_data.position(0) = msg_ptr->pose.position.x;
    new_data.position(1) = msg_ptr->pose.position.y;
    new_data.position(2) = msg_ptr->pose.position.z;

    tf2::Quaternion tmp_quat(
        msg_ptr->pose.orientation.x,
        msg_ptr->pose.orientation.y,
        msg_ptr->pose.orientation.z,
        msg_ptr->pose.orientation.w);
    tf2::Matrix3x3 tmp_mat(tmp_quat);
    double roll, pitch, yaw;
    tmp_mat.getRPY(roll, pitch, yaw);
    new_data.rpy(0) = roll;
    new_data.rpy(1) = pitch;
    new_data.rpy(2) = yaw;

    new_data.expired = false;

    last_servo_mocap_ = new_data;
};

// -------------------------------------------------------------------------------------------------------------
// Broker Client Overrides
// -------------------------------------------------------------------------------------------------------------

void PIDControllerNode::codeCallback(
    const theo_msgs::msg::TheoCode::SharedPtr msg)
{
    switch (static_cast<BrokerInfo::BroadcastStatus>(msg->code))
    {
    case BrokerInfo::BroadcastStatus::Standby_After_Received:
    {
        // switch mode and confirm
        // later confirm and send confirmation
        change_teleop_authority_to(false);
        switch_mode_(ControllerMode::Responding);
        send_broker_request_(BrokerInfo::BroadcastCodes::Confirm);
        break;
    }

    case BrokerInfo::BroadcastStatus::Broadcasting_Trajectory:
    {
        // switch mode / no need to confirm
        change_teleop_authority_to(false);
        switch_mode_(ControllerMode::Active);
        break;
    }

    default:
    {
        reset_idle();
        break;
    }
    }
};

void PIDControllerNode::timerCallback()
{

    if (awaiting_broker_response_)
        return;
    // double current_time_seconds = this -> get_clock() -> now().seconds();

    switch (mode_)
    {

    case ControllerMode::Idle:
    {
        break;
    }

    case ControllerMode::Responding:
    {
        if (process_on_responding())
        {
            if (broker_status_ == BrokerInfo::BroadcastStatus::Broadcasting_Configuration)
            {
                send_broker_request_(BrokerInfo::BroadcastCodes::Confirm);
            }
            reset_idle();
        };
        break;
    }

    case ControllerMode::Active:
    {
        process_on_active();
        break;
    }

    default:
        break;
    }
};

// -------------------------------------------------------------------------------------------------------------
// Main
// -------------------------------------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<PIDControllerNode> node = std::make_shared<PIDControllerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
