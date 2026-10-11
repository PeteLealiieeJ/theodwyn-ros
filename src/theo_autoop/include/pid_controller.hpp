#ifndef THEODWYNROS_THEOAUTOOP_PIDCONTROLLER_
#define THEODWYNROS_THEOAUTOOP_PIDCONTROLLER_

#include <array>
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <math.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <angles/angles.h>
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "theo_msgs/msg/theo_waypoint.hpp"
#include "theo_msgs/msg/theo_servo_cmd.hpp"
#include "theo_comm/broker_client_node.hpp"

namespace Theodwyn
{
    /**
     * @brief Autonomous waypoint-tracking controller for the chassis and pan/tilt servo.
     *
     * Combines waypoint feedforward velocities with PID feedback computed from motion
     * capture poses. Participates in the broker handshake as the robotic party: it first
     * drives to the configuration waypoint (Responding), then tracks the trajectory (Active).
     */
    class PIDControllerNode : public BrokerClientNode
    {
    public:
        /// @brief Declares parameters, creates publishers/subscribers, and sets up the camera alignment rotation.
        PIDControllerNode();

        /// @brief Returns authority to teleop and resets the controller to Idle.
        ~PIDControllerNode();

        /// @brief Operating modes of the controller.
        enum class ControllerMode : int8_t
        {
            Idle,       ///< Doing nothing
            Responding, ///< Responding to some requested pose, generates feedback to brokerage when response completes
            Active      ///< Processing commands on topic
        };

        /// @brief Latest motion capture pose of a tracked body.
        struct MocapData
        {
            double receive_time;      ///< Node clock time at receipt [s]
            Eigen::Vector3d position; ///< Position [m]
            Eigen::Vector3d rpy;      ///< Roll, pitch, yaw [rad]
            bool expired = true;      ///< True if already consumed or never received
        };

        /// @brief Pan/tilt angles derived from motion capture.
        struct PanTiltData
        {
            double receive_time; ///< Time of the mocap data used [s]
            double pan_angle;    ///< Pan angle [rad]
            double tilt_angle;   ///< Tilt angle [rad]
            bool expired = true; ///< True if the angles are not valid
        };

        /// @brief Latest reference waypoint for the chassis and (optionally) servo.
        struct WaypointData
        {
            double receive_time;                         ///< Node clock time at receipt [s]
            Eigen::Vector3d chassis_position;            ///< Desired chassis position [m]
            Eigen::Vector3d chassis_rpy;                 ///< Desired chassis roll, pitch, yaw [rad]
            Eigen::Vector3d chassis_linear_velocity;     ///< Feedforward linear velocity [m/s]
            Eigen::Vector3d chassis_angular_velocity;    ///< Feedforward angular velocity [rad/s]
            Eigen::Vector2d servo_pan_tilt_angle;        ///< Desired pan, tilt angles [rad]
            Eigen::Vector2d servo_pan_tilt_velocity;     ///< Feedforward pan, tilt rates [rad/s]
            bool servo_enabled = false;                  ///< True if the waypoint specifies servo targets
            bool expired = true;                         ///< True if already consumed or never received
        };

        /// @brief Chassis error terms: [x, y, yaw].
        struct Error3D
        {
            double receive_time;                                  ///< Time the error was computed from [s]
            Eigen::Vector3d p_error = Eigen::Vector3d::Zero();    ///< Proportional (current) error
            Eigen::Vector3d i_error = Eigen::Vector3d::Zero();    ///< Accumulated (integral) error
            Eigen::Vector3d d_error = Eigen::Vector3d::Zero();    ///< Error rate of change (derivative)
        };

        /// @brief Servo error terms: [pan, tilt].
        struct Error2D
        {
            double receive_time;                                  ///< Time the error was computed from [s]
            Eigen::Vector2d p_error = Eigen::Vector2d::Zero();    ///< Proportional (current) error
            Eigen::Vector2d i_error = Eigen::Vector2d::Zero();    ///< Accumulated (integral) error
            Eigen::Vector2d d_error = Eigen::Vector2d::Zero();    ///< Error rate of change (derivative)
        };

        /// @brief Scalar error terms.
        struct Error1D
        {
            double p_error = 0; ///< Proportional (current) error
            double i_error = 0; ///< Accumulated (integral) error
            double d_error = 0; ///< Error rate of change (derivative)
        };

    private:
        /// Publisher of chassis velocity commands.
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_publisher_;
        /// Publisher of pan/tilt velocity commands.
        rclcpp::Publisher<theo_msgs::msg::TheoServoCmd>::SharedPtr servo_publisher_;
        /// Subscription to the autonomous authority flag.
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr authority_subscription_;
        /// Subscription to reference waypoints.
        rclcpp::Subscription<theo_msgs::msg::TheoWaypoint>::SharedPtr reference_subscription_;
        /// Subscription to the chassis motion capture pose.
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_chassis_subscription_;
        /// Subscription to the camera (servo) motion capture pose.
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_servo_subscription_;
        /// Publisher of the teleop authority flag (transient-local so late joiners see it).
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr teleop_authority_publisher_;

        /// @brief Returns to Idle and hands authority back to teleop if autonomous authority is lost (flag false).
        void authorityCallback(const std_msgs::msg::Bool::SharedPtr);

        /// @brief Stores the incoming waypoint (converting orientation to RPY) as the latest, unexpired waypoint.
        void waypointCallback(const theo_msgs::msg::TheoWaypoint::SharedPtr);

        /// @brief Stores the incoming chassis pose (converted to position + RPY) as the latest mocap data.
        void chassisMocapCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);

        /// @brief Stores the incoming camera pose (converted to position + RPY) as the latest servo mocap data.
        void servoMocapCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);

        /// @brief Switches mode in response to broker status (Responding on request, Active on trajectory broadcast, Idle otherwise).
        void codeCallback(const theo_msgs::msg::TheoCode::SharedPtr) override;

        /// @brief Periodic step: runs the response or active processing for the current mode.
        void timerCallback() override;

        /// Proportional gain on chassis linear error.
        double p_linear_gain_;
        /// Integral gain on chassis linear error.
        double i_linear_gain_;
        /// Derivative gain on chassis linear error.
        double d_linear_gain_;
        /// Proportional gain on chassis yaw error.
        double p_angular_gain_;
        /// Integral gain on chassis yaw error.
        double i_angular_gain_;
        /// Derivative gain on chassis yaw error.
        double d_angular_gain_;
        /// Proportional gain on servo pan/tilt error.
        double p_servo_gain_;
        /// Integral gain on servo pan/tilt error.
        double i_servo_gain_;
        /// Derivative gain on servo pan/tilt error.
        double d_servo_gain_;
        /// Position error [m] below which a configuration is considered reached.
        double proximity_met_range_meters_;
        /// Angular error [rad] below which a configuration is considered reached.
        double proximity_met_direction_radians_;
        /// Saturation limit of the chassis linear error integral term.
        double chassis_linear_error_integrator_threshold_;
        /// Saturation limit of the chassis angular error integral term.
        double chassis_angular_error_integrator_threshold_;
        /// Saturation limit of the servo error integral term.
        double servo_error_integrator_threshold_;
        /// Maximum time between samples [s] for which integral/derivative terms are updated.
        double valid_error_update_dt_;

        /// Default linear P gain.
        static constexpr double DEFAULT_p_linear_gain_ = 0.;
        /// Default linear I gain.
        static constexpr double DEFAULT_i_linear_gain_ = 0.;
        /// Default linear D gain.
        static constexpr double DEFAULT_d_linear_gain_ = 0.;
        /// Default angular P gain.
        static constexpr double DEFAULT_p_angular_gain_ = 0.;
        /// Default angular I gain.
        static constexpr double DEFAULT_i_angular_gain_ = 0.;
        /// Default angular D gain.
        static constexpr double DEFAULT_d_angular_gain_ = 0.;
        /// Default servo P gain.
        static constexpr double DEFAULT_p_servo_gain_ = 0.;
        /// Default servo I gain.
        static constexpr double DEFAULT_i_servo_gain_ = 0.;
        /// Default servo D gain.
        static constexpr double DEFAULT_d_servo_gain_ = 0.;
        /// Default position tolerance [m].
        static constexpr double DEFAULT_proximity_met_range_meters_ = 0.05;
        /// Default angular tolerance [deg].
        static constexpr double DEFAULT_proximity_met_direction_degrees_ = 5.;
        /// Default chassis linear integrator limit.
        static constexpr double DEFAULT_chassis_linear_error_integrator_threshold_ = 1.;
        /// Default chassis angular integrator limit.
        static constexpr double DEFAULT_chassis_angular_error_integrator_threshold_ = 1.;
        /// Default servo integrator limit.
        static constexpr double DEFAULT_servo_error_integrator_threshold_ = 1.;
        /// Default maximum valid sample interval [s].
        static constexpr double DEFAULT_valid_error_update_dt_ = 0.1;
        /// Default roll/pitch/yaw [deg] aligning the camera reference configuration (C0) with the camera (C).
        static constexpr std::array<double, 3> DEFAULT_rpy_C0_C{0., 0., 0.};
        /// Default topic carrying reference waypoints.
        static constexpr std::string_view DEFAULT_trajectory_topic_ = "external/trajectory_transmission";
        /// Default chassis motion capture topic.
        static constexpr std::string_view DEFAULT_viconchassis_topic_ = "/vicon/eowyn/eowyn";
        /// Default camera motion capture topic.
        static constexpr std::string_view DEFAULT_viconcam_topic_ = "/vicon/basler_cam/basler_cam";
        /// Default teleop authority topic.
        static constexpr std::string_view DEFAULT_joyauthority_topic_ = "joy_authority";
        /// Default autonomous authority topic.
        static constexpr std::string_view DEFAULT_autoauthority_topic_ = "auto_authority";
        /// Default chassis velocity command topic.
        static constexpr std::string_view DEFAULT_velcmd_topic_ = "velcmd_topic";
        /// Default servo command topic.
        static constexpr std::string_view DEFAULT_servocmd_topic_ = "servocmd_topic";

        /// Most recent chassis yaw [rad], used to rotate world-frame commands into the chassis frame.
        double last_yaw_;
        /// Accumulated chassis error integral [x, y, yaw].
        Eigen::Vector3d chassis_error_iterm_;
        /// Accumulated servo error integral [pan, tilt].
        Eigen::Vector2d servo_error_iterm_;
        /// Previous chassis errors, used for the derivative term.
        Error3D last_chassis_errors_;
        /// Previous servo errors, used for the derivative term.
        Error2D last_servo_errors_;
        /// Current controller mode.
        ControllerMode mode_;
        /// Latest waypoint received.
        WaypointData last_waypoint_;
        /// Latest chassis mocap pose received.
        MocapData last_chassis_mocap_;
        /// Latest camera mocap pose received.
        MocapData last_servo_mocap_;
        /// Rotation aligning the camera reference configuration with the chassis.
        tf2::Matrix3x3 R_C0_C_;

        /// @brief Publishes the teleop authority flag (true hands control to teleop).
        void change_teleop_authority_to(bool);

        /// @brief Resets error state and switches to the given mode.
        void switch_mode_(ControllerMode);

        /// @brief Copies the latest waypoint into the argument and marks the stored one expired.
        void grab_n_expire_waypoint_data(WaypointData &);

        /// @brief Copies the latest chassis mocap data into the argument and marks the stored one expired.
        void grab_n_expire_chassis_mocap_data(MocapData &);

        /// @brief Copies the latest servo mocap data into the argument and marks the stored one expired.
        void grab_n_expire_servo_mocap_data(MocapData &);

        /// @brief Returns the 2D rotation matrix (world to chassis frame) for the last known yaw.
        Eigen::Matrix2d get_last_rotm() const;

        /// @brief Zeroes stored errors (timestamped now) and the accumulated integral terms.
        void reset_signal_errors_();

        /// @brief Stores the given chassis errors as the previous errors.
        void update_chassis_errors_(const Error3D &);

        /// @brief Stores the given servo errors as the previous errors.
        void update_servo_errors_(const Error2D &);

        /// @brief Zeroes the accumulated chassis and servo integral terms.
        void unwind_accumulated_errors();

        /**
         * @brief Integrates the chassis error and returns the (upper-saturated) integral term.
         * @param err_in Current error [x, y, yaw].
         * @param dt     Time since the previous sample [s].
         */
        Eigen::Vector3d add_n_grab_chassis_error_i_term_(const Eigen::Vector3d &, double &);

        /**
         * @brief Integrates the servo error and returns the (upper-saturated) integral term.
         * @param err_in Current error [pan, tilt].
         * @param dt     Time since the previous sample [s].
         */
        Eigen::Vector2d add_n_grab_servo_error_i_term_(const Eigen::Vector2d &, double &);

        /**
         * @brief Computes P/I/D chassis errors between a waypoint and mocap pose and stores them.
         *        Integral and derivative terms are only updated if the sample interval is valid.
         */
        Error3D grab_n_update_chassis_errors(const WaypointData &, const MocapData &);

        /**
         * @brief Computes P/I/D servo errors between a waypoint and measured pan/tilt and stores them.
         *        Integral and derivative terms are only updated if the sample interval is valid.
         */
        Error2D grab_n_update_servo_errors(const WaypointData &, const PanTiltData &);

        /// @brief Active-mode step: publishes chassis (and servo, if enabled) commands for the latest waypoint.
        void process_on_active();

        /**
         * @brief Responding-mode step: publishes commands toward the latest waypoint.
         * @return True once the chassis (and servo, if enabled) are within tolerance of the waypoint;
         *         false if data is missing or the configuration is not yet reached.
         */
        bool process_on_responding();

        /// @brief Returns true if the mocap/pan-tilt state is within the proximity tolerances of the waypoint.
        bool check_configuration(const WaypointData &, const MocapData &, const PanTiltData &) const;

        /**
         * @brief Derives pan/tilt angles from the chassis and camera mocap orientations.
         * @param chassis_mocap Chassis pose.
         * @param servo_mocap   Camera pose.
         * @return Pan/tilt data; expired if either input is expired.
         */
        PanTiltData get_pantilt_from_mocap(const MocapData &, const MocapData &) const;

        /**
         * @brief Publishes a chassis Twist: feedforward velocity plus PID feedback (if mocap is valid),
         *        rotated into the chassis frame.
         */
        void publish_chassis_control_command(const WaypointData &, const MocapData &);

        /// @brief Publishes a pan/tilt command: feedforward rate plus PID feedback (if pan/tilt data is valid).
        void publish_servo_control_command(const WaypointData &, const PanTiltData &);

        /// @brief Switches to Idle and returns authority to teleop.
        void reset_idle();
    };

}

#endif
