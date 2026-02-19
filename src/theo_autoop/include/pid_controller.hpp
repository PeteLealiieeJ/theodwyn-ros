#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <math.h>
#include <angles/angles.h>
#include "controller_modes.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "theo_msgs/msg/theo_waypoint.hpp"
#include "theo_comm/broker_client_node.hpp"

struct MocapData{
    double receive_time;
    Eigen::Vector3d position;
    Eigen::Vector3d rpy;
    bool expired = true;
};


struct WaypointData{
    double receive_time;
    Eigen::Vector3d position;
    Eigen::Vector3d rpy;
    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;
    bool expired = true;
};


class PIDControllerNode : public BrokerClientNode {

    public:
        PIDControllerNode();
        ~PIDControllerNode();

    private:

        // Controller Handling | // Control Broadcast Parameters
        double p_linear_gain_;
        double i_linear_gain_; // not currently used -- eh
        double d_linear_gain_; // not currently used -- eh
        double p_angular_gain_;
        double i_angular_gain_; // not currently used -- eh
        double d_angular_gain_; // not currently used -- eh
        double proximity_met_range_meters_;
        double proximity_met_direction_radians_;
        double last_yaw_;
        ControllerMode mode_;
        WaypointData last_waypoint_;
        MocapData last_mocap_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_publisher_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr authority_subscription_;
        rclcpp::Subscription<theo_msgs::msg::TheoWaypoint>::SharedPtr reference_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vicon_subscription_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr teleop_authority_publisher_;
        void change_teleop_authority_to( bool );
        void switch_mode_( ControllerMode );
        void grab_n_expire_waypoint_data( WaypointData& );
        void grab_n_expire_mocap_data( MocapData& );
        Eigen::Matrix2d get_last_rotm();
        void process_on_active();
        bool process_on_responding();
        void publish_control_command( WaypointData, MocapData );
        void reset_idle();
        void authorityCallback( const std_msgs::msg::Bool::SharedPtr );
        void waypointCallback( const theo_msgs::msg::TheoWaypoint::SharedPtr );
        void mocapCallback( const geometry_msgs::msg::PoseStamped::SharedPtr );

        // // Broker Communications Overrides
        void codeCallback( const theo_msgs::msg::TheoCode::SharedPtr ) override;
        void timerCallback() override;
};


