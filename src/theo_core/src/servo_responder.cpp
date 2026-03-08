#include "servo_responder.hpp"


ServoResponderNode::ServoResponderNode()
: Node("servo_responder_node")
{
    this -> declare_parameter<double>( "update_timeout", 0.1 );
    this -> get_parameter( "update_timeout", this -> update_timeout_seconds_ );

    // Setup pub/sub+
    this -> subscription_ = this -> create_subscription<theo_msgs::msg::TheoServoCmd>(
        "servocmd_topic", 
        10, 
        std::bind( 
            &ServoResponderNode::cmdCallback, 
            this, 
            std::placeholders::_1
        )
    );

    this -> publisher_ = this -> create_publisher<theo_msgs::msg::TheoServoAddition>(
        "servoadd_topic", 
        10
    );

    this -> timer_ = this -> create_wall_timer(
        std::chrono::milliseconds(10), 
        std::bind(&ServoResponderNode::timerCallback, this)
    );

    this->last_vel_update_time_ = this->get_clock()->now();
    return;
};


void ServoResponderNode::cmdCallback( const theo_msgs::msg::TheoServoCmd::SharedPtr msg )
{
    ( this->last_vel_cmd_ ).pan_angular_velocity  = msg -> pan_vel;
    ( this->last_vel_cmd_ ).tilt_angular_velocity = msg -> tilt_vel;
    ( this->last_vel_cmd_ ).expired               = false;
    this->last_vel_update_time_                   = this->get_clock()->now();
};



void ServoResponderNode::publish_pantilt_command( double dt ) {
    theo_msgs::msg::TheoServoAddition add_msg_out;
    add_msg_out.pan_angle   = ( this->last_vel_cmd_.pan_angular_velocity )  * dt;
    add_msg_out.tilt_angle  = ( this->last_vel_cmd_.tilt_angular_velocity ) * dt;
    this -> publisher_ -> publish( add_msg_out );
};



void ServoResponderNode::timerCallback(){

    if( ( this->last_vel_cmd_ ).expired  ) return;

    double dt = rclcpp::Duration( ( this->get_clock()->now() ) - ( this->last_vel_update_time_ ) ).nanoseconds() * 1e-9;
    if( dt > ( this->update_timeout_seconds_ )  )   this -> last_vel_cmd_.expired = true;
    else                                            this -> publish_pantilt_command( dt );

};



// -------------------------------------------------------------------------------------------------------------
// Main
// -------------------------------------------------------------------------------------------------------------

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<ServoResponderNode> node = std::make_shared<ServoResponderNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
