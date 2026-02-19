#include "joymapping.hpp"

JoyMapping::JoyMapping()
: 
Node("joymapping_node"),
mode_(TeleopMode::On)
{
    this -> declare_parameter<double>( "chassis_max_linspeed", 1.0 );
    this -> declare_parameter<double>( "chassis_max_angspeed", 1.0 );
    this -> declare_parameter<double>( "pantilt_maxspeed", 1.0 );
    this -> declare_parameter<double>( "settings_update_waitime", 0.5 );

    this -> get_parameter( 
        "chassis_max_linspeed",      
        this -> chassis_max_linspeed
    );
    this -> get_parameter( 
        "chassis_max_angspeed",      
        this -> chassis_max_angspeed 
    );
    this -> get_parameter( 
        "pantilt_maxspeed",      
        this -> pantilt_maxspeed 
    );
    this -> get_parameter( 
        "settings_update_waitime",      
        this -> settings_update_waitime 
    );

    // Settings Update Time
    this -> last_settings_update = this -> get_clock() -> now();
 
    // // Setup pub/sub+
    // Joystick Publisher
    this -> joy_subscription_ = this -> create_subscription<sensor_msgs::msg::Joy>(
        "joy", 
        10, 
        std::bind( 
            &JoyMapping::mappingCallback, 
            this, 
            std::placeholders::_1
        )
    );
    this -> authority_subscription_ = this -> create_subscription<std_msgs::msg::Bool>(
        "joy_authority", 
        10, 
        std::bind( 
            &JoyMapping::authorityCallback, 
            this, 
            std::placeholders::_1
        )
    );
    // Chassis Subscriber
    this -> chassis_publisher_ = this -> create_publisher<geometry_msgs::msg::Twist>(
        "velcmd_topic", 
        10
    );
    // Pantilt Subscriber
    this -> pantilt_publisher_ = this -> create_publisher<theo_msgs::msg::TheoServoState>(
        "ptcmd_topic", 
        10
    );

};


void JoyMapping::mappingCallback( 
    const sensor_msgs::msg::Joy::SharedPtr msg 
){
    if( this -> mode_ == TeleopMode::Off ) return; 
    // -> Joystick doesn't have authority (move lower to enable authority override)

    std::vector<float> axes      = msg -> axes;
    std::vector<int32_t> buttons = msg -> buttons;

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
    
    
    

    geometry_msgs::msg::Twist      chassis_msg_out;
    theo_msgs::msg::TheoServoState pantilt_msg_out;

    // // -> Chassis Movements
    // Left Stick for Chassis Translation
    chassis_msg_out.linear.x  = this -> chassis_max_linspeed *  axes[1];
    chassis_msg_out.linear.y  = this -> chassis_max_linspeed *  axes[0];
    // Bumpers for Angular Velocity
    chassis_msg_out.angular.z = this -> chassis_max_angspeed * double( buttons[6] - buttons[7]  );

    // // // -> Pantilt Movements
    // // Right Stick
    // pantilt_msg_out.tilt_angle = 
    // pantilt_msg_out.pan_angle  =
    

    // // Settings Changes
    // -> Max Chassis Speed Increases
    if ( abs(axes[7]) > 0.1 || abs(axes[6]) > 0.1 ){

        rclcpp::Time time = this -> get_clock() -> now();

        if ( ( time - this->last_settings_update ).seconds() > this -> settings_update_waitime ){

            if ( axes[7]>0.1 ){ // -> Max Chassis Speed Increases
                this -> chassis_max_linspeed *= 1.1;
                this -> chassis_max_angspeed *= 1.1;
            }
            if ( axes[7]<-0.1 ){ // -> Max Chassis Speed Decreases
                this -> chassis_max_linspeed *= 0.9;
                this -> chassis_max_angspeed *= 0.9;
            }
            if ( axes[6]<-0.1 ){ // -> Max Chassis Speed Increases
                this -> pantilt_maxspeed *= 1.1;
                this -> pantilt_maxspeed *= 1.1;
            }
            if ( axes[6]>0.1 ){ // -> Max Chassis Speed Decreases
                this -> pantilt_maxspeed *= 0.9;
                this -> pantilt_maxspeed *= 0.9;
            }
            this->last_settings_update = time;
            
            RCLCPP_INFO(
                this -> get_logger(),
                "CMLS: %.2f, CMAS: %.2f, PTMS: %.2f \r", 
                this -> chassis_max_linspeed, 
                this -> chassis_max_angspeed, 
                this -> pantilt_maxspeed  
            );
        }

    }

    this -> chassis_publisher_ -> publish( chassis_msg_out );
    this -> pantilt_publisher_ -> publish( pantilt_msg_out );

}


void JoyMapping::authorityCallback( const std_msgs::msg::Bool::SharedPtr msg ){
    bool has_authority = msg -> data;
    if( has_authority ) this -> mode_ = TeleopMode::On;
    else this -> mode_ = TeleopMode::Off;
}


int main(int argc, char * argv[])
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<JoyMapping>() );
    rclcpp::shutdown();
    return 0;
}
