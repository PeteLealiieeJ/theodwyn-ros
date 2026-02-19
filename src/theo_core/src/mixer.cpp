#include "mixer.hpp"

MixerNode::MixerNode()
: Node("mixer_node")
{
    
    // Setup Mixing matrix

    double r_wheel, lx,  ly,  omega_ref;
    bool flipdir_0, flipdir_1, flipdir_2, flipdir_3;

    this -> declare_parameter<double>( "r_wheel", 1 );
    this -> declare_parameter<double>( "lx", 1 );
    this -> declare_parameter<double>( "ly", 1 );
    this -> declare_parameter<double>( "omega_ref", 1 );
    this -> declare_parameter<bool>( "flipdir_0", false );
    this -> declare_parameter<bool>( "flipdir_1", false );
    this -> declare_parameter<bool>( "flipdir_2", false );
    this -> declare_parameter<bool>( "flipdir_3", false );

    this -> get_parameter( "r_wheel", r_wheel );
    this -> get_parameter( "lx", lx );
    this -> get_parameter( "ly", ly );
    this -> get_parameter( "omega_ref", omega_ref );
    this -> get_parameter( "flipdir_0", flipdir_0 );
    this -> get_parameter( "flipdir_1", flipdir_1 );
    this -> get_parameter( "flipdir_2", flipdir_2 );
    this -> get_parameter( "flipdir_3", flipdir_3 );


    const double c       = lx + ly;
    const double alpha_0 = (flipdir_0) ? -1. : 1.;
    const double alpha_1 = (flipdir_1) ? -1. : 1.;
    const double alpha_2 = (flipdir_2) ? -1. : 1.;
    const double alpha_3 = (flipdir_3) ? -1. : 1.;
    Eigen::Matrix<double, 4, 1> _Fvec { alpha_0 , alpha_1 , alpha_2 , alpha_3 };
    Eigen::Matrix<double,4,3> _Mu {
        { 1. , -1., -c },
        { 1. ,  1., -c },
        { 1. ,  1.,  c },
        { 1. , -1.,  c }
    };

    this -> MixMat = ( 1./ ( r_wheel * omega_ref ) ) * _Fvec.asDiagonal() * _Mu; 

    // Setup pub/sub+
    this -> subscription_ = this -> create_subscription<geometry_msgs::msg::Twist>(
        "velcmd_topic", 
        10, 
        std::bind( 
            &MixerNode::mixingCallback, 
            this, 
            std::placeholders::_1
        )
    );

    this -> publisher_ = this -> create_publisher<theo_msgs::msg::TheoMechanumCmd>(
        "thrtlcmd_topic", 
        10
    );
}

void MixerNode::mixingCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg_in
) const 
{
    Eigen::Matrix<double,3,1> twist_in { 
        msg_in -> linear.x,
        msg_in -> linear.y,
        msg_in -> angular.z 
    };
    Eigen::Matrix<double,4,1> throttles_out = this->MixMat * twist_in;

    theo_msgs::msg::TheoMechanumCmd msg_out;
    msg_out.throttle_0 = throttles_out(0);
    msg_out.throttle_1 = throttles_out(1);
    msg_out.throttle_2 = throttles_out(2);
    msg_out.throttle_3 = throttles_out(3);

    this -> publisher_ -> publish( msg_out );
}

// ----------------------------------------------------------
// Main
// ----------------------------------------------------------

int main(int argc, char * argv[])
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<MixerNode>() );
    rclcpp::shutdown();
    return 0;
}
