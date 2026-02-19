#include "broker.hpp"

BrokerNode::BrokerNode()
    : 
    Node("broker_node"),
    status( BroadcastBrokerStatus::Idle ),
    brokerstatus_next(
        {
            {
                BroadcastBrokerStatus::Idle, 
                BroadcastBrokerStatus::Standby_After_Received
            },
            {
                BroadcastBrokerStatus::Standby_After_Received, 
                BroadcastBrokerStatus::Broadcasting_Configuration
            },
            {
                BroadcastBrokerStatus::Broadcasting_Configuration, 
                BroadcastBrokerStatus::Standby_After_Configuration
            },
            {
                BroadcastBrokerStatus::Standby_After_Configuration, 
                BroadcastBrokerStatus::Broadcasting_Trajectory
            },
            {
                BroadcastBrokerStatus::Broadcasting_Trajectory, 
                BroadcastBrokerStatus::Idle
            },
        }
    ),
    brokerstatus_reset(
        {
            {
                BroadcastBrokerStatus::Standby_After_Received, 
                BroadcastBrokerStatus::Idle
            },
            {
                BroadcastBrokerStatus::Broadcasting_Configuration, 
                BroadcastBrokerStatus::Idle
            },
            {
                BroadcastBrokerStatus::Standby_After_Configuration, 
                BroadcastBrokerStatus::Idle
            },
            {
                BroadcastBrokerStatus::Broadcasting_Trajectory, 
                BroadcastBrokerStatus::Idle
            },
            {
                BroadcastBrokerStatus::Broadcasting_Trajectory, 
                BroadcastBrokerStatus::Idle
            }
        }
    )
{

    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.transient_local();
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    // broker_to_robot Message (TheoCode)
    // -------------------------------
    // code |
    //      0  |-> configuration broadcast request received, send approval or denial 
    //      1  |-> trajectory broadcast request received, send approval or denial
    //      2  |-> configuration/trajectory broadcast concluded, reset (this determines whether the above is denied or approved)
    this -> publisher_2robotic_ = this -> create_publisher<theo_msgs::msg::TheoCode>(
        "broker_robotic_topic", 
        qos_profile
    );


    // broker_to_transmitter Message (TheoCode)
    // -------------------------------
    // code |
    //      0 |-> robot confirmed request, start configuration broadcast
    //      1 |-> robot confirmed configuration, stop configuration broadcast and approve transition
    //      2 |-> configuration/trajectory broadcast concluded, reset (this determines whether the above is denied or approved)
    this -> publisher_2transmitter_ = this -> create_publisher<theo_msgs::msg::TheoCode>(
        "broker_transmitter_topic", 
        qos_profile
    );


    // Service Message (TheoCode|TheoCode)
    // code |
    //      0 |-> confirm to next state
    //      1 |-> reset
    //      2 |-> check broker state
    // -------------------------------
    // response |-> broker status
    this -> broker_service_ = this -> create_service<theo_srvs::srv::TheoBrokerExchange>(
            "broker_exchange", 
            std::bind( 
                &BrokerNode::broker_exchange, 
                this, 
                std::placeholders::_1,
                std::placeholders::_2
            )
        );


    this->get_node_base_interface()->get_context()->add_pre_shutdown_callback(
        std::bind(&BrokerNode::broker_reset, this)
    );
};


BroadcastBrokerStatus BrokerNode::get_broker_status(){
    return this -> status;
};


void BrokerNode::broker_exchange(
    const std::shared_ptr<theo_srvs::srv::TheoBrokerExchange::Request> request,
    std::shared_ptr<theo_srvs::srv::TheoBrokerExchange::Response>      response
){
    bool approval       = false;
    switch( static_cast<BroadcastBrokerCodes>( request -> code ) )
    {
        case BroadcastBrokerCodes::Confirm :
        {
            RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Broker received transition confirmation code from [ID:%ld]", request -> id );
            approval = this -> broker_transition( static_cast<BroadcastBrokerId>(request -> id) ); // = this->brokerstatus_next[ this->status ]
            break;
        }   
        case BroadcastBrokerCodes::Reset :
        {
            RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Broker received reset request code from [ID:%ld]", request -> id );
            approval = this -> broker_reset(); // = this->brokerstatus_reset[ this->status ]
            break;
        }
        case BroadcastBrokerCodes::Check :
        {
            RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Broker received status check request code from [ID:%ld]", request -> id );
            approval = true;
            break;
        }
        default:
            break;
    }
    // Send approve and (maybe updated) status to requested party
    response -> approval    = approval;
    response -> code        = static_cast<int>( this -> get_broker_status() );
};


theo_msgs::msg::TheoCode BrokerNode::get_broker_message(){
    theo_msgs::msg::TheoCode msg_out;
    msg_out.code = static_cast<int>( this -> get_broker_status() );
    return msg_out;
};


void BrokerNode::publish_status_2robotic(){
    this -> publisher_2robotic_ -> publish( this -> get_broker_message() );
};


void BrokerNode::publish_status_2transmitter(){
    this -> publisher_2transmitter_ -> publish( this -> get_broker_message() );
};


bool BrokerNode::broker_reset(){
    // Change Status and Inform Sinks
    this -> status = this -> brokerstatus_reset[ this -> status ];
    RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Broker resetting to Idle" );
    this -> publish_status_2robotic();
    this -> publish_status_2transmitter();
    return true;
};


bool BrokerNode::broker_transition_from_idle_( BroadcastBrokerId id ){
    // (1) receive broadcast request from transmitter
    //      -> (1a) receive request from transmitter, respond with confirmation or denial (return to idle)
    //      -> (1b) publish state to broker_to_robot topic
    //      -> (1c) switch to [Standby_After_Received]
    if( id != BroadcastBrokerId::Transmitter ) return false;
    this -> status = this -> brokerstatus_next[ this -> status ];
    RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Broker transitioned to Standby After Request" );
    this -> publish_status_2robotic();
    return true;
};


bool BrokerNode::broker_transition_from_sar_( BroadcastBrokerId id ){
    // (2) receive broadcast confirmation request from robot
    //      -> (1a) receive request from robot, respond with confirmation or denial (return to idle)
    //      -> (1b) publish state to broker_to_transmitter topic
    //      -> (1c) switch to [Broadcasting_Configuration]
    if( id != BroadcastBrokerId::Robotic ) return false;
    this -> status = this -> brokerstatus_next[ this -> status ];
    RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Broker transitioned to Broadcasting Configuration" );
    this -> publish_status_2transmitter();
    return true;
};


bool BrokerNode::broker_transition_from_bc_( BroadcastBrokerId id ){
    // (3) receive configuration confirmation request from robot
    //      -> (1a) receive request from robot, respond with confirmation or denial (return to idle)
    //      -> (1b) publish state to broker_to_transmitter topic
    //      -> (1c) switch to [Standby_After_Configuration]
    if( id != BroadcastBrokerId::Robotic ) return false;
    this -> status = this -> brokerstatus_next[ this -> status ];
    RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Broker transitioned to Standby After Configuration" );
    this -> publish_status_2transmitter();
    return true;
};


bool BrokerNode::broker_transition_from_sac_( BroadcastBrokerId id ){
    // (4) receive broadcast confirmation request from transmitter
    //      -> (1a) receive request from transmitter, respond with confirmation or denial (return to idle)
    //      -> (1b) publish state to broker_to_robot topic
    //      -> (1c) switch to [Broadcasting_Trajectory]
    if( id != BroadcastBrokerId::Transmitter ) return false;
    this -> status = this -> brokerstatus_next[ this -> status ];
    RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Broker transitioned to Broadcasting Trajectory" );
    this -> publish_status_2robotic();
    return true;
};


bool BrokerNode::broker_transition_from_bt_( BroadcastBrokerId id ){
    // (5) receive broadcast error/halt request from transmitter/robot
    //      -> (1a) receive request from transmitter/robot, respond with confirmation or denial (return to idle)
    //      -> (1b) publish state to broker_to_transmitter/broker_to_robot topic
    //      -> (1c) switch to [Reset_After_Received]
    if( id != BroadcastBrokerId::Transmitter ) return false;
    return this -> broker_reset();
};


bool BrokerNode::broker_transition( BroadcastBrokerId id ){

    bool approval = false;

    switch( this -> status )
    {
        case BroadcastBrokerStatus::Idle :
            approval = this -> broker_transition_from_idle_( id );
            break;

        case BroadcastBrokerStatus::Standby_After_Received :
            approval = this -> broker_transition_from_sar_( id );
            break;

        case BroadcastBrokerStatus::Broadcasting_Configuration :
            approval = this -> broker_transition_from_bc_( id );
            break;

        case BroadcastBrokerStatus::Standby_After_Configuration :
            approval = this -> broker_transition_from_sac_( id );
            break;

        case BroadcastBrokerStatus::Broadcasting_Trajectory :
            approval = this -> broker_transition_from_bt_( id );
            break;
        
        default:
            break;
    }


    return approval;

};


// ----------------------------------------------------------
// Main
// ----------------------------------------------------------

int main(int argc, char * argv[])
{
    rclcpp::init( argc, argv );
    rclcpp::spin( std::make_shared<BrokerNode>() );
    rclcpp::shutdown();
    return 0;
};
