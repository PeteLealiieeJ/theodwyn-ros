#pragma once

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "broker_ids.hpp"
#include "broker_statuses.hpp"
#include "broker_codes.hpp"
#include "theo_msgs/msg/theo_code.hpp"
#include "theo_srvs/srv/theo_broker_exchange.hpp"


class BrokerClientNode : public rclcpp::Node {

    public:
        BrokerClientNode( std::string, BroadcastBrokerId, std::string, std::string );
        // Broker Communications
        // --> Broker Service
        rclcpp::Client<theo_srvs::srv::TheoBrokerExchange>::SharedPtr broker_client_;
        BroadcastBrokerId broker_id_;
        bool awaiting_broker_response_;
        BroadcastBrokerStatus broker_status_;
        rclcpp::TimerBase::SharedPtr timer_;
        void send_broker_request_( BroadcastBrokerCodes );
        void brokerResponseCallback( rclcpp::Client<theo_srvs::srv::TheoBrokerExchange>::SharedFuture);
        // --> Broker P&S
        rclcpp::Subscription<theo_msgs::msg::TheoCode>::SharedPtr broker_subscription_;
        virtual void codeCallback( const theo_msgs::msg::TheoCode::SharedPtr );
        virtual void timerCallback();

};

// PSA: I didn't feel like messing with the cmake such that this links properly outside of this
// project's build specifically. Thus, consider thos a header only build ... 

BrokerClientNode::BrokerClientNode(
    std::string node_name, BroadcastBrokerId id_in, std::string srv_name_in, std::string fb_topic_in
)
    : Node(node_name)
{
    this -> broker_id_     = id_in;
    this -> broker_status_ = BroadcastBrokerStatus::Unknown;
    this -> broker_client_ = this -> create_client<theo_srvs::srv::TheoBrokerExchange>(srv_name_in);
    
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.transient_local();
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

    this -> broker_subscription_  = this -> create_subscription<theo_msgs::msg::TheoCode>(
        fb_topic_in, 
        qos_profile, 
        std::bind( 
            &BrokerClientNode::codeCallback, 
            this, 
            std::placeholders::_1
        )
    );
    this -> timer_ = this -> create_wall_timer(
        std::chrono::milliseconds(10), 
        std::bind(&BrokerClientNode::timerCallback, this)
    );
    this -> awaiting_broker_response_ = false;
}


void BrokerClientNode::send_broker_request_( 
    BroadcastBrokerCodes code
){
    // Setup brokerage request
    auto request    = std::make_shared<theo_srvs::srv::TheoBrokerExchange::Request>();
    request -> code = static_cast<int>( code );
    request -> id   = static_cast<int>( this -> broker_id_ );

    // Wait for service if it's unavailable
    while( 
        !( this -> broker_client_ -> wait_for_service(std::chrono::seconds(1)) ) 
    ){ 
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return;
        }   
        // Send Info Message
        RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Waiting for Broker Service" );
    }

    // Sending request, awaiting response
    this -> awaiting_broker_response_ = true;
    auto response = this -> broker_client_ -> async_send_request( 
        request, 
        std::bind(
            &BrokerClientNode::brokerResponseCallback, 
            this, 
            std::placeholders::_1
        ) 
    );
    RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Request sent to Broker." );
};


void BrokerClientNode::brokerResponseCallback( 
    rclcpp::Client<theo_srvs::srv::TheoBrokerExchange>::SharedFuture response 
){
    // Wait for the response, if it doesn't come send error
    if( response.wait_for( std::chrono::seconds(3) ) == std::future_status::ready ){
        auto response_ptr = response.get();
        this -> broker_status_ = static_cast<BroadcastBrokerStatus>( response_ptr -> code );
        if( response_ptr -> approval ) RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "APPROVED Response received from Broker." );
        else RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "DENIED Response received from Broker." );
    }
    else{
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Response wasn't received from Broker.");
    };
    this -> awaiting_broker_response_ = false;
}


void BrokerClientNode::codeCallback( const theo_msgs::msg::TheoCode::SharedPtr ){};
        
void BrokerClientNode::timerCallback(){};