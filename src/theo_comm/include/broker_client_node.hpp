#ifndef THEODWYNROS_THEOCOMM_BROKERCLIENTNODE_
#define THEODWYNROS_THEOCOMM_BROKERCLIENTNODE_

#include <string>
#include <cstdint>
#include "rclcpp/rclcpp.hpp"
#include "broker_info.hpp"
#include "theo_msgs/msg/theo_code.hpp"
#include "theo_srvs/srv/theo_broker_exchange.hpp"

namespace Theodwyn
{

    class BrokerClientNode : public rclcpp::Node
    {
    public:
        BrokerClientNode(const std::string &, BrokerInfo::BroadcastId, const std::string &, const std::string &);

    protected:
        // Functional Members
        bool awaiting_broker_response_;
        BrokerInfo::BroadcastStatus broker_status_;

        // Functional Methods
        void send_broker_request_(BrokerInfo::BroadcastCodes);

        // ROS Callbacks
        virtual void codeCallback(const theo_msgs::msg::TheoCode::SharedPtr) = 0;
        virtual void timerCallback() = 0;

    private:
        // ROS members
        rclcpp::Client<theo_srvs::srv::TheoBrokerExchange>::SharedPtr broker_client_;
        rclcpp::Subscription<theo_msgs::msg::TheoCode>::SharedPtr broker_subscription_;

        // Functional Members
        BrokerInfo::BroadcastId broker_id_;
        rclcpp::TimerBase::SharedPtr timer_;

        // ROS Callbacks
        void brokerResponseCallback(rclcpp::Client<theo_srvs::srv::TheoBrokerExchange>::SharedFuture);
    };

}

#endif