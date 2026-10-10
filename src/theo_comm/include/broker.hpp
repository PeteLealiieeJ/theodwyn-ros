#ifndef THEODWYNROS_THEOCOMM_BROKER_
#define THEODWYNROS_THEOCOMM_BROKER_

#include <map>
#include "rclcpp/rclcpp.hpp"
#include "broker_info.hpp"
#include "theo_msgs/msg/theo_code.hpp"
#include "theo_srvs/srv/theo_broker_exchange.hpp"

namespace Theodwyn
{

    class BrokerNode : public rclcpp::Node
    {
    public:
        BrokerNode();
        BrokerInfo::BroadcastStatus get_broker_status();

    private:
        BrokerInfo::BroadcastStatus status;
        bool broker_reset();
        bool broker_transition(BrokerInfo::BroadcastId);
        bool broker_transition_from_idle_(BrokerInfo::BroadcastId);
        bool broker_transition_from_sar_(BrokerInfo::BroadcastId);
        bool broker_transition_from_bc_(BrokerInfo::BroadcastId);
        bool broker_transition_from_sac_(BrokerInfo::BroadcastId);
        bool broker_transition_from_bt_(BrokerInfo::BroadcastId);

        theo_msgs::msg::TheoCode get_broker_message();
        void publish_status_2robotic();
        void publish_status_2transmitter();

        std::map<BrokerInfo::BroadcastStatus, BrokerInfo::BroadcastStatus> brokerstatus_next;
        std::map<BrokerInfo::BroadcastStatus, BrokerInfo::BroadcastStatus> brokerstatus_reset;

        rclcpp::Publisher<theo_msgs::msg::TheoCode>::SharedPtr publisher_2robotic_;
        rclcpp::Publisher<theo_msgs::msg::TheoCode>::SharedPtr publisher_2transmitter_;
        rclcpp::Service<theo_srvs::srv::TheoBrokerExchange>::SharedPtr broker_service_;

        void broker_exchange(
            const std::shared_ptr<theo_srvs::srv::TheoBrokerExchange::Request>,
            std::shared_ptr<theo_srvs::srv::TheoBrokerExchange::Response>);
    };

}

#endif