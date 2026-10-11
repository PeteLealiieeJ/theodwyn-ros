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

    /**
     * @brief Abstract base node for parties that take part in the broker handshake.
     *
     * Handles the broker service client, the status subscription, and a 10 ms timer.
     * Derived nodes implement codeCallback() and timerCallback().
     */
    class BrokerClientNode : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructs the client node.
         * @param node_name   Name of the ROS node.
         * @param id          Identity this node presents to the broker.
         * @param srv_name    Name of the broker exchange service.
         * @param fb_topic    Topic on which the broker publishes status to this node.
         */
        BrokerClientNode(const std::string &, BrokerInfo::BroadcastId, const std::string &, const std::string &);

    protected:
        /// True while a request to the broker is in flight; derived timers should idle while set.
        bool awaiting_broker_response_;

        /// Last broker status received in a service response (Unknown until the first response).
        BrokerInfo::BroadcastStatus broker_status_;

        /**
         * @brief Sends a request to the broker, blocking until the service is available.
         *
         * Sets awaiting_broker_response_ until the response callback completes.
         * @param code Request code to send.
         */
        void send_broker_request_(BrokerInfo::BroadcastCodes);

        /// @brief Called when the broker publishes a new status on this node's feedback topic.
        virtual void codeCallback(const theo_msgs::msg::TheoCode::SharedPtr) = 0;

        /// @brief Called periodically (every 10 ms) by the node's wall timer.
        virtual void timerCallback() = 0;

    private:
        /// Client for the broker exchange service.
        rclcpp::Client<theo_srvs::srv::TheoBrokerExchange>::SharedPtr broker_client_;

        /// Subscription to the broker's status topic for this party.
        rclcpp::Subscription<theo_msgs::msg::TheoCode>::SharedPtr broker_subscription_;

        /// Identity this node presents in requests to the broker.
        BrokerInfo::BroadcastId broker_id_;

        /// Wall timer driving timerCallback().
        rclcpp::TimerBase::SharedPtr timer_;

        /**
         * @brief Handles the broker's response: stores the returned status, logs approval/denial,
         *        and clears awaiting_broker_response_.
         */
        void brokerResponseCallback(rclcpp::Client<theo_srvs::srv::TheoBrokerExchange>::SharedFuture);
    };

}

#endif
