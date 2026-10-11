#ifndef THEODWYNROS_THEOCOMM_BROKER_
#define THEODWYNROS_THEOCOMM_BROKER_

#include <map>
#include "rclcpp/rclcpp.hpp"
#include "broker_info.hpp"
#include "theo_msgs/msg/theo_code.hpp"
#include "theo_srvs/srv/theo_broker_exchange.hpp"

namespace Theodwyn
{

    /**
     * @brief State machine node that mediates the handshake between the trajectory
     *        transmitter and the robot.
     *
     * Clients send requests through the broker exchange service; on each accepted
     * transition the broker publishes its new status to the robotic and/or transmitter topics.
     */
    class BrokerNode : public rclcpp::Node
    {
    public:
        /// @brief Sets up the status maps, publishers, exchange service, and a reset-on-shutdown callback.
        BrokerNode();

        /// @brief Returns the broker's current state.
        BrokerInfo::BroadcastStatus get_broker_status();

    private:
        /// Current state of the broker.
        BrokerInfo::BroadcastStatus status;

        /**
         * @brief Resets the broker to Idle and informs both the robotic and transmitter sides.
         * @return Always true.
         */
        bool broker_reset();

        /**
         * @brief Dispatches a confirmation request to the handler for the current state.
         * @param id Identity of the requesting party.
         * @return True if the transition was approved.
         */
        bool broker_transition(BrokerInfo::BroadcastId);

        /// @brief Idle -> Standby_After_Received. Only approved for the transmitter; notifies the robot.
        bool broker_transition_from_idle_(BrokerInfo::BroadcastId);

        /// @brief Standby_After_Received -> Broadcasting_Configuration. Only approved for the robot; notifies the transmitter.
        bool broker_transition_from_sar_(BrokerInfo::BroadcastId);

        /// @brief Broadcasting_Configuration -> Standby_After_Configuration. Only approved for the robot; notifies the transmitter.
        bool broker_transition_from_bc_(BrokerInfo::BroadcastId);

        /// @brief Standby_After_Configuration -> Broadcasting_Trajectory. Only approved for the transmitter; notifies the robot.
        bool broker_transition_from_sac_(BrokerInfo::BroadcastId);

        /// @brief Broadcasting_Trajectory -> Idle. Only approved for the transmitter; performs a reset.
        bool broker_transition_from_bt_(BrokerInfo::BroadcastId);

        /// @brief Packages the current broker status into a TheoCode message.
        theo_msgs::msg::TheoCode get_broker_message();

        /// @brief Publishes the current status on the robotic topic.
        void publish_status_2robotic();

        /// @brief Publishes the current status on the transmitter topic.
        void publish_status_2transmitter();

        /// Maps each state to the state a confirmed transition leads to.
        std::map<BrokerInfo::BroadcastStatus, BrokerInfo::BroadcastStatus> brokerstatus_next;

        /// Maps each non-idle state to the state reached on reset (Idle).
        std::map<BrokerInfo::BroadcastStatus, BrokerInfo::BroadcastStatus> brokerstatus_reset;

        /// Publisher of broker status to the robot (transient-local, reliable).
        rclcpp::Publisher<theo_msgs::msg::TheoCode>::SharedPtr publisher_2robotic_;

        /// Publisher of broker status to the transmitter (transient-local, reliable).
        rclcpp::Publisher<theo_msgs::msg::TheoCode>::SharedPtr publisher_2transmitter_;

        /// Service through which clients confirm, reset, or check the broker.
        rclcpp::Service<theo_srvs::srv::TheoBrokerExchange>::SharedPtr broker_service_;

        /**
         * @brief Service callback handling Confirm/Reset/Check requests.
         * @param request  Contains the request code and the requester id.
         * @param response Filled with the approval flag and the (possibly updated) broker status.
         */
        void broker_exchange(
            const std::shared_ptr<theo_srvs::srv::TheoBrokerExchange::Request>,
            std::shared_ptr<theo_srvs::srv::TheoBrokerExchange::Response>);
    };

}

#endif
