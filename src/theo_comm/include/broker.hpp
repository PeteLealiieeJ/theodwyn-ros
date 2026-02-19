#include <map>
#include "rclcpp/rclcpp.hpp"
#include "broker_statuses.hpp"
#include "broker_codes.hpp"
#include "broker_ids.hpp"
#include "theo_msgs/msg/theo_code.hpp"
#include "theo_srvs/srv/theo_broker_exchange.hpp"

class BrokerNode : public rclcpp::Node {

    public:

        BrokerNode();
        BroadcastBrokerStatus get_broker_status();

    private:

        BroadcastBrokerStatus status;
        bool broker_reset();
        bool broker_transition( BroadcastBrokerId );
        bool broker_transition_from_idle_( BroadcastBrokerId );
        bool broker_transition_from_sar_( BroadcastBrokerId );
        bool broker_transition_from_bc_( BroadcastBrokerId );
        bool broker_transition_from_sac_( BroadcastBrokerId );
        bool broker_transition_from_bt_( BroadcastBrokerId );

        theo_msgs::msg::TheoCode get_broker_message();
        void publish_status_2robotic();
        void publish_status_2transmitter();

        std::map<BroadcastBrokerStatus, BroadcastBrokerStatus> brokerstatus_next;
        std::map<BroadcastBrokerStatus, BroadcastBrokerStatus> brokerstatus_reset;        

        rclcpp::Publisher<theo_msgs::msg::TheoCode>::SharedPtr publisher_2robotic_;
        rclcpp::Publisher<theo_msgs::msg::TheoCode>::SharedPtr publisher_2transmitter_;
        rclcpp::Service<theo_srvs::srv::TheoBrokerExchange>::SharedPtr  broker_service_;

        void broker_exchange(
            const std::shared_ptr<theo_srvs::srv::TheoBrokerExchange::Request>,
            std::shared_ptr<theo_srvs::srv::TheoBrokerExchange::Response>
        );
};