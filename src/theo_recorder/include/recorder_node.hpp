#include "rclcpp/rclcpp.hpp"
#include "theo_comm/broker_statuses.hpp"

#include "rosbag2_interfaces/srv/resume.hpp"
#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/split_bagfile.hpp"
#include "rosbag2_interfaces/srv/stop.hpp"
#include "theo_msgs/msg/theo_code.hpp"

// command for ros2bag: ros2 bag record --start-paused

class RecorderNode : public rclcpp::Node {
    public: 
        RecorderNode();
        ~RecorderNode();

    private:
        rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedPtr          resumer_client_;
        rclcpp::Client<rosbag2_interfaces::srv::IsPaused>::SharedPtr        check_paused_client_;
        rclcpp::Client<rosbag2_interfaces::srv::Pause>::SharedPtr           pauser_client_;
        rclcpp::Client<rosbag2_interfaces::srv::SplitBagfile>::SharedPtr    splitter_client_;
        rclcpp::Client<rosbag2_interfaces::srv::Stop>::SharedPtr            stopper_client_;
        rclcpp::Subscription<theo_msgs::msg::TheoCode>::SharedPtr           broker_subscription_;

        void send_resume_request_();
        bool check_is_paused_();
        void send_pause_request_();
        void send_split_request_();
        void send_stop_request_();
        void codeCallback( const theo_msgs::msg::TheoCode::SharedPtr msg );
};
