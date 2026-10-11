#ifndef THEODWYNROS_THEORECORDER_RECORDERNODE_
#define THEODWYNROS_THEORECORDER_RECORDERNODE_

#include "rclcpp/rclcpp.hpp"
#include "theo_comm/broker_info.hpp"

#include "rosbag2_interfaces/srv/resume.hpp"
#include "rosbag2_interfaces/srv/is_paused.hpp"
#include "rosbag2_interfaces/srv/pause.hpp"
#include "rosbag2_interfaces/srv/split_bagfile.hpp"
#include "theo_msgs/msg/theo_code.hpp"

// command for ros2bag: ros2 bag record --start-paused
namespace Theodwyn
{
    /**
     * @brief Node that controls a running rosbag2 recorder based on broker status.
     *
     * Recording is resumed when the trajectory broadcast begins; on any other status
     * it pauses the recorder (if recording) and splits the bag file.
     * Expects the recorder to be started paused.
     */
    class RecorderNode : public rclcpp::Node
    {
    public:
        /// @brief Creates the rosbag2 service clients and the broker status subscription.
        RecorderNode();

    private:
        /// Client for the rosbag2 resume service.
        rclcpp::Client<rosbag2_interfaces::srv::Resume>::SharedPtr resumer_client_;
        /// Client for the rosbag2 is_paused service.
        rclcpp::Client<rosbag2_interfaces::srv::IsPaused>::SharedPtr check_paused_client_;
        /// Client for the rosbag2 pause service.
        rclcpp::Client<rosbag2_interfaces::srv::Pause>::SharedPtr pauser_client_;
        /// Client for the rosbag2 split_bagfile service.
        rclcpp::Client<rosbag2_interfaces::srv::SplitBagfile>::SharedPtr splitter_client_;
        /// Subscription to the broker's robotic status topic.
        rclcpp::Subscription<theo_msgs::msg::TheoCode>::SharedPtr broker_subscription_;
        /// Callback group for the subscription (separate from clients so service calls can complete inside the callback).
        rclcpp::CallbackGroup::SharedPtr subscription_cb_group_;
        /// Callback group for the service clients.
        rclcpp::CallbackGroup::SharedPtr client_cb_group_;

        /// @brief Requests that rosbag2 resume recording; waits for the service and up to 3 s for the reply.
        void send_resume_request_();

        /**
         * @brief Queries whether rosbag2 is currently paused.
         * @return True if paused (also returned if interrupted while waiting for the service).
         */
        bool check_is_paused_();

        /// @brief Requests that rosbag2 pause recording.
        void send_pause_request_();

        /// @brief Requests that rosbag2 split the bag file.
        void send_split_request_();

        /**
         * @brief Resumes recording on Broadcasting_Trajectory; otherwise pauses and splits if recording.
         * @param msg Broker status message.
         */
        void codeCallback(const theo_msgs::msg::TheoCode::SharedPtr msg);
    };
}

#endif
