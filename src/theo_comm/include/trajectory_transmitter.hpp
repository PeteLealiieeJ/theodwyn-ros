#ifndef THEODWYNROS_THEOCOMM_TRAJECTORYTRANSMITTER_
#define THEODWYNROS_THEOCOMM_TRAJECTORYTRANSMITTER_

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <chrono>
#include <thread>
#include <memory>
#include <filesystem>
#include "rclcpp/rclcpp.hpp"
#include "theo_msgs/msg/theo_waypoint.hpp"
#include "broker_client_node.hpp"

namespace Theodwyn
{

    class TrajectoryTransmitterNode : public BrokerClientNode
    {

    public:
        TrajectoryTransmitterNode();

        enum class TransmitterMode : int8_t
        {
            Standby,
            Idle,
            BroadcastConfiguration,
            BroadcastTrajectory,
            SpinDown
        };

        struct TransmitterParsedData
        {
            bool succeeded = false;
            std::vector<double> data;
        };

    private:
        // ROS members
        rclcpp::Publisher<theo_msgs::msg::TheoWaypoint>::SharedPtr publisher_;

        // ROS Callbacks
        void codeCallback(const theo_msgs::msg::TheoCode::SharedPtr) override;
        void timerCallback() override;

        // Functional Parameters
        bool loop_source;
        double delay_time_seconds;

        // Functional Parameter Defaults
        static constexpr bool DEFAULT_loop_source_ = false;
        static constexpr double DEFAULT_delay_time_seconds_ = 0.;
        static constexpr std::string_view DEFAULT_transmission_topic_ = "trajectory_transmission";

        // Functional Members: CSV File Handling M&M
        std::vector<std::string> source_vec;
        std::vector<std::string>::iterator source_vec_iterator;
        std::ifstream file_handle;
        size_t N_time_chassis_fields_;
        size_t N_time_chassis_servo_fields_;
        theo_msgs::msg::TheoWaypoint::SharedPtr initial_msg_ptr;

        // Functional Methods: CSV File Handling M&M
        void collect_source_files_names(std::string);
        void reset_source_iterator();
        void reset_file_handle(std::string);
        bool next_file_handle();
        TransmitterParsedData parse_next_(double);
        void write_parsed2msg(TransmitterParsedData, theo_msgs::msg::TheoWaypoint &);

        // Functional Members: Trajectory Broadcast Parameters
        TransmitterMode mode_;
        double trajectory_broadcast_start_time_seconds;

        // Functional Methods: Trajectory Broadcast Parameters
        void switch_mode_(TransmitterMode);
    };

}

#endif