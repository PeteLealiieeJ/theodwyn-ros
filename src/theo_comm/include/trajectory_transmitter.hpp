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

    /**
     * @brief Node that reads trajectory CSV file(s) and broadcasts them as waypoints,
     *        coordinating with the robot through the broker.
     *
     * The source may be a single CSV or a folder of CSVs. Each CSV row is
     * `time, position(3), quaternion wxyz(4), linear vel(3), angular vel(3)` optionally
     * followed by `pan, tilt, pan_vel, tilt_vel`.
     */
    class TrajectoryTransmitterNode : public BrokerClientNode
    {

    public:
        /// @brief Declares parameters, collects and opens the CSV source(s), creates the publisher, and queries the broker.
        TrajectoryTransmitterNode();

        /// @brief Operating modes of the transmitter.
        enum class TransmitterMode : int8_t
        {
            Standby,                ///< Waiting to send a broadcast request to an idle broker
            Idle,                   ///< Request sent; waiting on the broker
            BroadcastConfiguration, ///< Repeatedly publishing the initial waypoint so the robot can reach it
            BroadcastTrajectory,    ///< Publishing the CSV trajectory in time
            SpinDown                ///< Closing files and shutting the node down
        };

        /// @brief Result of parsing a CSV line.
        struct TransmitterParsedData
        {
            bool succeeded = false;  ///< True if a valid row at or after the target time was parsed
            std::vector<double> data; ///< Parsed row values (time first)
        };

    private:
        /// Publisher of waypoint messages.
        rclcpp::Publisher<theo_msgs::msg::TheoWaypoint>::SharedPtr publisher_;

        /// @brief Reacts to broker status changes by switching transmitter mode / confirming transitions.
        void codeCallback(const theo_msgs::msg::TheoCode::SharedPtr) override;

        /// @brief Periodic state-machine step: requests broadcast, publishes waypoints, or spins down.
        void timerCallback() override;

        /// If true, restarts from the first CSV after the last one completes.
        bool loop_source;

        /// Seconds to wait after the trajectory broadcast starts before the first row is due.
        double delay_time_seconds;

        /// Default for loop_source.
        static constexpr bool DEFAULT_loop_source_ = false;
        /// Default for delay_time_seconds.
        static constexpr double DEFAULT_delay_time_seconds_ = 0.;
        /// Default topic that waypoints are published on.
        static constexpr std::string_view DEFAULT_transmission_topic_ = "trajectory_transmission";

        /// Paths of the CSV files to broadcast.
        std::vector<std::string> source_vec;
        /// Iterator to the CSV currently being broadcast.
        std::vector<std::string>::iterator source_vec_iterator;
        /// Handle of the open CSV file.
        std::ifstream file_handle;
        /// Expected column count of a row without servo data.
        size_t N_time_chassis_fields_;
        /// Expected column count of a row with servo data.
        size_t N_time_chassis_servo_fields_;
        /// Initial (zero-velocity) waypoint of the current CSV, used for the configuration broadcast.
        theo_msgs::msg::TheoWaypoint::SharedPtr initial_msg_ptr;

        /**
         * @brief Fills source_vec from a CSV file or a folder of CSVs; shuts down if none are found.
         * @param source Path to a CSV file or directory.
         */
        void collect_source_files_names(std::string);

        /// @brief Rewinds to the first CSV in source_vec and opens it.
        void reset_source_iterator();

        /**
         * @brief Opens a CSV, records its first row as the initial waypoint, then reopens it from the start.
         * @param file_name Path of the CSV to open.
         */
        void reset_file_handle(std::string);

        /**
         * @brief Advances to the next CSV in source_vec.
         * @return True if another file was opened, false if none remain.
         */
        bool next_file_handle();

        /**
         * @brief Reads lines until one is valid and not earlier than the target time.
         * @param target_time_seconds Rows with time before this are skipped.
         * @return Parsed row; succeeded is false if the file ran out of valid rows.
         */
        TransmitterParsedData parse_next_(double);

        /**
         * @brief Copies a parsed CSV row into a waypoint message (including servo fields if present).
         * @param parsed_data Row to convert.
         * @param msg_out     Message to populate.
         */
        void write_parsed2msg(TransmitterParsedData, theo_msgs::msg::TheoWaypoint &);

        /// Current transmitter mode.
        TransmitterMode mode_;
        /// Node clock time (seconds) at which the trajectory broadcast began.
        double trajectory_broadcast_start_time_seconds;

        /// @brief Switches the transmitter mode, logging the change.
        void switch_mode_(TransmitterMode);
    };

}

#endif
