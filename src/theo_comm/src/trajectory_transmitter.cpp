#include "trajectory_transmitter.hpp"

using namespace Theodwyn;

// -------------------------------------------------------------------------------------------------------------
// Constructer / Destructor
// -------------------------------------------------------------------------------------------------------------

TrajectoryTransmitterNode::TrajectoryTransmitterNode()
    : BrokerClientNode("trajectory_transmitter_node", BrokerInfo::BroadcastId::Transmitter, "broker_exchange", "broker_transmitter_topic"),
      N_time_chassis_fields_(14),
      N_time_chassis_servo_fields_(14 + 4),
      mode_(TransmitterMode::Standby)
{

    // setup ros parameters and file handle
    declare_parameter<std::string>("source_path", "");
    declare_parameter<bool>("loop_source", DEFAULT_loop_source_);
    declare_parameter<double>("delay_time", DEFAULT_delay_time_seconds_);
    declare_parameter<std::string>("transmission_topic", std::string(DEFAULT_transmission_topic_));

    std::string source, transmission_topic;
    get_parameter("source_path", source);
    get_parameter("delay_time", delay_time_seconds);
    get_parameter("loop_source", loop_source);
    get_parameter("transmission_topic", transmission_topic);

    collect_source_files_names(source);
    reset_source_iterator();

    // setup service/publishers/subscribers
    publisher_ = create_publisher<theo_msgs::msg::TheoWaypoint>(transmission_topic, 10);
    send_broker_request_(BrokerInfo::BroadcastCodes::Check);
};

// -------------------------------------------------------------------------------------------------------------
// CSV File Handling
// -------------------------------------------------------------------------------------------------------------

void TrajectoryTransmitterNode::collect_source_files_names(std::string source)
{
    // shutdown node if destination just doesn't exist
    if (!std::filesystem::exists(source))
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"), "Failed to find specified source: [%s] ... Shutting Down",
            (source).c_str());
        rclcpp::shutdown();
    }

    source_vec.clear();
    if (
        std::filesystem::is_regular_file(source) && std::filesystem::path(source).extension() == ".csv")
    {
        // --> check if source is csv
        // make singleton vector to match downstream functionality
        source_vec.push_back(std::filesystem::path(source).string());
    }
    else if (std::filesystem::is_directory(source))
    {
        // --> check if source is folder
        // iterate through folder, check extension, and add to vector
        for (const std::filesystem::directory_entry &file_i : std::filesystem::directory_iterator(source))
        {
            if (
                file_i.is_regular_file() && file_i.path().extension() == ".csv")
            {
                source_vec.push_back(file_i.path().string());
            }
        }
    }

    if (source_vec.empty())
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"), "Specified source was neither a CSV nor Folder containing CSVs: [%s] ... Shutting Down",
            (source).c_str());
        rclcpp::shutdown();
    };
}

void TrajectoryTransmitterNode::reset_source_iterator()
{
    source_vec_iterator = source_vec.begin();
    reset_file_handle(*(source_vec_iterator));
}

void TrajectoryTransmitterNode::reset_file_handle(std::string file_name)
{
    // Close file handler and open new
    file_handle.close();
    file_handle = std::ifstream(file_name);

    // Record configuration point
    TransmitterParsedData initial_data = parse_next_(0.);
    if (!initial_data.succeeded)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"), "Failed to Parse File: [%s] ... Shutting Down",
            (file_name).c_str());
        rclcpp::shutdown();
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parsing: [%s]", (file_name).c_str());
        initial_msg_ptr = std::make_shared<theo_msgs::msg::TheoWaypoint>();
        initial_msg_ptr->chassis_state.pose.position.x = initial_data.data[1];
        initial_msg_ptr->chassis_state.pose.position.y = initial_data.data[2];
        initial_msg_ptr->chassis_state.pose.position.z = initial_data.data[3];
        initial_msg_ptr->chassis_state.pose.orientation.w = initial_data.data[4];
        initial_msg_ptr->chassis_state.pose.orientation.x = initial_data.data[5];
        initial_msg_ptr->chassis_state.pose.orientation.y = initial_data.data[6];
        initial_msg_ptr->chassis_state.pose.orientation.z = initial_data.data[7];
        initial_msg_ptr->chassis_state.twist.linear.x = 0.;
        initial_msg_ptr->chassis_state.twist.linear.y = 0.;
        initial_msg_ptr->chassis_state.twist.linear.z = 0.;
        initial_msg_ptr->chassis_state.twist.angular.x = 0.;
        initial_msg_ptr->chassis_state.twist.angular.y = 0.;
        initial_msg_ptr->chassis_state.twist.angular.z = 0.;
        if (
            initial_data.data.size() == ((N_time_chassis_servo_fields_)))
        {
            initial_msg_ptr->servo_state.angle.pan_angle = initial_data.data[14];
            initial_msg_ptr->servo_state.angle.tilt_angle = initial_data.data[15];
            initial_msg_ptr->servo_state.vel.pan_vel = 0.;
            initial_msg_ptr->servo_state.vel.tilt_vel = 0.;
            initial_msg_ptr->servo_enabled = true;
        }
        else
            initial_msg_ptr->servo_enabled = false;
    }

    // close file handler again and open new
    // TODO: I dont like that this happens twice but it will be addressed in a future pr
    file_handle.close();
    file_handle = std::ifstream(file_name);
}

bool TrajectoryTransmitterNode::next_file_handle()
{
    ++(source_vec_iterator);
    if (source_vec_iterator != source_vec.end())
    {
        reset_file_handle(*(source_vec_iterator));
        return true;
    }
    return false;
}

TrajectoryTransmitterNode::TransmitterParsedData TrajectoryTransmitterNode::parse_next_(double target_time_seconds)
{

    TransmitterParsedData parsed_data_out; // default succeeded = false
    std::string line_out;

    // continue loop until data is successfully received or data runs out
    while (
        (!parsed_data_out.succeeded) &&
        (std::getline(file_handle, line_out)))
    {

        boost::tokenizer<boost::escaped_list_separator<char>> token(line_out);
        std::vector<double> data_vec_at_iter;
        parsed_data_out.succeeded = true; // will switch back to false if timing is in the past or errors are detected in csv

        bool checked_time = false;
        for (
            boost::tokenizer<boost::escaped_list_separator<char>>::iterator iter = token.begin();
            iter != token.end();
            ++iter)
        {

            try
            {
                // append data from csv line (casted to double) to data_out vector
                double data_at_iter = boost::lexical_cast<double>(*iter);

                // check that time ( @data_at_iter[0] ) is after target time, if it is proceed else contine to next line
                if (!checked_time)
                {
                    if (data_at_iter < target_time_seconds)
                    {
                        parsed_data_out.succeeded = false;
                        break;
                    }
                    else
                    {
                        checked_time = true;
                    }
                }

                // add data to output vector
                data_vec_at_iter.push_back(data_at_iter);
            }
            catch (const boost::bad_lexical_cast &error)
            {
                // continue to the next line if the data is unexpected
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Issue encountered parsing CSV ... Skipping line");
                parsed_data_out.succeeded = false;
                break;
            }
        }

        // if all data received successfully, copy data to previous scope and publish
        if (parsed_data_out.succeeded)
        {
            if (
                data_vec_at_iter.size() == (N_time_chassis_fields_) ||
                data_vec_at_iter.size() == (N_time_chassis_servo_fields_))
            {
                parsed_data_out.data = data_vec_at_iter;
            }
            else
                parsed_data_out.succeeded = false;
        }
    }

    return parsed_data_out;
};

void TrajectoryTransmitterNode::write_parsed2msg(
    TransmitterParsedData parsed_data,
    theo_msgs::msg::TheoWaypoint &msg_out)
{
    msg_out.chassis_state.pose.position.x = parsed_data.data[1];
    msg_out.chassis_state.pose.position.y = parsed_data.data[2];
    msg_out.chassis_state.pose.position.z = parsed_data.data[3];
    msg_out.chassis_state.pose.orientation.w = parsed_data.data[4];
    msg_out.chassis_state.pose.orientation.x = parsed_data.data[5];
    msg_out.chassis_state.pose.orientation.y = parsed_data.data[6];
    msg_out.chassis_state.pose.orientation.z = parsed_data.data[7];
    msg_out.chassis_state.twist.linear.x = parsed_data.data[8];
    msg_out.chassis_state.twist.linear.y = parsed_data.data[9];
    msg_out.chassis_state.twist.linear.z = parsed_data.data[10];
    msg_out.chassis_state.twist.angular.x = parsed_data.data[11];
    msg_out.chassis_state.twist.angular.y = parsed_data.data[12];
    msg_out.chassis_state.twist.angular.z = parsed_data.data[13];
    if (
        parsed_data.data.size() == (N_time_chassis_servo_fields_))
    {
        msg_out.servo_state.angle.pan_angle = parsed_data.data[14];
        msg_out.servo_state.angle.tilt_angle = parsed_data.data[15];
        msg_out.servo_state.vel.pan_vel = parsed_data.data[16];
        msg_out.servo_state.vel.tilt_vel = parsed_data.data[17];
        msg_out.servo_enabled = true;
    }
    else
        msg_out.servo_enabled = false;
};

// -------------------------------------------------------------------------------------------------------------
// Trajectory Broadcast
// -------------------------------------------------------------------------------------------------------------

void TrajectoryTransmitterNode::switch_mode_(TransmitterMode mode_in)
{
    switch (mode_in)
    {
    case TransmitterMode::Standby:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transmitter switching to Standby Mode");
        mode_ = mode_in;
        break;
    case TransmitterMode::Idle:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transmitter switching to Idle Mode");
        mode_ = mode_in;
        break;
    case TransmitterMode::BroadcastConfiguration:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transmitter switching to Configuration Broadcast Mode");
        mode_ = mode_in;
        break;
    case TransmitterMode::BroadcastTrajectory:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transmitter switching to Trajectory Broadcast Mode");
        mode_ = mode_in;
        break;
    case TransmitterMode::SpinDown:
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Transmitter switching to Spin Down Mode");
        mode_ = mode_in;
        break;
    default:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "User requested switch to unrecognized mode ... Skipping");
        break;
    }
};

// -------------------------------------------------------------------------------------------------------------
// Broker Client Overrides
// -------------------------------------------------------------------------------------------------------------

void TrajectoryTransmitterNode::codeCallback(
    const theo_msgs::msg::TheoCode::SharedPtr msg)
{
    switch (static_cast<BrokerInfo::BroadcastStatus>(msg->code))
    {
    case BrokerInfo::BroadcastStatus::Idle:
    {
        break;
    }

    case BrokerInfo::BroadcastStatus::Broadcasting_Configuration:
    {
        // --> start configuration broadcast
        switch_mode_(TransmitterMode::BroadcastConfiguration);
        break;
    }

    case BrokerInfo::BroadcastStatus::Standby_After_Configuration:
    {
        // --> stop configuration broadcast
        // --> start trajectory broadcast (after delay)
        // --> send confirmation that broadcast starts
        switch_mode_(TransmitterMode::BroadcastTrajectory);
        trajectory_broadcast_start_time_seconds = get_clock()->now().seconds();
        send_broker_request_(BrokerInfo::BroadcastCodes::Confirm);
        break;
    }

    default:
    {
        // --> stop all broadcast / spindown node
        switch_mode_(TransmitterMode::SpinDown);
        break;
    }
    }
};

void TrajectoryTransmitterNode::timerCallback()
{

    if (awaiting_broker_response_)
    {
        return;
    };
    double current_time_seconds = get_clock()->now().seconds();

    switch (mode_)
    {
    case TransmitterMode::Standby:
    {
        if (broker_status_ == BrokerInfo::BroadcastStatus::Idle)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Broker is Idle. Sending broadcast request.");
            send_broker_request_(BrokerInfo::BroadcastCodes::Confirm);
            switch_mode_(TransmitterMode::Idle);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Broker is Busy. Try Again Later :( ... Shutting Down");
            switch_mode_(TransmitterMode::SpinDown);
        }

        break;
    }

    case TransmitterMode::Idle:
    {
        break;
    }

    case TransmitterMode::BroadcastConfiguration:
    {
        theo_msgs::msg::TheoWaypoint waypoint_msg = *initial_msg_ptr;
        waypoint_msg.header.stamp.sec = current_time_seconds;
        publisher_->publish(waypoint_msg);
        break;
    }

    case TransmitterMode::BroadcastTrajectory:
    {
        double target_time_seconds =
            current_time_seconds - trajectory_broadcast_start_time_seconds - delay_time_seconds;

        if (target_time_seconds < 0.)
        {
            // the initial waypoint with zero velocity should be published while waiting for the next
            theo_msgs::msg::TheoWaypoint waypoint_msg = *initial_msg_ptr;
            waypoint_msg.header.stamp.sec = current_time_seconds;
            publisher_->publish(waypoint_msg);
            break;
        }

        TransmitterParsedData data_next = parse_next_(target_time_seconds);

        if (!data_next.succeeded)
        {
            if (next_file_handle())
            {
                // prepare to broadcast next csv
                switch_mode_(TransmitterMode::Standby);
            }
            else if (loop_source)
            {
                reset_source_iterator();
                switch_mode_(TransmitterMode::Standby);
            }
            else
            {
                // no csv's left spin down
                switch_mode_(TransmitterMode::SpinDown);
            }
            send_broker_request_(BrokerInfo::BroadcastCodes::Reset);
            break;
        }

        theo_msgs::msg::TheoWaypoint waypoint_msg;
        write_parsed2msg(data_next, waypoint_msg);
        double waypoint_time_seconds =
            delay_time_seconds + trajectory_broadcast_start_time_seconds + data_next.data[0];
        waypoint_msg.header.stamp.sec = waypoint_time_seconds;
        // wait until waypoint time, if its still in the future
        double wait_time = waypoint_time_seconds - (get_clock()->now().seconds());
        if (wait_time > 0)
        {
            rclcpp::sleep_for(
                std::chrono::round<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(wait_time)));
        };
        publisher_->publish(waypoint_msg);
        break;
    }

    case TransmitterMode::SpinDown:
    {
        file_handle.close();
        rclcpp::shutdown();
        break;
    }

    default:
        break;
    }
};

// -------------------------------------------------------------------------------------------------------------
// Main
// -------------------------------------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TrajectoryTransmitterNode> node = std::make_shared<TrajectoryTransmitterNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}