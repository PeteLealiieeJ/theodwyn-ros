#include "recorder_node.hpp"


RecorderNode::RecorderNode() 
: Node("recorder_node") 
{
    this -> resumer_client_ = this -> create_client<rosbag2_interfaces::srv::Resume>(
        "/rosbag2_recorder/resume"
    );

    this -> check_paused_client_ = this -> create_client<rosbag2_interfaces::srv::IsPaused>(
        "/rosbag2_recorder/is_paused"
    );

    this -> pauser_client_ = this -> create_client<rosbag2_interfaces::srv::Pause>(
        "/rosbag2_recorder/pause"
    );

    this -> splitter_client_ = this -> create_client<rosbag2_interfaces::srv::SplitBagfile>(
        "/rosbag2_recorder/split_bagfile"
    );

    this -> stopper_client_ = this -> create_client<rosbag2_interfaces::srv::Stop>(
        "/rosbag2_recorder/stop"
    );

    
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.transient_local();
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    this -> broker_subscription_  = this -> create_subscription<theo_msgs::msg::TheoCode>(
        "external/broker_robotic_topic", 
        qos_profile, 
        std::bind( 
            &RecorderNode::codeCallback, 
            this, 
            std::placeholders::_1
        )
    );

};


RecorderNode::~RecorderNode(){
    if( !rclcpp::ok() ) this -> send_stop_request_();
};


void RecorderNode::send_resume_request_(){
    auto request = std::make_shared<rosbag2_interfaces::srv::Resume::Request>();

    while (!this->resumer_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the recorder service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recorder service not available, waiting again...");
    }

    auto result = this->resumer_client_->async_send_request(request);
    if ( rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS )
    {
        RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Recording resumed.");
    } else {
        RCLCPP_ERROR( rclcpp::get_logger("rclcpp"), "Failed to resume recording.");
    }
};


bool RecorderNode::check_is_paused_(){
    auto request = std::make_shared<rosbag2_interfaces::srv::IsPaused::Request>();

    while (!this->check_paused_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the recorder service. Exiting.");
            return false;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recorder service not available, waiting again...");
    }

    auto result = this->check_paused_client_->async_send_request(request);
    if ( rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS )
    {
        RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Recording resumed.");
    } else {
        RCLCPP_ERROR( rclcpp::get_logger("rclcpp"), "Failed to resume recording.");
    }

    return bool( result.get()->paused );
};


void RecorderNode::send_pause_request_(){
    auto request = std::make_shared<rosbag2_interfaces::srv::Pause::Request>();

    while (!this->pauser_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the recorder service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recorder service not available, waiting again...");
    }

    auto result = this->pauser_client_->async_send_request(request);
    if ( rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS )
    {
        RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Recording paused.");
    } else {
        RCLCPP_ERROR( rclcpp::get_logger("rclcpp"), "Failed to pause recording.");
    }
}


void RecorderNode::send_split_request_(){
    auto request = std::make_shared<rosbag2_interfaces::srv::SplitBagfile::Request>();

    while (!this->splitter_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the recorder service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recorder service not available, waiting again...");
    }

    auto result = this->splitter_client_->async_send_request(request);
    if ( rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS )
    {
        RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Recording split.");
    } else {
        RCLCPP_ERROR( rclcpp::get_logger("rclcpp"), "Failed to split recording.");
    }
}


void RecorderNode::send_stop_request_(){
    auto request = std::make_shared<rosbag2_interfaces::srv::Stop::Request>();

    while (!this->stopper_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the recorder service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recorder service not available, waiting again...");
    }

    auto result = this->stopper_client_->async_send_request(request);
    if ( rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS )
    {
        RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Recording stopped.");
    } else {
        RCLCPP_ERROR( rclcpp::get_logger("rclcpp"), "Failed to stop recording.");
    }
}


void RecorderNode::codeCallback(
    const theo_msgs::msg::TheoCode::SharedPtr msg
)
{
    switch (  static_cast<BroadcastBrokerStatus>(msg -> code) )
    {
        case BroadcastBrokerStatus::Broadcasting_Trajectory :
        {
            this -> send_resume_request_();
            break;
        }
        
        default:
        {
            if( !( this -> check_is_paused_() )  ){
                this -> send_pause_request_();
                this -> send_split_request_();
            }
            break;
        }
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node( std::make_shared<RecorderNode>() );
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
