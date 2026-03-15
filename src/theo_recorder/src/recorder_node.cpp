#include "recorder_node.hpp"


RecorderNode::RecorderNode() 
: Node("recorder_node") 
{

    this -> subscription_cb_group_ 
        = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this -> client_cb_group_  
        = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    this -> resumer_client_ = this -> create_client<rosbag2_interfaces::srv::Resume>(
        "/rosbag2_recorder/resume",
        rclcpp::SystemDefaultsQoS(),
        this -> client_cb_group_  
    );

    this -> check_paused_client_ = this -> create_client<rosbag2_interfaces::srv::IsPaused>(
        "/rosbag2_recorder/is_paused",
        rclcpp::SystemDefaultsQoS(),
        this -> client_cb_group_  
    );

    this -> pauser_client_ = this -> create_client<rosbag2_interfaces::srv::Pause>(
        "/rosbag2_recorder/pause",
        rclcpp::SystemDefaultsQoS(),
        this -> client_cb_group_  
    );

    this -> splitter_client_ = this -> create_client<rosbag2_interfaces::srv::SplitBagfile>(
        "/rosbag2_recorder/split_bagfile",
        rclcpp::SystemDefaultsQoS(),
        this -> client_cb_group_  
    );
    
    rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
    qos_profile.transient_local();
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = this -> subscription_cb_group_;
    this -> broker_subscription_  = this -> create_subscription<theo_msgs::msg::TheoCode>(
        "external/broker_robotic_topic", 
        qos_profile, 
        std::bind( 
            &RecorderNode::codeCallback, 
            this, 
            std::placeholders::_1
        ),
        sub_opts
    );

};


void RecorderNode::send_resume_request_(){
    auto request = std::make_shared<rosbag2_interfaces::srv::Resume::Request>();

    while(
        !(this->resumer_client_->wait_for_service(std::chrono::seconds(1)))
    ){
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the rosbag2 recorder service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rosbag2 recording service not available, waiting again...");
    }

    auto result = this->resumer_client_->async_send_request(request);
    if( result.wait_for( std::chrono::seconds(3) ) == std::future_status::ready )
    {
        RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Services resumed rosbag2 recording");
    } else {
        RCLCPP_ERROR( rclcpp::get_logger("rclcpp"), "Services FAILED resume rosbag2 recording");
    }
};


bool RecorderNode::check_is_paused_(){
    auto request = std::make_shared<rosbag2_interfaces::srv::IsPaused::Request>();

    while(
        !(this->check_paused_client_->wait_for_service(std::chrono::seconds(1)))
    ){
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the rosbag2 pause checking service. Exiting.");
            return true;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rosbag2 service for checking paused state not available, waiting again...");
    }

    auto result = this->check_paused_client_->async_send_request(request);
    if( result.wait_for( std::chrono::seconds(3) ) == std::future_status::ready )
    {
        RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Services checked whether the rosbag2 was recording");
    } else {
        RCLCPP_ERROR( rclcpp::get_logger("rclcpp"), "Services FAILED to check if rosbag2 recording");
    }

    return bool( result.get()->paused );
};


void RecorderNode::send_pause_request_(){
    auto request = std::make_shared<rosbag2_interfaces::srv::Pause::Request>();

    while(
        !(this->pauser_client_->wait_for_service(std::chrono::seconds(1)))
    ){
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the rosbag2 pausing service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rosbag2 pausing service not available, waiting again...");
    }

    auto result = this->pauser_client_->async_send_request(request);
    if( result.wait_for( std::chrono::seconds(3) ) == std::future_status::ready )
    {
        RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Services paused rosbag2 recording");
    } else {
        RCLCPP_ERROR( rclcpp::get_logger("rclcpp"), "Services FAILED to pause rosbag2 recording");
    }
}


void RecorderNode::send_split_request_(){
    auto request = std::make_shared<rosbag2_interfaces::srv::SplitBagfile::Request>();
    while(
        !(this->splitter_client_->wait_for_service(std::chrono::seconds(1)))
    ){
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the rosbag2 splitting service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rosbag2 splitting service not available, waiting again...");
    }

    auto result = this->splitter_client_->async_send_request(request);
    if( result.wait_for( std::chrono::seconds(3) ) == std::future_status::ready )
    {
        RCLCPP_INFO( rclcpp::get_logger("rclcpp"), "Services split rosbag2 recording");
    } else {
        RCLCPP_ERROR( rclcpp::get_logger("rclcpp"), "Services FAILED to split rosbag2 recording");
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
    rclcpp::executors::MultiThreadedExecutor executor;
    std::shared_ptr<RecorderNode> node = std::make_shared<RecorderNode>();
    executor.add_node( node );
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
