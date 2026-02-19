#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "theo_msgs/msg/theo_mechanum_cmd.hpp"


class MixerNode : public rclcpp::Node {

    public:
        Eigen::Matrix<double,4,3> MixMat;

        MixerNode();

    private:
        void mixingCallback( const geometry_msgs::msg::Twist::SharedPtr ) const;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        rclcpp::Publisher<theo_msgs::msg::TheoMechanumCmd>::SharedPtr publisher_;
};