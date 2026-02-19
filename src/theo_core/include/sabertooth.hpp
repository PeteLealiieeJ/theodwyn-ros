#include <boost/asio.hpp>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "theo_msgs/msg/theo_mechanum_cmd.hpp"


class SimpleSerialWriter{

    public:
        boost::asio::io_service&    io_srvc;
        boost::asio::serial_port    serial;

        SimpleSerialWriter( boost::asio::io_service&  );
        ~SimpleSerialWriter();
        int configureSerial( const std::string&, unsigned int);
        int writeSerialUI8( uint8_t& );
        bool is_open();
        void close();

    private:
        void _configureSerial( const std::string& portname, unsigned int baudrate );

};


class SingleSabertooth{

    public:
        boost::asio::io_service&    io_srvc;
        SimpleSerialWriter          serial_writer;

        SingleSabertooth( boost::asio::io_service&, const std::string&, unsigned int );
        void sendThrottles( double&, double& );

    private:
       
        uint8_t _convertThrottle_0( double& );
        uint8_t _convertThrottle_1( double& );
        
};


class DualSabertooth{

    public:
        boost::asio::io_service&    io_srvc;
        SingleSabertooth            saber_0;
        SingleSabertooth            saber_1;

        DualSabertooth( boost::asio::io_service&, const std::string&, const std::string&, unsigned int );
        void sendThrottles( double&, double&, double&, double& );

};


class SabertoothSubscriber : public rclcpp::Node {
    
    public:
        boost::asio::io_service&    io_srvc;
        std::string                 portname_0;
        std::string                 portname_1;
        int32_t                     baudrate;

        SabertoothSubscriber( boost::asio::io_service& );

    private:
        std::unique_ptr<DualSabertooth> sabertooth_ptr;
        rclcpp::Subscription<theo_msgs::msg::TheoMechanumCmd>::SharedPtr subscription_;

        void serialCallback( const theo_msgs::msg::TheoMechanumCmd::SharedPtr ) const;
};