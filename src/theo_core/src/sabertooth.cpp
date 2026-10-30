#include "sabertooth.hpp"

// ----------------------------------------------------------
// Simple Serial Connection
// ----------------------------------------------------------

SimpleSerialWriter::SimpleSerialWriter(boost::asio::io_service &io_srvc_in)
    : io_srvc(io_srvc_in), serial(io_srvc)
{
}

SimpleSerialWriter::~SimpleSerialWriter()
{
    close();
}

int SimpleSerialWriter::configureSerial(
    const std::string &portname,
    unsigned int baudrate)
{
    try
    {
        _configureSerial(portname, baudrate);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Could not configure properly "
                  << portname << ": " << e.what()
                  << std::endl;
        return -1;
    }
    return 0;
}

void SimpleSerialWriter::_configureSerial(
    const std::string &portname,
    unsigned int baudrate)
{
    if (serial.is_open())
        serial.close();
    serial.open(portname);
    serial.set_option(
        boost::asio::serial_port::baud_rate(baudrate));
}

int SimpleSerialWriter::writeSerialUI8(uint8_t &uint8_in)
{
    try
    {
        boost::asio::write(serial, boost::asio::buffer(&uint8_in, 1));
    }
    catch (boost::system::error_code &e)
    {
        std::cerr << "Could not write properly : "
                  << e.message()
                  << std::endl;
        return -1;
    }
    return 0;
}

bool SimpleSerialWriter::is_open()
{
    return serial.is_open();
}

void SimpleSerialWriter::close()
{
    serial.close();
}

// ----------------------------------------------------------
// Sabertooth
// ----------------------------------------------------------

SingleSabertooth::SingleSabertooth(
    boost::asio::io_service &io_srvc_in,
    const std::string &portname,
    unsigned int baudrate) : io_srvc(io_srvc_in), serial_writer(io_srvc)
{
    serial_writer.configureSerial(portname, baudrate);
}

void SingleSabertooth::sendThrottles(
    double throttle_0,
    double throttle_1)
{
    uint8_t sig8b_0 = _convertThrottle_0(throttle_0);
    uint8_t sig8b_1 = _convertThrottle_1(throttle_1);

    serial_writer.writeSerialUI8(sig8b_0);
    // Maybe include a delay ~5 microseconds
    // (idk) between these calls
    serial_writer.writeSerialUI8(sig8b_1);
}

uint8_t SingleSabertooth::_convertThrottle_0(double throttle_0) const
{
    int sigout = 64;
    if (throttle_0 > 0.)
    {
        sigout = std::min(
            (int)std::round(64. + throttle_0 * 63.),
            127);
    }
    else if (throttle_0 < 0.)
    {
        sigout = std::max(
            (int)std::round(64. + throttle_0 * 63.),
            1);
    }
    return sigout;
}

uint8_t SingleSabertooth::_convertThrottle_1(double throttle_1) const
{
    int sigout = 192;
    if (throttle_1 > 0.)
    {
        sigout = std::min(
            (int)std::round(192. + throttle_1 * 63.),
            255);
    }
    else if (throttle_1 < 0.)
    {
        sigout = std::max(
            (int)std::round(192. + throttle_1 * 64.),
            128);
    }
    return sigout;
}

// ----------------------------------------------------------
// Dual Sabertooth
// ----------------------------------------------------------

DualSabertooth::DualSabertooth(
    boost::asio::io_service &io_srvc_in,
    const std::string &portname_0,
    const std::string &portname_1,
    unsigned int baudrate) : io_srvc(io_srvc_in),
                             saber_0(io_srvc, portname_0, baudrate),
                             saber_1(io_srvc, portname_1, baudrate) {};

void DualSabertooth::sendThrottles(
    double throttle_0,
    double throttle_1,
    double throttle_2,
    double throttle_3)
{
    saber_0.sendThrottles(throttle_0, throttle_1);
    saber_1.sendThrottles(throttle_2, throttle_3);
};

// ----------------------------------------------------------
// Sabertooth Subscriber Node
// ----------------------------------------------------------

SabertoothSubscriber::SabertoothSubscriber(
    boost::asio::io_service &io_srvc_in)
    : Node("sabertooth_node"), io_srvc(io_srvc_in), sabertooth_ptr(nullptr)
{

    // Pull Down Parameters from YAML
    declare_parameter<std::string>("portname_0", "/dev/ttyTHS1");
    declare_parameter<std::string>("portname_1", "/dev/ttyTHS2");
    declare_parameter<int32_t>("baudrate", 9600);

    get_parameter("portname_0", portname_0);
    get_parameter("portname_1", portname_1);
    get_parameter("baudrate", baudrate);

    // Configure Serial Connection
    sabertooth_ptr = std::make_unique<DualSabertooth>(
        io_srvc,
        portname_0,
        portname_1,
        static_cast<unsigned int>(baudrate));

    // The ROS Subscrription
    subscription_ = create_subscription<theo_msgs::msg::TheoMechanumCmd>(
        "thrtlcmd_topic",
        10,
        std::bind(
            &SabertoothSubscriber::serialCallback,
            this,
            std::placeholders::_1));
};

void SabertoothSubscriber::serialCallback(
    const theo_msgs::msg::TheoMechanumCmd::SharedPtr msg) const
{
    if (sabertooth_ptr)
    { // Check nullptr
        sabertooth_ptr->sendThrottles(
            msg->throttle_0,
            msg->throttle_1,
            msg->throttle_3, // based on wiring
            msg->throttle_2);
    }
};

// ----------------------------------------------------------
// Main
// ----------------------------------------------------------

int main(int argc, char *argv[])
{
    boost::asio::io_service io;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SabertoothSubscriber>(io));
    rclcpp::shutdown();
    return 0;
}