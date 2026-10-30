#ifndef THEODWYNROS_THEOCORE_SABERTOOTH_
#define THEODWYNROS_THEOCORE_SABERTOOTH_

#include <boost/asio.hpp>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "theo_msgs/msg/theo_mechanum_cmd.hpp"

/// @brief Minimal wrapper around a Boost.Asio serial port that writes single bytes.
class SimpleSerialWriter
{

public:
    /// IO service the serial port is attached to.
    boost::asio::io_service &io_srvc;
    /// Underlying serial port.
    boost::asio::serial_port serial;

    /// @param io_srvc_in IO service used to construct the serial port.
    SimpleSerialWriter(boost::asio::io_service &);
    /// @brief Closes the serial port.
    ~SimpleSerialWriter();

    /**
     * @brief Opens the port and sets the baud rate, closing it first if already open.
     * @param portname Device path (e.g. /dev/ttyTHS1).
     * @param baudrate Baud rate.
     * @return 0 on success, -1 on failure (error is printed to stderr).
     */
    int configureSerial(const std::string &, unsigned int);

    /**
     * @brief Writes one byte to the port.
     * @param uint8_in Byte to write.
     * @return 0 on success, -1 on failure.
     */
    int writeSerialUI8(uint8_t &);

    /// @brief Returns whether the serial port is open.
    bool is_open();

    /// @brief Closes the serial port.
    void close();

private:
    /// @brief Throwing implementation of configureSerial().
    void _configureSerial(const std::string &portname, unsigned int baudrate);
};

/**
 * @brief Driver for one Sabertooth motor controller in simplified serial mode.
 *
 * Motor 0 is commanded with bytes 1-127 (64 = stop) and motor 1 with bytes 128-255 (192 = stop).
 */
class SingleSabertooth
{

public:
    /// IO service shared with the serial writer.
    boost::asio::io_service &io_srvc;
    /// Serial connection to the controller.
    SimpleSerialWriter serial_writer;

    /**
     * @param io_srvc_in IO service for the serial port.
     * @param portname   Serial device path.
     * @param baudrate   Baud rate.
     */
    SingleSabertooth(boost::asio::io_service &, const std::string &, unsigned int);

    /**
     * @brief Sends throttle commands to both motors of this controller.
     * @param throttle_0 Throttle for motor 0 in [-1, 1].
     * @param throttle_1 Throttle for motor 1 in [-1, 1].
     */
    void sendThrottles(double, double);

private:
    /// @brief Converts a [-1, 1] throttle to the motor 0 command byte (1-127, 64 = stop).
    uint8_t _convertThrottle_0(double) const;
    /// @brief Converts a [-1, 1] throttle to the motor 1 command byte (128-255, 192 = stop).
    uint8_t _convertThrottle_1(double) const;
};

/// @brief Pair of Sabertooth controllers driving four motors, each on its own serial port.
class DualSabertooth
{

public:
    /// IO service shared with both controllers.
    boost::asio::io_service &io_srvc;
    /// First controller (throttles 0 and 1).
    SingleSabertooth saber_0;
    /// Second controller (throttles 2 and 3).
    SingleSabertooth saber_1;

    /**
     * @param io_srvc_in IO service for the serial ports.
     * @param portname_0 Serial device path of the first controller.
     * @param portname_1 Serial device path of the second controller.
     * @param baudrate   Baud rate used for both ports.
     */
    DualSabertooth(boost::asio::io_service &, const std::string &, const std::string &, unsigned int);

    /**
     * @brief Sends throttles to all four motors; each in [-1, 1].
     * @param throttle_0 Motor 0 (saber_0).
     * @param throttle_1 Motor 1 (saber_0).
     * @param throttle_2 Motor 2 (saber_1).
     * @param throttle_3 Motor 3 (saber_1).
     */
    void sendThrottles(double, double, double, double);
};

/// @brief ROS node that forwards mecanum throttle commands to the Sabertooth controllers over serial.
class SabertoothSubscriber : public rclcpp::Node
{

public:
    /// IO service used by the serial connections.
    boost::asio::io_service &io_srvc;
    /// Serial device path of the first controller (ROS parameter portname_0).
    std::string portname_0;
    /// Serial device path of the second controller (ROS parameter portname_1).
    std::string portname_1;
    /// Serial baud rate (ROS parameter baudrate).
    int32_t baudrate;

    /// @param io_srvc_in IO service for the serial ports.
    SabertoothSubscriber(boost::asio::io_service &);

private:
    /// Controller pair; null until constructed.
    std::unique_ptr<DualSabertooth> sabertooth_ptr;
    /// Subscription to throttle commands.
    rclcpp::Subscription<theo_msgs::msg::TheoMechanumCmd>::SharedPtr subscription_;

    /**
     * @brief Forwards a throttle command to the controllers (throttles 2 and 3 are swapped to match wiring).
     * @param msg Throttle command.
     */
    void serialCallback(const theo_msgs::msg::TheoMechanumCmd::SharedPtr) const;
};

#endif
