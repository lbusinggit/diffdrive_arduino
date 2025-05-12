#include "diffdrive_arduino/arduino_comms.h"
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>

// Setup the serial connection
void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
    serial_conn_.setPort(serial_device);  // Set serial port (e.g., /dev/ttyUSB0)
    serial_conn_.setBaudrate(baud_rate);  // Set baud rate (e.g., 57600)
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);  // Set timeout
    serial_conn_.setTimeout(tt);
    serial_conn_.open();  // Open serial port
}

// Send a simple message just to keep communication alive
void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

// Request encoder values from Arduino
void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    std::string response = sendMsg("e\r");  // Send command to get encoder data

    std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);  // Find space separating the two numbers
    std::string token_1 = response.substr(0, del_pos);              // First value
    std::string token_2 = response.substr(del_pos + delimiter.length()); // Second value

    val_1 = std::atoi(token_1.c_str());  // Convert string to int
    val_2 = std::atoi(token_2.c_str());
}

// Send motor speed commands to Arduino
void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";  // Format string
    sendMsg(ss.str(), false);  // Send without printing
}

// Send PID gain values to Arduino
void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";  // Format: "u kp:kd:ki:ko"
    sendMsg(ss.str());  // Send the message
}

// Core message-sending function
std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.write(msg_to_send);              // Write to serial
    std::string response = serial_conn_.readline(); // Wait for Arduino response

    if (print_output)
    {
        // Logging lines (currently disabled)
        // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }

    return response;  // Return response to caller
}
