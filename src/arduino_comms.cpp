#include "diffdrive_arduino/arduino_comms.h"
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>

// Setup the serial connection
void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt);
    serial_conn_.open();
}

// Send a keep-alive message
void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

// Read 4 encoder values from Arduino
void ArduinoComms::readEncoderValues(int &lf_enc, int &lb_enc, int &rf_enc, int &rb_enc)
{
    std::string response = sendMsg("e\r");  // Example response: "123 124 125 126"

    std::stringstream ss(response);
    std::string token;
    int values[4] = {0};

    for (int i = 0; i < 4 && ss >> token; ++i)
    {
        values[i] = std::atoi(token.c_str());
    }

    // Compute averages
    int left_avg = (values[0] + values[1]) / 2;
    int right_avg = (values[2] + values[3]) / 2;

    // Store the averaged value in both encoders of each side
    lf_enc = left_avg;
    lb_enc = left_avg;
    rf_enc = right_avg;
    rb_enc = right_avg;
}


// Send 4 motor commands to Arduino
void ArduinoComms::setMotorValues(int lf_cmd, int lb_cmd, int rf_cmd, int rb_cmd)
{
    std::stringstream ss;
    ss << "m " << lf_cmd << " " << lb_cmd << " " << rf_cmd << " " << rb_cmd << "\r";
    sendMsg(ss.str(), false);
}

// Send PID gains
void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

// Serial write + read wrapper
std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline();

    if (print_output)
    {
        // Uncomment for debug logs
        // RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        // RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }

    return response;
}
