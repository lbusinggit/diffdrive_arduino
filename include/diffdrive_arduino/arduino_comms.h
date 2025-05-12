#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <serial/serial.h>
#include <cstring>
#include <string>
#include <array>

// This class handles all communication between the computer and the Arduino for 4 motors and 4 encoders
class ArduinoComms
{
public:
  ArduinoComms() {}

  ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
    : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms))
  {}

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

  void sendEmptyMsg();

  // Reads 4 encoder values from Arduino
  void readEncoderValues(int &lf_enc, int &lb_enc, int &rf_enc, int &rb_enc);

  // Sends 4 motor command values to Arduino
  void setMotorValues(int lf_cmd, int lb_cmd, int rf_cmd, int rb_cmd);

  // Set PID parameters (for now assumed same for all wheels, but could be extended)
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const { return serial_conn_.isOpen(); }

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);

private:
  serial::Serial serial_conn_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
