#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>

// Struct to store configuration settings for the 4-wheel diff drive robot
struct Config
{
  // Names of the left side wheel joints
  std::string left_front_wheel_name = "left_wheel_fwd_joint";
  std::string left_back_wheel_name  = "left_wheel_rear_joint";

  // Names of the right side wheel joints
  std::string right_front_wheel_name = "right_wheel_fwd_joint";
  std::string right_back_wheel_name  = "right_wheel_rear_joint";

  // Control loop frequency in Hz (number of updates per second)
  float loop_rate = 30;

  // Serial port name to connect to Arduino (Linux device path)
  std::string device = "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3.3:1.0-port0";

  // Baud rate for serial communication
  int baud_rate = 57600;

  // Timeout for serial communication in milliseconds
  int timeout = 1000;

  // Encoder ticks per revolution of the motor shaft
  int enc_counts_per_rev = 210;
};

#endif // DIFFDRIVE_ARDUINO_CONFIG_H
