#ifndef DIFFDRIVE_ARDUINO_REAL_ROBOT_H
#define DIFFDRIVE_ARDUINO_REAL_ROBOT_H

#include <cstring>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include "config.h"
#include "wheel.h"
#include "arduino_comms.h"

using hardware_interface::return_type;

class DiffDriveArduino : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  DiffDriveArduino();  // Constructor

  // Setup using robot description (from URDF)
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  // Provide controller access to hardware state
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Provide controller access to write commands
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Called when interface starts
  return_type start() override;

  // Called when interface stops
  return_type stop() override;

  // Read encoder/state data from Arduino
  return_type read() override;

  // Write velocity/command data to Arduino
  return_type write() override;

private:
  Config cfg_;             // Configuration (from xacro/URDF)
  ArduinoComms arduino_;   // Serial comm handler

  // 4 wheels: front and back for both sides
  Wheel lf_wheel_;  // Left front
  Wheel lb_wheel_;  // Left back
  Wheel rf_wheel_;  // Right front
  Wheel rb_wheel_;  // Right back

  rclcpp::Logger logger_;  // ROS logger

  std::chrono::time_point<std::chrono::system_clock> time_;  // Time tracking
};

#endif // DIFFDRIVE_ARDUINO_REAL_ROBOT_H
