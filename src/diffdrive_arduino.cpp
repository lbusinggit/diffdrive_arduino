#include "diffdrive_arduino/diffdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

DiffDriveArduino::DiffDriveArduino()
: logger_(rclcpp::get_logger("DiffDriveArduino")) {}

return_type DiffDriveArduino::configure(const hardware_interface::HardwareInfo & info) {
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");
  time_ = std::chrono::system_clock::now();

  // Load config parameters from hardcoded Config struct
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  cfg_.left_front_wheel_name = info_.hardware_parameters["left_front_wheel_name"];
  cfg_.left_back_wheel_name = info_.hardware_parameters["left_back_wheel_name"];
  cfg_.right_front_wheel_name = info_.hardware_parameters["right_front_wheel_name"];
  cfg_.right_back_wheel_name = info_.hardware_parameters["right_back_wheel_name"];


  // Setup each wheel
  lf_wheel_.setup(cfg_.left_front_wheel_name, cfg_.enc_counts_per_rev);
  lb_wheel_.setup(cfg_.left_back_wheel_name, cfg_.enc_counts_per_rev);
  rf_wheel_.setup(cfg_.right_front_wheel_name, cfg_.enc_counts_per_rev);
  rb_wheel_.setup(cfg_.right_back_wheel_name, cfg_.enc_counts_per_rev);

  // Connect to Arduino
  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);

  RCLCPP_INFO(logger_, "Finished Configuration");
  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(lf_wheel_.name, hardware_interface::HW_IF_VELOCITY, &lf_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(lf_wheel_.name, hardware_interface::HW_IF_POSITION, &lf_wheel_.pos));

  state_interfaces.emplace_back(hardware_interface::StateInterface(lb_wheel_.name, hardware_interface::HW_IF_VELOCITY, &lb_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(lb_wheel_.name, hardware_interface::HW_IF_POSITION, &lb_wheel_.pos));

  state_interfaces.emplace_back(hardware_interface::StateInterface(rf_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rf_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(rf_wheel_.name, hardware_interface::HW_IF_POSITION, &rf_wheel_.pos));

  state_interfaces.emplace_back(hardware_interface::StateInterface(rb_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rb_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(rb_wheel_.name, hardware_interface::HW_IF_POSITION, &rb_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(lf_wheel_.name, hardware_interface::HW_IF_VELOCITY, &lf_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(lb_wheel_.name, hardware_interface::HW_IF_VELOCITY, &lb_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(rf_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rf_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(rb_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rb_wheel_.cmd));

  return command_interfaces;
}

return_type DiffDriveArduino::start() {
  RCLCPP_INFO(logger_, "Starting Controller...");
  arduino_.sendEmptyMsg();
  arduino_.setPidValues(30, 20, 0, 100);  // PID: P, D, I, scaling
  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type DiffDriveArduino::stop() {
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::read() {
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;

  if (!arduino_.connected()) return return_type::ERROR;

  // Read encoder counts
  arduino_.readEncoderValues(lf_wheel_.enc, lb_wheel_.enc, rf_wheel_.enc, rb_wheel_.enc);

  // Update each wheel's pos and vel
  auto update_wheel = [deltaSeconds](Wheel &wheel) {
    double prev_pos = wheel.pos;
    wheel.pos = wheel.calcEncAngle();
    wheel.vel = (wheel.pos - prev_pos) / deltaSeconds;
  };

  update_wheel(lf_wheel_);
  update_wheel(lb_wheel_);
  update_wheel(rf_wheel_);
  update_wheel(rb_wheel_);

  return return_type::OK;
}
hardware_interface::return_type DiffDriveArduino::write() {
  if (!arduino_.connected()) return return_type::ERROR;

  // Compute average left and right wheel commands
  double left_cmd_avg = (lf_wheel_.cmd + lb_wheel_.cmd) / 2.0;
  double right_cmd_avg = (rf_wheel_.cmd + rb_wheel_.cmd) / 2.0;

  // Convert to encoder ticks per loop iteration
  int left_cmd_ticks = static_cast<int>(left_cmd_avg / lf_wheel_.rads_per_count / cfg_.loop_rate);
  int right_cmd_ticks = static_cast<int>(right_cmd_avg / rf_wheel_.rads_per_count / cfg_.loop_rate);

  // Send the same command to both motors on each side
  arduino_.setMotorValues(
    left_cmd_ticks,  // Left Front
    left_cmd_ticks,  // Left Back
    right_cmd_ticks, // Right Front
    right_cmd_ticks  // Right Back
  );

  return return_type::OK;
}



// Plugin export
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(DiffDriveArduino, hardware_interface::SystemInterface)
