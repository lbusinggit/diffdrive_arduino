#include "diffdrive_arduino/fake_robot.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

FakeRobot::FakeRobot()
  : logger_(rclcpp::get_logger("FakeRobot"))
{}

return_type FakeRobot::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  cfg_.left_front_wheel_name  = info_.hardware_parameters["left_front_wheel_name"];
  cfg_.left_back_wheel_name   = info_.hardware_parameters["left_back_wheel_name"];
  cfg_.right_front_wheel_name = info_.hardware_parameters["right_front_wheel_name"];
  cfg_.right_back_wheel_name  = info_.hardware_parameters["right_back_wheel_name"];

  lf_wheel_.setup(cfg_.left_front_wheel_name, cfg_.enc_counts_per_rev);
  lb_wheel_.setup(cfg_.left_back_wheel_name, cfg_.enc_counts_per_rev);
  rf_wheel_.setup(cfg_.right_front_wheel_name, cfg_.enc_counts_per_rev);
  rb_wheel_.setup(cfg_.right_back_wheel_name, cfg_.enc_counts_per_rev);

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> FakeRobot::export_state_interfaces()
{
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

std::vector<hardware_interface::CommandInterface> FakeRobot::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(lf_wheel_.name, hardware_interface::HW_IF_VELOCITY, &lf_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(lb_wheel_.name, hardware_interface::HW_IF_VELOCITY, &lb_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(rf_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rf_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(rb_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rb_wheel_.cmd));

  return command_interfaces;
}

return_type FakeRobot::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");
  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

return_type FakeRobot::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}

hardware_interface::return_type FakeRobot::read()
{
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;

  lf_wheel_.pos += lf_wheel_.vel * deltaSeconds;
  lb_wheel_.pos += lb_wheel_.vel * deltaSeconds;
  rf_wheel_.pos += rf_wheel_.vel * deltaSeconds;
  rb_wheel_.pos += rb_wheel_.vel * deltaSeconds;

  return return_type::OK;
}

hardware_interface::return_type FakeRobot::write()
{
  lf_wheel_.vel = lf_wheel_.cmd;
  lb_wheel_.vel = lb_wheel_.cmd;
  rf_wheel_.vel = rf_wheel_.cmd;
  rb_wheel_.vel = rb_wheel_.cmd;

  return return_type::OK;  
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  FakeRobot,
  hardware_interface::SystemInterface
)
