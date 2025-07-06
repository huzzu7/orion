// serial_drive_hardware.cpp
#include "drive_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>

namespace serial_drive
{

hardware_interface::CallbackReturn SerialDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try
  {
    // Create IO context
    io_context_ = std::make_unique<drivers::common::IoContext>();
    
    // Get serial port configuration from parameters
    std::string device_name = info.hardware_parameters.at("serial_port");
    uint32_t baud_rate = std::stoi(info.hardware_parameters.at("baud_rate"));
    
    // Create serial port configuration
    auto serial_config = drivers::serial_driver::SerialPortConfig(
      baud_rate,
      drivers::serial_driver::FlowControl::NONE,
      drivers::serial_driver::Parity::NONE,
      drivers::serial_driver::StopBits::ONE
    );
    
    // Create serial port
    serial_port_ = std::make_unique<drivers::serial_driver::SerialPort>(
      *io_context_, device_name, serial_config);
    
    // Open the serial port
    serial_port_->open();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SerialDriveHardware"), "Failed to open serial port: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::fill(std::begin(cmd_), std::end(cmd_), 0.0);
  std::fill(std::begin(pos_), std::end(pos_), 0.0);
  std::fill(std::begin(vel_), std::end(vel_), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SerialDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < 2; ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SerialDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < 2; ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn SerialDriveHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SerialDriveHardware"), "Activating interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SerialDriveHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("SerialDriveHardware"), "Deactivating interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SerialDriveHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_port_ && serial_port_->is_open())
  {
    std::vector<uint8_t> buffer;
    size_t bytes_read = serial_port_->receive(buffer);
    
    if (bytes_read > 0)
    {
      // Convert buffer to string
      std::string line(buffer.begin(), buffer.end());
      
      // Parse encoder data
      double enc_left = 0.0, enc_right = 0.0;
      if (sscanf(line.c_str(), "encL:%lf,encR:%lf", &enc_left, &enc_right) == 2)
      {
        pos_[0] += enc_left;
        pos_[1] += enc_right;
        vel_[0] = enc_left;
        vel_[1] = enc_right;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SerialDriveHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_port_ && serial_port_->is_open())
  {
    std::stringstream ss;
    ss << "vL:" << cmd_[0] << ",vR:" << cmd_[1] << "\n";
    std::string command = ss.str();
    
    // Convert string to vector<uint8_t>
    std::vector<uint8_t> buffer(command.begin(), command.end());
    serial_port_->send(buffer);
  }
  return hardware_interface::return_type::OK;
}

} // namespace serial_drive

PLUGINLIB_EXPORT_CLASS(serial_drive::SerialDriveHardware, hardware_interface::SystemInterface)
