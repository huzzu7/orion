// serial_drive_hardware.hpp
#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_port.hpp"
#include "io_context/io_context.hpp"

namespace serial_drive
{
class SerialDriveHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SerialDriveHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<drivers::serial_driver::SerialPort> serial_port_;
  std::unique_ptr<drivers::common::IoContext> io_context_;
  double cmd_[2];     // Left, Right wheel velocity commands
  double pos_[2];     // Left, Right wheel positions
  double vel_[2];     // Left, Right wheel velocities
};
} // namespace serial_drive