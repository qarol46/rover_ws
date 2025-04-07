#ifndef WHEELED_ROBOT_HARDWARE_HPP_
#define WHEELED_ROBOT_HARDWARE_HPP_

#include <vector>
#include <memory>
#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "udp_wheeled_robot.hpp"

namespace ros2_control_wheeled_robot_hardware
{
class WheeledRobotHardware : public hardware_interface::SystemInterface, public rclcpp::Node
{
public:
  WheeledRobotHardware();

  // Методы интерфейса оборудования
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // UDP-сокет для обмена данными с оборудованием
  std::unique_ptr<Eth_Socket> udp_socket_;

  // Векторы для хранения состояний и команд
  std::vector<double> hw_positions_; 
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // Геометрические параметры
  double wheel_separation_;
  double wheel_radius_;
};

}  // namespace ros2_control_wheeled_robot_hardware

#endif  // WHEELED_ROBOT_HARDWARE_HPP_