#include "ros2_control_wheeled_robot_hardware/wheeled_robot_hardware.hpp"
#include "ros2_control_wheeled_robot_hardware/udp_wheeled_robot.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_wheeled_robot_hardware
{

WheeledRobotHardware::WheeledRobotHardware()
: Node("wheeled_robot_hardware"){}

hardware_interface::CallbackReturn WheeledRobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Инициализация массивов состояний и команд
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  // Проверка интерфейсов для каждого сустава
  for (const auto & joint : info_.joints) {
    RCLCPP_DEBUG(get_logger(), "Configuring joint: %s", joint.name.c_str());

    // Проверка command interfaces
    if (joint.command_interfaces.size() != 1 || 
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(get_logger(), "Joint %s must have exactly one velocity command interface", 
                  joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Проверка state interfaces
    bool has_position = false;
    bool has_velocity = false;
    for (const auto & state_interface : joint.state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION) has_position = true;
      if (state_interface.name == hardware_interface::HW_IF_VELOCITY) has_velocity = true;
    }
    
    if (!has_position || !has_velocity) {
      RCLCPP_ERROR(get_logger(), "Joint %s must have both position and velocity state interfaces",
                  joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Инициализация параметров подключения
  try {
    const std::string udp_ip = info_.hardware_parameters.at("udp_ip");
    const int udp_port = std::stoi(info_.hardware_parameters.at("udp_port"));
    const int local_port = std::stoi(info_.hardware_parameters.at("local_port"));

    wheel_separation_ = std::stod(info_.hardware_parameters.at("wheel_separation"));
    wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));

    udp_socket_ = std::make_unique<Eth_Socket>();
    if (!udp_socket_->Initialize(udp_ip, udp_port, local_port)) {
      throw std::runtime_error("Socket initialization failed");
    }

    RCLCPP_INFO(get_logger(), "UDP socket initialized: %s:%d (local port: %d)", 
                udp_ip.c_str(), udp_port, local_port);

  }
    catch (const std::out_of_range& e) {
    RCLCPP_ERROR(get_logger(), "Missing required parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  } 
    catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Initialization error: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Hardware interface successfully initialized");
  RCLCPP_INFO(get_logger(), " - Wheel separation: %.3f m", wheel_separation_);
  RCLCPP_INFO(get_logger(), " - Wheel radius: %.3f m", wheel_radius_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WheeledRobotHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring hardware interface...");

  // Сброс всех значений
  std::fill(hw_positions_.begin(), hw_positions_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);

  RCLCPP_INFO(get_logger(), "Successfully configured");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
WheeledRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
WheeledRobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn WheeledRobotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating hardware interface...");

  // Инициализация команд текущими значениями скоростей
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    hw_commands_[i] = hw_velocities_[i];
  }

  RCLCPP_INFO(get_logger(), "Successfully activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WheeledRobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating hardware interface...");
  
  // Остановка всех колес при деактивации
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  write(rclcpp::Time(), rclcpp::Duration(0, 0)); // Применяем нулевые команды

  RCLCPP_INFO(get_logger(), "Successfully deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WheeledRobotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double wheel_velocities[6] = {0.0};
  double wheel_positions[6] = {0.0};

  // Получение данных от робота
  if (!udp_socket_->GetWheelStates(wheel_velocities, wheel_positions)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, 
                         "Failed to receive wheel states");
    return hardware_interface::return_type::ERROR;
  }

  // Обновление состояний
  for (size_t i = 0; i < hw_velocities_.size(); ++i) {
    hw_velocities_[i] = wheel_velocities[i];
    
    // Если позиции не приходят от устройства, интегрируем скорости
    hw_positions_[i] = (wheel_positions[i] != 0.0) 
                      ? wheel_positions[i] 
                      : hw_positions_[i] + hw_velocities_[i] * period.seconds();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WheeledRobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double left_avg = (hw_commands_[0] + hw_commands_[1] + hw_commands_[2]) / 3.0;
  double right_avg = (hw_commands_[3] + hw_commands_[4] + hw_commands_[5]) / 3.0;

  double velocity_command[2] = {
    (left_avg + right_avg) * wheel_radius_ / 2.0,  // linear
    (right_avg - left_avg) * wheel_radius_ / wheel_separation_  // angular
  };

  if (!udp_socket_->SendWheelSpeeds(velocity_command)) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to send wheel speeds");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_wheeled_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_wheeled_robot_hardware::WheeledRobotHardware,
  hardware_interface::SystemInterface)