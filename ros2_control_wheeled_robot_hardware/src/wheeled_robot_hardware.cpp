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
: rclcpp::Node("wheeled_robot_hardware") {}

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

  // Проверка интерфейсов
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 || 
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(get_logger(), "Joint %s must have exactly one velocity command interface", 
                  joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

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

  for (const auto& joint : info_.joints) {
    RCLCPP_INFO(get_logger(), "Joint %s has interfaces:", joint.name.c_str());
    for (const auto& iface : joint.state_interfaces) {
      RCLCPP_INFO(get_logger(), " - State: %s", iface.name.c_str());
    }
    for (const auto& iface : joint.command_interfaces) {
      RCLCPP_INFO(get_logger(), " - Command: %s", iface.name.c_str());
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

    RCLCPP_INFO(get_logger(), "Hardware interface initialized with %zu joints", info_.joints.size());
    RCLCPP_INFO(get_logger(), " - Wheel separation: %.3f m", wheel_separation_);
    RCLCPP_INFO(get_logger(), " - Wheel radius: %.3f m", wheel_radius_);

  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Initialization failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

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
  RCLCPP_INFO(get_logger(), "Activating hardware interface");
  
  // Инициализация команд текущими значениями скоростей
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    hw_commands_[i] = hw_velocities_[i];
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WheeledRobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating hardware interface");
  
  // Остановка всех колес
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  write(rclcpp::Time(), rclcpp::Duration(0, 0));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WheeledRobotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::vector<double> wheel_velocities(hw_velocities_.size());
  std::vector<double> wheel_positions(hw_positions_.size());

  if (!udp_socket_->GetWheelStates(wheel_velocities.data(), wheel_positions.data())) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                       "Failed to receive wheel states. Using last values");
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < hw_velocities_.size(); ++i) {
    hw_velocities_[i] = wheel_velocities[i];
    hw_positions_[i] += hw_velocities_[i] * period.seconds(); // Интеграция позиции
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WheeledRobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (hw_commands_.size() < 2) {
    return hardware_interface::return_type::ERROR;
  }

  // Рассчитываем средние скорости для левых и правых колес
  double left_avg = 0.0, right_avg = 0.0;
  size_t left_count = 0, right_count = 0;
  
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    if (info_.joints[i].name.find("left") != std::string::npos) {
      left_avg += hw_commands_[i];
      left_count++;
    } else if (info_.joints[i].name.find("right") != std::string::npos) {
      right_avg += hw_commands_[i];
      right_count++;
    }
  }

  if (left_count > 0) left_avg /= left_count;
  if (right_count > 0) right_avg /= right_count;

  // Преобразуем в линейную и угловую скорости
  double velocity_command[2] = {
      (left_avg + right_avg) * wheel_radius_ / 2.0,  // linear (m/s)
      (right_avg - left_avg) * wheel_radius_ / wheel_separation_  // angular (rad/s)
  };

  if (!udp_socket_->SendWheelSpeeds(velocity_command)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                        "Failed to send wheel speeds");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_wheeled_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_wheeled_robot_hardware::WheeledRobotHardware,
  hardware_interface::SystemInterface)