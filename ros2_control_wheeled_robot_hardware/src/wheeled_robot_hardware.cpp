#include "ros2_control_wheeled_robot_hardware/wheeled_robot_hardware.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <stdexcept>

namespace ros2_control_wheeled_robot_hardware
{

WheeledRobotHardware::WheeledRobotHardware()
: logger_(rclcpp::get_logger("WheeledRobotHardware"))
{
}

hardware_interface::CallbackReturn WheeledRobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 || 
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(logger_, "Joint %s must have exactly one velocity command interface", 
                  joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_velocity = false;
    for (const auto & state_interface : joint.state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_VELOCITY) has_velocity = true;
    }

    if (!has_velocity) {
      RCLCPP_ERROR(logger_, "Joint %s must have velocity state interface",
                  joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  try {
    wheel_separation_ = std::stod(info_.hardware_parameters.at("wheel_separation"));
    wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
    
    const std::string udp_ip = info_.hardware_parameters.at("udp_ip");
    const int udp_port = std::stoi(info_.hardware_parameters.at("udp_port"));
    const int local_port = std::stoi(info_.hardware_parameters.at("local_port"));

    udp_socket_ = std::make_unique<Eth_Socket>();
    if (!udp_socket_->Initialize(udp_ip, udp_port, local_port)) {
      throw std::runtime_error("Socket initialization failed");
    }
    socket_connected_ = true;

    RCLCPP_INFO(logger_, "Hardware initialized with %zu joints", info_.joints.size());
    RCLCPP_INFO(logger_, "Wheel separation: %.3f m, Wheel radius: %.3f m", 
               wheel_separation_, wheel_radius_);

  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Initialization failed: %s", e.what());
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
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
        
    // Add position state interface if it's declared in the URDF
    for (const auto & state_interface : info_.joints[i].state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_POSITION) {
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            info_.joints[i].name,
            hardware_interface::HW_IF_POSITION,
            &hw_positions_[i]));
      }
    }
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
  const rclcpp_lifecycle::State & )
{
  RCLCPP_INFO(logger_, "Activating hardware interface");
  first_read_ = true;
  
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    hw_commands_[i] = hw_velocities_[i];
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WheeledRobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & )
{
  RCLCPP_INFO(logger_, "Deactivating hardware interface");
  
  // Stop all wheels
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  write(rclcpp::Time(), rclcpp::Duration(0, 0));

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type WheeledRobotHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!socket_connected_) {
    RCLCPP_WARN(logger_, "Socket not connected");
    return hardware_interface::return_type::ERROR;
  }

  double wheel_velocities[6] = {0};
  double wheel_positions[6] = {0};
  
  if (udp_socket_->GetWheelStates(wheel_velocities, wheel_positions)) {
    if (first_read_) {
      last_successful_comm_ = time;
      first_read_ = false;
    }
    RCLCPP_INFO(logger_, "GetWheelStates success");

    for (size_t i = 0; i < hw_velocities_.size(); ++i) {
      hw_velocities_[i] = wheel_velocities[i];
      hw_positions_[i] += hw_velocities_[i] * period.seconds();
    }
    
    last_successful_comm_ = time;
  } else {
    if (!first_read_ && (time - last_successful_comm_).seconds() > COMM_TIMEOUT) {
      socket_connected_ = false;
      RCLCPP_ERROR(logger_, "Communication timeout, marking socket as disconnected");
    }
    return hardware_interface::return_type::ERROR;

    RCLCPP_INFO(logger_, "GetWheelStates failed");
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WheeledRobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!socket_connected_) {
    RCLCPP_WARN(logger_, "Unable to send commands - socket disconnected");
    return hardware_interface::return_type::ERROR;
  }

  if (hw_commands_.size() < 2) {
    return hardware_interface::return_type::ERROR;
  }

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

  double velocity_command[2] = {
    (left_avg + right_avg) * wheel_radius_ / 2.0,
    (right_avg - left_avg) * wheel_radius_ / wheel_separation_
  };

  if (!udp_socket_->SendWheelSpeeds(velocity_command)) {
    RCLCPP_ERROR(logger_, "Failed to send wheel speeds");
    socket_connected_ = false;
    return hardware_interface::return_type::ERROR;
  }
  
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_wheeled_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_wheeled_robot_hardware::WheeledRobotHardware,
  hardware_interface::SystemInterface)