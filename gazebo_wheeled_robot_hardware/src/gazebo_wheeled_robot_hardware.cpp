#include "gazebo_wheeled_robot_hardware.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gazebo_ros_control
{

GazeboWheeledRobotHardware::GazeboWheeledRobotHardware()
: logger_(rclcpp::get_logger("GazeboWheeledRobotHardware"))
{
}

void GazeboWheeledRobotHardware::SetModel(gazebo::physics::ModelPtr model)
{
  model_ = model;
}

hardware_interface::CallbackReturn GazeboWheeledRobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);

  // Проверка интерфейсов
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 || 
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(logger_, "Joint %s must have exactly one velocity command interface", 
                  joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_velocity = false;
    bool has_position = false;
    for (const auto & state_interface : joint.state_interfaces) {
      if (state_interface.name == hardware_interface::HW_IF_VELOCITY) has_velocity = true;
      if (state_interface.name == hardware_interface::HW_IF_POSITION) has_position = true;
    }

    if (!has_velocity || !has_position) {
      RCLCPP_ERROR(logger_, "Joint %s must have velocity and position state interfaces",
                  joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  try {
    wheel_separation_ = std::stod(info_.hardware_parameters.at("wheel_separation"));
    wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
    
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
GazeboWheeledRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_[i]));
        
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
GazeboWheeledRobotHardware::export_command_interfaces()
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

hardware_interface::CallbackReturn GazeboWheeledRobotHardware::on_activate(
  const rclcpp_lifecycle::State & )
{
  RCLCPP_INFO(logger_, "Activating hardware interface");
  
  if (!model_) {
    RCLCPP_ERROR(logger_, "Gazebo model is not set");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Получаем указатели на joints из Gazebo
  joints_.clear();
  for (const auto & joint_info : info_.joints) {
    gazebo::physics::JointPtr joint = model_->GetJoint(joint_info.name);
    if (!joint) {
      RCLCPP_ERROR(logger_, "Joint %s not found in Gazebo model", joint_info.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joints_.push_back(joint);
  }

  // Инициализируем команды
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    hw_commands_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_positions_[i] = joints_[i]->Position(0);
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GazeboWheeledRobotHardware::on_deactivate(
  const rclcpp_lifecycle::State & )
{
  RCLCPP_INFO(logger_, "Deactivating hardware interface");
  
  // Останавливаем все колеса
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  for (auto & joint : joints_) {
    joint->SetVelocity(0, 0.0);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GazeboWheeledRobotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (joints_.empty()) {
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joints_.size(); ++i) {
    hw_velocities_[i] = joints_[i]->GetVelocity(0);
    hw_positions_[i] = joints_[i]->Position(0);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboWheeledRobotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (joints_.empty()) {
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < joints_.size(); ++i) {
    joints_[i]->SetVelocity(0, hw_commands_[i]);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace gazebo_ros_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  gazebo_ros_control::GazeboWheeledRobotHardware,
  hardware_interface::SystemInterface)