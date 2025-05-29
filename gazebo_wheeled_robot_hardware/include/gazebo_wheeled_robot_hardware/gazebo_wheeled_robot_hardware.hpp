#ifndef GAZEBO_WHEELED_ROBOT_HARDWARE_HPP_
#define GAZEBO_WHEELED_ROBOT_HARDWARE_HPP_

#include <gazebo_ros_control/gazebo_ros_control_plugin.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>

namespace gazebo_ros_control
{

class GazeboWheeledRobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GazeboWheeledRobotHardware)

  GazeboWheeledRobotHardware();
  virtual ~GazeboWheeledRobotHardware() = default;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void SetModel(gazebo::physics::ModelPtr model);

private:
  gazebo::physics::ModelPtr model_;
  std::vector<gazebo::physics::JointPtr> joints_;
  
  std::vector<double> hw_commands_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_positions_;
  
  double wheel_separation_ = 0.0;
  double wheel_radius_ = 0.0;
  
  rclcpp::Logger logger_;
};

}  // namespace gazebo_ros_control

#endif  // GAZEBO_WHEELED_ROBOT_HARDWARE_HPP_