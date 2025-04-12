#include "sp_udp_communication/udp_robot_server.hpp"

namespace sp_udp_communication {

UdpRobotServer::UdpRobotServer(const rclcpp::NodeOptions & options)
    : Node("udp_robot_server", options)
{
    try {
        socket_ = std::make_unique<Eth_Socket>();
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            [this]() { this->process_incoming_data(); });
            
        RCLCPP_INFO(this->get_logger(), "UDP Robot Server initialized");
    } catch (const std::exception & e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize: %s", e.what());
        throw;
    }
}

void UdpRobotServer::process_incoming_data()
{
    double wheel_velocities[6];
    double wheel_positions[6];
    double torque_feedback[6];
    
    if (socket_->move(0, 0, wheel_velocities, torque_feedback, wheel_positions)) {
        RCLCPP_DEBUG(this->get_logger(), "Processed incoming data");
    }
}

} // namespace sp_udp_communication

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sp_udp_communication::UdpRobotServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}