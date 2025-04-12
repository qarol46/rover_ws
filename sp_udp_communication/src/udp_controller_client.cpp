#include "sp_udp_communication/udp_controller_client.hpp"

namespace sp_udp_communication {

UdpControllerClient::UdpControllerClient(const rclcpp::NodeOptions & options)
    : Node("udp_controller_client", options)
{
    try {
        socket_ = std::make_unique<Eth_Socket>();
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() { this->send_commands(); });
            
        RCLCPP_INFO(this->get_logger(), "UDP Controller Client initialized");
    } catch (const std::exception & e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize: %s", e.what());
        throw;
    }
}

void UdpControllerClient::send_commands()
{
    double vel_command[2] = {0.5, 0.1}; // Пример команд
    double wheel_velocities[6];
    double wheel_positions[6];
    
    socket_->SendCommand(vel_command);
    socket_->GetWheelStates(wheel_velocities, wheel_positions);
    
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Wheel states received");
}

} // namespace sp_udp_communication

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<sp_udp_communication::UdpControllerClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}