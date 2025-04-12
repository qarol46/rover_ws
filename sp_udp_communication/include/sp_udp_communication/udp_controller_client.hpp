#ifndef SP_UDP_COMMUNICATION__UDP_CONTROLLER_CLIENT_HPP_
#define SP_UDP_COMMUNICATION__UDP_CONTROLLER_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sp_udp_communication/udp_manipulator.hpp"

namespace sp_udp_communication {

class UdpControllerClient : public rclcpp::Node
{
public:
    explicit UdpControllerClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void send_commands();
    std::unique_ptr<Eth_Socket> socket_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace sp_udp_communication

#endif // SP_UDP_COMMUNICATION__UDP_CONTROLLER_CLIENT_HPP_