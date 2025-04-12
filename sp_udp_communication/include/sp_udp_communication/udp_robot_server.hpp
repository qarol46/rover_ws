#ifndef SP_UDP_COMMUNICATION__UDP_ROBOT_SERVER_HPP_
#define SP_UDP_COMMUNICATION__UDP_ROBOT_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sp_udp_communication/udp_manipulator.hpp"

namespace sp_udp_communication {

class UdpRobotServer : public rclcpp::Node
{
public:
    explicit UdpRobotServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void process_incoming_data();
    std::unique_ptr<Eth_Socket> socket_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace sp_udp_communication

#endif // SP_UDP_COMMUNICATION__UDP_ROBOT_SERVER_HPP_