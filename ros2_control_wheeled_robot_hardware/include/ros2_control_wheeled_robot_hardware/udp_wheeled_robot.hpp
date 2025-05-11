#ifndef UDP_WHEELED_ROBOT_HPP_
#define UDP_WHEELED_ROBOT_HPP_

#include "ros2_control_wheeled_robot_hardware/message.hpp"
#include <asio.hpp>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>  // Добавляем этот include
#include <rclcpp/rclcpp.hpp>

class Eth_Socket
{
public:
    Eth_Socket();
    ~Eth_Socket();

    bool Initialize(const std::string& ip, int server_port, int local_port);
    bool IsValid() const { return socket_.is_open(); }

    bool SendWheelSpeeds(const double speeds[2]);
    bool GetWheelStates(double velocities[6], double positions[6]);

private:
    void handle_receive(const asio::error_code& error, size_t bytes_transferred);
    void start_receive();
    bool first_update_ = true;
    std::mutex send_mutex_;  // Добавляем мьютекс сюда

    asio::io_context io_context_;
    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint server_endpoint_;
    asio::ip::udp::endpoint remote_endpoint_;
    std::array<uint8_t, sizeof(Message)> recv_buffer_;
    std::atomic<bool> response_received_{false};
    Message last_received_msg_;
    std::thread io_thread_;
};

#endif