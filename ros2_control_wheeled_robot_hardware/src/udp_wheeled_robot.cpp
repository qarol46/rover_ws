#include "ros2_control_wheeled_robot_hardware/udp_wheeled_robot.hpp"
#include <chrono>
#include <thread>

Eth_Socket::Eth_Socket() : socket_(io_context_) {}

Eth_Socket::~Eth_Socket() {
    io_context_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
    socket_.close();
}

bool Eth_Socket::Initialize(const std::string& ip, int server_port, int local_port) {
    try {
        socket_.open(asio::ip::udp::v4());
        socket_.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), local_port));
        server_endpoint_ = asio::ip::udp::endpoint(
            asio::ip::address::from_string(ip), server_port);

        socket_.set_option(asio::socket_base::reuse_address(true));
        io_thread_ = std::thread([this]() { io_context_.run(); });
        start_receive();
        
        RCLCPP_INFO(rclcpp::get_logger("Eth_Socket"), 
                   "UDP Client initialized: local port %d, server %s:%d", 
                   local_port, ip.c_str(), server_port);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Eth_Socket"), 
                    "UDP initialization error: %s", e.what());
        return false;
    }
}

bool Eth_Socket::SendWheelSpeeds(const double speeds[2]) {
    try {
        Message msg;
        msg.linear_vel = static_cast<int16_t>(speeds[0] * 1000);  // m/s -> mm/s
        msg.angular_vel = static_cast<int16_t>(speeds[1] * 1000); // rad/s -> mrad/s

        std::array<uint8_t, sizeof(Message)> send_buffer;
        std::memcpy(send_buffer.data(), &msg, sizeof(Message));

        response_received_ = false;
        socket_.async_send_to(
            asio::buffer(send_buffer), server_endpoint_,
            [this](const asio::error_code& error, size_t /*bytes_sent*/) {
                if (error) {
                    RCLCPP_ERROR(rclcpp::get_logger("Eth_Socket"), 
                                "Send error: %s", error.message().c_str());
                }
            });

        // Wait for response with timeout
        auto start = std::chrono::steady_clock::now();
        while (!response_received_ && 
              std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        return response_received_;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Eth_Socket"), "Send error: %s", e.what());
        return false;
    }
}

bool Eth_Socket::GetWheelStates(double velocities[6], double positions[6]) {
    if (!response_received_) return false;

    for (int i = 0; i < 6; ++i) {
        velocities[i] = static_cast<double>(last_received_msg_.velocity[i]) / 1000.0;
        positions[i] = static_cast<double>(last_received_msg_.odom[i]) / 1000.0;
    }
    return true;
}

void Eth_Socket::start_receive() {
    socket_.async_receive_from(
        asio::buffer(recv_buffer_), remote_endpoint_,
        [this](const asio::error_code& error, size_t bytes_transferred) {
            handle_receive(error, bytes_transferred);
        });
}

void Eth_Socket::handle_receive(const asio::error_code& error, size_t bytes_transferred) {
    if (!error && bytes_transferred == sizeof(Message)) {
        std::memcpy(&last_received_msg_, recv_buffer_.data(), sizeof(Message));
        response_received_ = true;
    }
    start_receive();
}