#include "ros2_control_wheeled_robot_hardware/udp_wheeled_robot.hpp"
#include <chrono>
#include <thread>
#include <iomanip>

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
        msg.linear_vel = static_cast<int16_t>(speeds[0] * 1000);
        msg.angular_vel = static_cast<int16_t>(speeds[1] * 1000);

        RCLCPP_INFO(rclcpp::get_logger("Eth_Socket"), 
                   "Sending: lin=%.3f m/s, ang=%.3f rad/s",
                   speeds[0], speeds[1]);

        std::array<uint8_t, sizeof(Message)> send_buffer;
        std::memcpy(send_buffer.data(), &msg, sizeof(Message));

        response_received_ = false;
        socket_.async_send_to(
            asio::buffer(send_buffer), server_endpoint_,
            [this](const asio::error_code& error, size_t bytes_sent) {
                if (error) {
                    RCLCPP_WARN(rclcpp::get_logger("Eth_Socket"), 
                               "Send warning: %s", error.message().c_str());
                } else {
                    RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"),
                                "Sent %zu bytes", bytes_sent);
                    this->start_receive();
                }
            });

        // Ожидание ответа
        auto start = std::chrono::steady_clock::now();
        while (!response_received_ && 
              std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (!response_received_) {
            RCLCPP_WARN(rclcpp::get_logger("Eth_Socket"), 
                       "No response received");
        }
        return response_received_;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Eth_Socket"), 
                    "Send error: %s", e.what());
        return false;
    }
}

bool Eth_Socket::GetWheelStates(double velocities[6], double positions[6]) {
    if(first_update_) {
        RCLCPP_INFO(rclcpp::get_logger("Eth_Socket"), 
                   "First update - returning zero states");
        for (int i = 0; i < 6; ++i) {
            velocities[i] = 0.0;
            positions[i] = 0.0;
        }
        first_update_ = false;
        return true;
    }

    if (!response_received_) {
        RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"), 
                     "No response received yet");
        return false;
    }

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
            if (!error && bytes_transferred == sizeof(Message)) {
                std::memcpy(&last_received_msg_, recv_buffer_.data(), sizeof(Message));
                response_received_ = true;
                RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"),
                            "Received %zu bytes", bytes_transferred);
            } else if (error) {
                RCLCPP_WARN(rclcpp::get_logger("Eth_Socket"),
                           "Receive error: %s", error.message().c_str());
            }
            start_receive(); // Продолжаем слушать
        });
}