#include "ros2_control_wheeled_robot_hardware/udp_wheeled_robot.hpp"
#include <chrono>
#include <thread>
#include <iomanip>

Eth_Socket::Eth_Socket() : socket_(io_context_) {
    RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"), "Socket created");
}

Eth_Socket::~Eth_Socket() {
    io_context_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
    socket_.close();
    RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"), "Socket destroyed");
}

bool Eth_Socket::Initialize(const std::string& ip, int server_port, int local_port) {
    try {
        socket_.open(asio::ip::udp::v4());
        socket_.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), local_port));
        server_endpoint_ = asio::ip::udp::endpoint(
            asio::ip::address::from_string(ip), server_port);

        socket_.set_option(asio::socket_base::reuse_address(true));
        io_thread_ = std::thread([this]() { 
            RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"), "IO thread started");
            io_context_.run(); 
            RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"), "IO thread stopped");
        });
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
    std::lock_guard<std::mutex> lock(send_mutex_);
    
    try {
        Message msg;
        msg.linear_vel = static_cast<int16_t>(speeds[0] * 1000);
        msg.angular_vel = static_cast<int16_t>(speeds[1] * 1000);

        RCLCPP_INFO(rclcpp::get_logger("Eth_Socket"), 
                   "Sending: lin=%.3f m/s, ang=%.3f rad/s",
                   speeds[0], speeds[1]);

        std::array<uint8_t, sizeof(Message)> send_buffer;
        std::memcpy(send_buffer.data(), &msg, sizeof(Message));

        // Логирование сырых данных
        std::stringstream raw_data;
        for (const auto& byte : send_buffer) {
            raw_data << std::hex << std::setw(2) << std::setfill('0') 
                     << static_cast<int>(byte) << " ";
        }
        RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"), 
                    "Raw send data: %s", raw_data.str().c_str());

        response_received_ = false;
        
        // Синхронная отправка
        asio::error_code ec;
        size_t bytes_sent = socket_.send_to(
            asio::buffer(send_buffer), server_endpoint_, 0, ec);
        
        if (ec) {
            RCLCPP_ERROR(rclcpp::get_logger("Eth_Socket"),
                        "Send failed: %s", ec.message().c_str());
            return false;
        }
        
        RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"),
                    "Sent %zu bytes, waiting for response...", bytes_sent);

        // Ожидание ответа с таймаутом
        auto start = std::chrono::steady_clock::now();
        while (!response_received_) {
            io_context_.run_one_for(std::chrono::milliseconds(10));
            
            if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(1000)) {
                RCLCPP_WARN(rclcpp::get_logger("Eth_Socket"),
                           "Response timeout after 1000ms");
                return false;
            }
        }

        RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"),
                    "Response received in %ld ms",
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - start).count());
        
        return true;
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

    RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"), 
                "Current states: vel[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] pos[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                velocities[0], velocities[1], velocities[2],
                velocities[3], velocities[4], velocities[5],
                positions[0], positions[1], positions[2],
                positions[3], positions[4], positions[5]);

    return true;
}

void Eth_Socket::start_receive() {
    socket_.async_receive_from(
        asio::buffer(recv_buffer_), remote_endpoint_,
        [this](const asio::error_code& error, size_t bytes_transferred) {
            if (!error) {
                if (bytes_transferred == sizeof(Message)) {
                    std::memcpy(&last_received_msg_, recv_buffer_.data(), sizeof(Message));
                    response_received_ = true;
                    RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"),
                                "Received %zu bytes from %s", 
                                bytes_transferred,
                                remote_endpoint_.address().to_string().c_str());
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("Eth_Socket"),
                               "Invalid message size: %zu (expected %zu)",
                               bytes_transferred, sizeof(Message));
                }
            } else {
                RCLCPP_WARN(rclcpp::get_logger("Eth_Socket"),
                           "Receive error: %s", error.message().c_str());
            }
            start_receive(); // Продолжаем слушать
        });
}