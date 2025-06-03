#include "ros2_control_wheeled_robot_hardware/udp_wheeled_robot.hpp"
#include "ros2_control_wheeled_robot_hardware/message.hpp" // Подключаем заголовочный файл с структурой Message
#include <chrono>
#include <thread>
#include <iomanip>
#include <cmath>
#include <arpa/inet.h> // Для htons/ntohs

// Константы из trk211_ethernet.cpp
constexpr float reduction = 58.64;  // Передаточное число редуктора
constexpr float wheel_radius = 0.19;  // Радиус колеса в метрах
constexpr float track = 0.8;  // Колея робота

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
        float linear_vel = speeds[0];  // m/s
        float angular_vel = speeds[1]; // rad/s

        // Преобразование скоростей в RPM
        linear_vel = 60.0f * linear_vel * reduction / (wheel_radius * 2.0f * M_PI);
        angular_vel = 60.0f * angular_vel * reduction * (track/2.0f) / (wheel_radius * M_PI);

        // Ограничение максимальной скорости (2400 RPM)
        float max_rpm = 2400.0f;
        if (fabs(linear_vel) + fabs(angular_vel) > max_rpm) {
            float scale = max_rpm / (fabs(linear_vel) + fabs(angular_vel));
            linear_vel *= scale;
            angular_vel *= scale;
        }

        // Применяем масштабирование и корректируем знак для обратного направления
        linear_vel = linear_vel * 8.74f / 9.0f;
        angular_vel = angular_vel * 8.74f / 9.0f;

        Message msg;
        msg.linear_vel = htons(static_cast<int16_t>(linear_vel));
        msg.angular_vel = htons(static_cast<int16_t>(angular_vel));

        // Отправка данных
        response_received_ = false;
        asio::error_code ec;
        socket_.send_to(asio::buffer(&msg, sizeof(msg)), server_endpoint_, 0, ec);
        
        // Ожидание ответа
        auto start = std::chrono::steady_clock::now();
        while (!response_received_) {
            io_context_.run_one_for(std::chrono::milliseconds(10));
            if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(1000)) {
                return false;
            }
        }
        return true;
    } catch (...) {
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
        // Velocity from rpm to rad/s
        velocities[i] = (1.0f / (8.74f * reduction * 9.548f)) * 
                       static_cast<double>(last_received_msg_.velocity[i]);
        
        // Position from encoder ticks to radians
        positions[i] = (2.0 * M_PI / (8.0 * reduction)) * 
                      static_cast<double>(last_received_msg_.odom[i]);
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