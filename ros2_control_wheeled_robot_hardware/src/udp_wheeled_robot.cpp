#include "ros2_control_wheeled_robot_hardware/udp_wheeled_robot.hpp"
#include <chrono>
#include <thread>
#include <iostream>

Eth_Socket::Eth_Socket() : socket_(io_context_) {}

Eth_Socket::~Eth_Socket() {
    io_context_.stop();
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
    socket_.close();
}

bool Eth_Socket::Initialize(const std::string& ip, int port, int local_port) {
    try {
        socket_.open(asio::ip::udp::v4());
        socket_.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), local_port));
        server_endpoint_ = asio::ip::udp::endpoint(
            asio::ip::address::from_string(ip), port);

        // Установка таймаута
        socket_.set_option(asio::socket_base::receive_buffer_size(8192));
        socket_.set_option(asio::socket_base::reuse_address(true));

        io_thread_ = std::thread([this]() { io_context_.run(); });
        start_receive();
        return true;
    } catch (const std::exception& e) {
        std::cerr << "UDP initialization error: " << e.what() << std::endl;
        return false;
    }
}
bool Eth_Socket::SendWheelSpeeds(const double speeds[2]) {
    try {
        Message msg;
        msg.linear_vel = static_cast<int16_t>(speeds[0] * 1000);  // м/с -> мм/с
        msg.angular_vel = static_cast<int16_t>(speeds[1] * 1000); // рад/с -> мрад/с

        std::array<uint8_t, sizeof(Message)> send_buffer;
        std::memcpy(send_buffer.data(), &msg, sizeof(Message));

        response_received_ = false;
        socket_.async_send_to(
            asio::buffer(send_buffer), server_endpoint_,
            [this](const asio::error_code& error, size_t /*bytes_sent*/) {
                if (error) {
                    std::cerr << "Send error: " << error.message() << std::endl;
                }
            });

        // Ждем ответа с таймаутом
        auto start = std::chrono::steady_clock::now();
        while (!response_received_ && 
               std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        return response_received_;
    } catch (const std::exception& e) {
        std::cerr << "Send error: " << e.what() << std::endl;
        return false;
    }
}

bool Eth_Socket::GetWheelStates(double speeds[6], double positions[6]) {
    if (!response_received_) {
        return false;
    }

    for (int i = 0; i < 6; ++i) {
        speeds[i] = last_received_msg_.velocity[i] / 1000.0;  // мм/с -> м/с
        positions[i] = last_received_msg_.odom[i] / 1000.0;   // мм -> м
    }
    return true;
}

void Eth_Socket::start_receive() {
    socket_.async_receive_from(
        asio::buffer(recv_buffer_), remote_endpoint_,
        [this](const asio::error_code& error, size_t bytes_transferred) {
            this->handle_receive(error, bytes_transferred);
        });
}

void Eth_Socket::handle_receive(const asio::error_code& error, size_t bytes_transferred) {
    if (!error && bytes_transferred == sizeof(Message)) {
        std::memcpy(&last_received_msg_, recv_buffer_.data(), sizeof(Message));
        response_received_ = true;
    } else if (error) {
        std::cerr << "Receive error: " << error.message() << std::endl;
    }
    start_receive();
}