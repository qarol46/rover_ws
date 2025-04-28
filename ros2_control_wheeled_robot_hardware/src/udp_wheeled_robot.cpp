// udp_wheeled_robot.cpp
#include "ros2_control_wheeled_robot_hardware/udp_wheeled_robot.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <rclcpp/logging.hpp>

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
    // Клиент привязывается к порту 8888
    socket_.bind(asio::ip::udp::endpoint(asio::ip::udp::v4(), 8888));
    // Сервер на порту 8889
    server_endpoint_ = asio::ip::udp::endpoint(
      asio::ip::address::from_string(ip), 8889);

    // Set socket options
    socket_.set_option(asio::socket_base::receive_buffer_size(8192));
    socket_.set_option(asio::socket_base::reuse_address(true));

    // Start IO thread
    io_thread_ = std::thread([this]() { io_context_.run(); });
    start_receive();
    
    RCLCPP_INFO(rclcpp::get_logger("Eth_Socket"), 
               "UDP Client initialized: local port 8888, server %s:8889", 
               ip.c_str());
    
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
    // Convert to network byte order
    msg.linear_vel = htons(static_cast<int16_t>(speeds[0] * 1000));  // m/s -> mm/s
    msg.angular_vel = htons(static_cast<int16_t>(speeds[1] * 1000)); // rad/s -> mrad/s

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

    // Wait for response with increased timeout (500ms)
    auto start = std::chrono::steady_clock::now();
    while (!response_received_ && 
          std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!response_received_) {
      RCLCPP_WARN(rclcpp::get_logger("Eth_Socket"), 
                 "No response received within timeout period");
    }

    return response_received_;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("Eth_Socket"), 
                "Send error: %s", e.what());
    return false;
  }
}

bool Eth_Socket::GetWheelStates(double velocities[6], double positions[6]) {
  if (!response_received_) {
    RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"), 
                "No response received yet, can't get wheel states");
    return false;
  }

  for (int i = 0; i < 6; ++i) {
    // Convert from network byte order and scale
    velocities[i] = static_cast<double>(ntohs(last_received_msg_.velocity[i])) / 1000.0;  // mm/s -> m/s
    positions[i] = static_cast<double>(ntohs(last_received_msg_.odom[i])) / 1000.0;       // mm -> m
    RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"), 
                "Wheel %d - velocity: %.3f m/s, position: %.3f m", 
                i, velocities[i], positions[i]);
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
    RCLCPP_DEBUG(rclcpp::get_logger("Eth_Socket"), 
                "Received message with %zu bytes", bytes_transferred);
  } else if (error) {
    RCLCPP_ERROR(rclcpp::get_logger("Eth_Socket"), 
                "Receive error: %s", error.message().c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("Eth_Socket"), 
                "Received message with wrong size: %zu (expected %zu)", 
                bytes_transferred, sizeof(Message));
  }
  start_receive();
}