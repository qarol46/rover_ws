#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>

// Встроенная структура сообщения (аналог message.hpp)
#pragma pack(push, 1)
struct Message {
    uint8_t number_device;
    uint8_t operating_mode;
    uint8_t work_device;
    uint8_t null_array_1[11];
    int16_t linear_vel;
    int16_t angular_vel;
    uint8_t null_array_2[46];
    int16_t torque[6];
    uint8_t __RES2[2];
    int16_t velocity[6];
    uint8_t __RES3[2];
    int16_t odom[6];
    uint8_t null_array_3[23];
    uint8_t sender_addres;

    Message() 
        : number_device(0x23),
          operating_mode(0x01),
          work_device(0x00),
          linear_vel(0),
          angular_vel(0),
          sender_addres(0xAA) {
        memset(null_array_1, 0, sizeof(null_array_1));
        memset(null_array_2, 0, sizeof(null_array_2));
        memset(torque, 0, sizeof(torque));
        memset(velocity, 0, sizeof(velocity));
        memset(odom, 0, sizeof(odom));
        memset(null_array_3, 0, sizeof(null_array_3));
        memset(__RES2, 0, sizeof(__RES2));
        memset(__RES3, 0, sizeof(__RES3));
    }
};
#pragma pack(pop)

class UDPEchoServer : public rclcpp::Node {
public:
    UDPEchoServer() : Node("udp_echo_server"), socket_(io_context_) {
        try {
            socket_.open(asio::ip::udp::v4());
            socket_.bind(asio::ip::udp::endpoint(
                asio::ip::address::from_string("127.0.0.1"), 8889));
            
            log_file_.open("udp_server_log.txt", std::ios::out | std::ios::app);
            RCLCPP_INFO(this->get_logger(), "UDP Server started on port 8889");
            
            io_thread_ = std::thread([this]() { io_context_.run(); });
            start_receive();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Server error: %s", e.what());
            rclcpp::shutdown();
        }
    }

    ~UDPEchoServer() {
        io_context_.stop();
        if (io_thread_.joinable()) io_thread_.join();
        if (log_file_.is_open()) log_file_.close();
    }

private:
    void start_receive() {
        socket_.async_receive_from(
            asio::buffer(recv_buffer_), remote_endpoint_,
            [this](const asio::error_code& error, size_t bytes_transferred) {
                handle_receive(error, bytes_transferred);
            });
    }

    void handle_receive(const asio::error_code& error, size_t bytes_transferred) {
        if (!error && bytes_transferred == sizeof(Message)) {
            Message recv_msg;
            std::memcpy(&recv_msg, recv_buffer_.data(), sizeof(Message));

            // Логирование
            log_message("Received", recv_msg);

            // Подготовка ответа
            Message send_msg = recv_msg;
            process_wheel_data(send_msg);

            // Отправка ответа
            std::array<uint8_t, sizeof(Message)> send_buffer;
            std::memcpy(send_buffer.data(), &send_msg, sizeof(Message));
            socket_.async_send_to(
                asio::buffer(send_buffer), remote_endpoint_,
                [this](const asio::error_code& error, size_t /*bytes_sent*/) {
                    if (error) {
                        RCLCPP_ERROR(this->get_logger(), "Send error: %s", error.message().c_str());
                    }
                    start_receive();
                });
            
            log_message("Sent", send_msg);
        } else {
            start_receive();
        }
    }

    void process_wheel_data(Message& msg) {
        double linear = static_cast<double>(msg.linear_vel) / 1000.0;
        double angular = static_cast<double>(msg.angular_vel) / 1000.0;

        for (int i = 0; i < 6; i++) {
            double wheel_vel = linear + ((i < 3) ? -1 : 1) * angular * 0.4;
            msg.velocity[i] = static_cast<int16_t>(wheel_vel * 1000);
            
            static uint32_t odom_counter[6] = {0};
            odom_counter[i] += abs(msg.velocity[i]);
            msg.odom[i] = static_cast<int16_t>(odom_counter[i] % 65535);
        }
    }

    void log_message(const std::string& prefix, const Message& msg) {
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::stringstream log_line;
        log_line << "[" << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S")
                 << "." << std::setfill('0') << std::setw(3) << now_ms.count() << "] "
                 << prefix << " - Linear: " << (msg.linear_vel / 1000.0) << " m/s, "
                 << "Angular: " << (msg.angular_vel / 1000.0) << " rad/s";

        RCLCPP_INFO(this->get_logger(), "%s", log_line.str().c_str());
        if (log_file_.is_open()) {
            log_file_ << log_line.str() << "\n";
        }
    }

    asio::io_context io_context_;
    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint remote_endpoint_;
    std::array<uint8_t, sizeof(Message)> recv_buffer_;
    std::thread io_thread_;
    std::ofstream log_file_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UDPEchoServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}