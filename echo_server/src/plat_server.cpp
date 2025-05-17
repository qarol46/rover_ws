#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <iomanip>

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

            // Логирование полученных данных
            RCLCPP_INFO(this->get_logger(), "Received cmd: lin=%.3f m/s, ang=%.3f rad/s", 
                       static_cast<double>(recv_msg.linear_vel) / 1000.0,
                       static_cast<double>(recv_msg.angular_vel) / 1000.0);
            
            // Лог сырых данных
            //log_raw_data("Received raw", recv_buffer_);

            // Подготовка ответа с пересчитанными данными
            Message send_msg;
            process_wheel_data(recv_msg, send_msg);

            // Логирование отправляемых данных
            RCLCPP_INFO(this->get_logger(), "Sending: velocities=[%d, %d, %d, %d, %d, %d], odom=[%d, %d, %d, %d, %d, %d]",
                       send_msg.velocity[0], send_msg.velocity[1], send_msg.velocity[2],
                       send_msg.velocity[3], send_msg.velocity[4], send_msg.velocity[5],
                       send_msg.odom[0], send_msg.odom[1], send_msg.odom[2],
                       send_msg.odom[3], send_msg.odom[4], send_msg.odom[5]);
            
            // Лог сырых данных
            std::array<uint8_t, sizeof(Message)> send_buffer;
            std::memcpy(send_buffer.data(), &send_msg, sizeof(Message));
            //log_raw_data("Sending raw", send_buffer);

            // Отправка ответа
            socket_.async_send_to(
                asio::buffer(send_buffer), remote_endpoint_,
                [this](const asio::error_code& error, size_t /*bytes_sent*/) {
                    if (error) {
                        RCLCPP_ERROR(this->get_logger(), "Send error: %s", error.message().c_str());
                    }
                    start_receive();
                });
        } else {
            start_receive();
        }
    }

    void process_wheel_data(const Message& input, Message& output) {
        // Копируем заголовочные поля
        output.number_device = input.number_device;
        output.operating_mode = input.operating_mode;
        output.work_device = input.work_device;
        output.sender_addres = input.sender_addres;

        double linear = static_cast<double>(input.linear_vel) / 1000.0;
        double angular = static_cast<double>(input.angular_vel) / 1000.0;

        for (int i = 0; i < 6; i++) {
            // Рассчитываем скорость для каждого колеса
            double wheel_vel = linear + ((i < 3) ? -1 : 1) * angular * 0.4;
            output.velocity[i] = static_cast<int16_t>(wheel_vel * 1000);
            
            // Обновляем одометрию
            static uint32_t odom_counter[6] = {0};
            odom_counter[i] += abs(output.velocity[i]);
            output.odom[i] = static_cast<int16_t>(odom_counter[i] % 65535);
        }
    }

    void log_raw_data(const std::string& prefix, const std::array<uint8_t, sizeof(Message)>& buffer) {
        std::stringstream ss;
        ss << prefix << " data: ";
        for (size_t i = 0; i < buffer.size(); ++i) {
            ss << std::hex << std::setw(2) << std::setfill('0') 
               << static_cast<int>(buffer[i]) << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    asio::io_context io_context_;
    asio::ip::udp::socket socket_;
    asio::ip::udp::endpoint remote_endpoint_;
    std::array<uint8_t, sizeof(Message)> recv_buffer_;
    std::thread io_thread_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UDPEchoServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}