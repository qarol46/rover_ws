#include <rclcpp/rclcpp.hpp>
#include <asio.hpp>
#include <iomanip>
#include <cmath>

// Константы для преобразования единиц (должны совпадать с клиентом)
constexpr float reduction = 58.64;  // Передаточное число редуктора
constexpr float wheel_radius = 0.19;  // Радиус колеса в метрах
constexpr float track = 0.8;  // Колея робота

#pragma pack(push, 1)
struct Message {
    uint8_t number_device; //0  
    uint8_t operating_mode; //1
    uint8_t work_device; //2
    uint8_t null_array_1[11]; //3-13
    int16_t linear_vel; //14-15
    int16_t angular_vel; //16-17
    uint8_t null_array_2[46]; //18-63
    int16_t torque[6]; //64-75
    uint8_t __RES2[2]; //76-77
    int16_t velocity[6]; //78-89
    uint8_t __RES3[2]; //90-91
    int16_t odom[6]; //92-103
    uint8_t null_array_3[23]; //104-126
    uint8_t sender_addres; //127

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

            // Преобразуем порядок байт для числовых полей
            recv_msg.linear_vel = ntohs(recv_msg.linear_vel);
            recv_msg.angular_vel = ntohs(recv_msg.angular_vel);

            // Логирование полученных данных (в RPM)
            float lin_rpm = recv_msg.linear_vel * 9.0f / 8.74f;
            float ang_rpm = -recv_msg.angular_vel * 9.0f / 8.74f; // Учитываем знак минус
            
            RCLCPP_INFO(this->get_logger(), 
                       "Received cmd: lin=%.1f RPM (raw=%d), ang=%.1f RPM (raw=%d)", 
                       lin_rpm, recv_msg.linear_vel,
                       ang_rpm, recv_msg.angular_vel);

            // Подготовка ответа
            Message send_msg;
            process_wheel_data(recv_msg, send_msg);

            // Преобразуем порядок байт перед отправкой
            for (int i = 0; i < 6; i++) {
                send_msg.velocity[i] = htons(send_msg.velocity[i]);
                send_msg.odom[i] = htons(send_msg.odom[i]);
            }

            // Логирование отправляемых данных
            RCLCPP_INFO(this->get_logger(), 
                       "Sending: vel=[%d, %d, %d, %d, %d, %d], odom=[%d, %d, %d, %d, %d, %d]",
                       ntohs(send_msg.velocity[0]), ntohs(send_msg.velocity[1]),
                       ntohs(send_msg.velocity[2]), ntohs(send_msg.velocity[3]),
                       ntohs(send_msg.velocity[4]), ntohs(send_msg.velocity[5]),
                       ntohs(send_msg.odom[0]), ntohs(send_msg.odom[1]),
                       ntohs(send_msg.odom[2]), ntohs(send_msg.odom[3]),
                       ntohs(send_msg.odom[4]), ntohs(send_msg.odom[5]));

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
        } else {
            if (error) {
                RCLCPP_ERROR(this->get_logger(), "Receive error: %s", error.message().c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid message size: %zu (expected %zu)",
                           bytes_transferred, sizeof(Message));
            }
            start_receive();
        }
    }

    void process_wheel_data(const Message& input, Message& output) {
        // Копируем заголовочные поля
        output.number_device = input.number_device;
        output.operating_mode = input.operating_mode;
        output.work_device = input.work_device;
        output.sender_addres = input.sender_addres;

        // Преобразуем скорости из raw значений в RPM
        float linear_rpm = input.linear_vel * 9.0f / 8.74f;
        float angular_rpm = -input.angular_vel * 9.0f / 8.74f; // Учитываем знак минус

        // Рассчитываем скорость для каждого колеса
        for (int i = 0; i < 6; i++) {
            // Объявляем переменную перед использованием
            float wheel_rpm;
        
            // Моделируем скорость колеса (линейная + угловая компоненты)
            if(i % 2 == 0) {
                wheel_rpm = linear_rpm - angular_rpm * 0.5f;
            } else {
                wheel_rpm = -(linear_rpm + angular_rpm * 0.5f);
            }
        
            // Преобразуем RPM в raw значение (как в клиенте)
            output.velocity[i] = static_cast<int16_t>(wheel_rpm * 8.74f * 9.548f * reduction);
        
            // Обновляем одометрию (имитация энкодера)
            static int32_t odom_counter[6] = {0};
            odom_counter[i] += static_cast<int32_t>(output.velocity[i] * 0.1f); // Интегрируем скорость
            output.odom[i] = static_cast<int16_t>(odom_counter[i] / 100); // Масштабируем
        }
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