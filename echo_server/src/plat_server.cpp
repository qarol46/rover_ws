#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <fstream>     // Добавлено для работы с файлами
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>      // Для временных меток
#include <iomanip>     // Для форматирования вывода

// Определяем структуру сообщения
#pragma pack(push, 1)
struct message
{
    uint8_t number_device;    // 0
    uint8_t operating_mode;   // 1
    uint8_t work_device;      // 2
    uint8_t null_array_1[11]; // 3-13
    int16_t linear_vel;      // 14-15
    int16_t angular_vel;     // 16-17
    uint8_t null_array_2[46]; // 18-63
    int16_t torque[6];        // 64-75
    uint8_t __RES2[2];        // 76-77
    int16_t velocity[6];      // 78-89
    uint8_t __RES3[2];        // 90-91
    uint16_t odom[6];          // 92-103
    uint8_t null_array_3[23]; // 104-126
    uint8_t sender_addres;    // 127

    message()
        : number_device(0x23),
          operating_mode(0x01),
          work_device(0x00),
          linear_vel(0),
          angular_vel(0),
          sender_addres(0xAA)
    {
        std::memset(null_array_1, 0, sizeof(null_array_1));
        std::memset(null_array_2, 0, sizeof(null_array_2));
        std::memset(torque, 0, sizeof(torque));
        std::memset(__RES2, 0, sizeof(__RES2));
        std::memset(velocity, 0, sizeof(velocity));
        std::memset(__RES3, 0, sizeof(__RES3));
        std::memset(odom, 0, sizeof(odom));
        std::memset(null_array_3, 0, sizeof(null_array_3));
    }
};
#pragma pack(pop)

class UDPEchoServer : public rclcpp::Node
{
public:
    UDPEchoServer() : Node("udp_echo_server")
    {
        // Создание UDP сокета
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            rclcpp::shutdown();
        }

        // Настройка адреса сервера
        std::memset(&servaddr_, 0, sizeof(servaddr_));
        servaddr_.sin_family = AF_INET;
        servaddr_.sin_addr.s_addr = inet_addr("127.0.0.1");
        servaddr_.sin_port = htons(8889);

        // Привязка адреса сервера к сокету
        if (bind(socket_fd_, (const struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Bind failed");
            close(socket_fd_);
            rclcpp::shutdown();
        }

        // Открываем файл для логирования
        log_file_.open("udp_server_log.txt", std::ios::out | std::ios::app);
        if (!log_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open log file");
        } else {
            RCLCPP_INFO(this->get_logger(), "Logging to udp_server_log.txt");
        }

        RCLCPP_INFO(this->get_logger(), "UDP Echo Server is up and running");
        start_server();
    }

    ~UDPEchoServer()
    {
        // Закрываем файл логов
        if (log_file_.is_open()) {
            log_file_.close();
        }

        close(socket_fd_);
    }

private:
    void start_server()
    {
        socklen_t len;
        struct sockaddr_in cliaddr;
        unsigned char buffer[sizeof(message)]; // Буфер для приема и отправки данных

        while (rclcpp::ok()) {
            len = sizeof(cliaddr);
            ssize_t n = recvfrom(socket_fd_, buffer, sizeof(buffer), 0, (struct sockaddr *)&cliaddr, &len);
            if (n < 0) {
                RCLCPP_ERROR(this->get_logger(), "Receive failed");
                continue;
            }

            // Получаем текущее время для временной метки
            auto now = std::chrono::system_clock::now();
            std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
            auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

            // Форматируем временную метку
            std::stringstream timestamp;
            timestamp << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S")
                      << '.' << std::setfill('0') << std::setw(3) << now_ms.count();

            // Логируем полученные данные
            RCLCPP_INFO(this->get_logger(), "Сервер получил данные (размер %ld байт):", n);

            // Логирование в файл
            if (log_file_.is_open()) {
                log_file_ << "[" << timestamp.str() << "] Received data (" << n << " bytes): ";
                for (int i = 0; i < n; ++i) {
                    log_file_ << std::hex << std::setw(2) << std::setfill('0') << (int)buffer[i] << " ";
                }
                log_file_ << std::dec << "\n";
            }

            // Распаковка полученного сообщения
            message recv_msg;
            memcpy(&recv_msg, buffer, sizeof(message));

            int16_t linear_vel_scaled = ntohs(recv_msg.linear_vel);
            int16_t angular_vel_scaled = ntohs(recv_msg.angular_vel);

            double linear_velocity = static_cast<double>(linear_vel_scaled) / 1000.0;
            double angular_velocity = static_cast<double>(angular_vel_scaled) / 1000.0;

            // Логирование полученных скоростей
            RCLCPP_INFO(this->get_logger(), "Получена линейная скорость: %f м/с", linear_velocity);
            RCLCPP_INFO(this->get_logger(), "Получена угловая скорость: %f рад/с", angular_velocity);

            // Логирование в файл
            if (log_file_.is_open()) {
                log_file_ << "[" << timestamp.str() << "] Linear velocity: " << linear_velocity << " m/s, "
                          << "Angular velocity: " << angular_velocity << " rad/s\n";
            }

            // Эмуляция данных обратной связи
            message send_msg = recv_msg; // Начинаем с копии полученного сообщения

            // Эмулируем некоторые данные для колес
            for (int i = 0; i < 6; ++i) {
                // Например, скорость каждого колеса пропорциональна линейной скорости
                double wheel_velocity = linear_velocity + ((i < 3) ? -1 : 1) * angular_velocity * 0.4; // Простая модель

                // Масштабируем и преобразуем в int16_t
                int16_t wheel_velocity_scaled = static_cast<int16_t>(wheel_velocity * 1000.0);

                // Устанавливаем значения в сообщении (преобразуем в сетевой порядок байт)
                send_msg.velocity[i] = htons(wheel_velocity_scaled);

                // Эмулируем одометрию (накапливаем значения)
                static uint32_t odom_counter[6] = {0};
                odom_counter[i] += wheel_velocity_scaled; // Простейшая модель одометрии
                send_msg.odom[i] = htons(odom_counter[i]);

                // Эмулируем момент (например, пропорционален скорости)
                int16_t torque = static_cast<int16_t>(wheel_velocity * 10.0); // Пример
                send_msg.torque[i] = htons(torque);

                // Логирование данных колес в файл
                if (log_file_.is_open()) {
                    log_file_ << "[" << timestamp.str() << "] Wheel " << i << " - Velocity: " << wheel_velocity
                              << " m/s, Odom: " << odom_counter[i] << ", Torque: " << torque << "\n";
                }
                RCLCPP_INFO(this->get_logger(), "Wheel %d - Velocity: %f m/s, Odom: %d, Torque: %d", i, wheel_velocity, odom_counter[i], torque);
            }

            // Копируем структуру в буфер для отправки
            memcpy(buffer, &send_msg, sizeof(message));

            // Отправка обратно клиенту
            ssize_t sent = sendto(socket_fd_, buffer, sizeof(message), 0, (struct sockaddr *)&cliaddr, len);
            if (sent < 0) {
                RCLCPP_ERROR(this->get_logger(), "Send failed");
            } else {
                RCLCPP_INFO(this->get_logger(), "Сервер отправил ответ.");

                // Логирование отправленных данных в файл
                if (log_file_.is_open()) {
                    log_file_ << "[" << timestamp.str() << "] Sent data (" << sent << " bytes)\n";
                    log_file_ << "[" << timestamp.str() << "] ----------------------------------------\n";
                }
            }
        }
    }

    int socket_fd_;
    struct sockaddr_in servaddr_;
    std::ofstream log_file_; // Файловый поток для логирования
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UDPEchoServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}