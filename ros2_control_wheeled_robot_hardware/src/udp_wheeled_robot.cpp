#include "ros2_control_wheeled_robot_hardware/udp_wheeled_robot.hpp"
#include <cstdlib>
#include <cerrno>

Eth_Socket::Eth_Socket() = default;

Eth_Socket::~Eth_Socket()
{
    if (sock_ != -1) {
        close(sock_);
    }
}

bool Eth_Socket::Initialize(const std::string& ip, int port, int local_port)
{
    // Создание UDP-сокета
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ == -1) {
        std::cerr << "Socket creation failed: " << strerror(errno) << std::endl;
        return false;
    }

    // Настройка для приема данных
    memset(&cliaddr_, 0, sizeof(cliaddr_));
    cliaddr_.sin_family = AF_INET;
    cliaddr_.sin_port = htons(local_port);
    cliaddr_.sin_addr.s_addr = htonl(INADDR_ANY);

    // Привязка сокета к адресу
    if (bind(sock_, (struct sockaddr *)&cliaddr_, sizeof(cliaddr_)) < 0) {
        std::cerr << "Bind failed: " << strerror(errno) << std::endl;
        close(sock_);
        sock_ = -1;
        return false;
    }

    // Настройка для отправки данных
    memset(&servaddr_, 0, sizeof(servaddr_));
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(port);
    if (inet_aton(ip.c_str(), &servaddr_.sin_addr) == 0) {
        std::cerr << "Invalid IP address" << std::endl;
        close(sock_);
        sock_ = -1;
        return false;
    }

    // Установка таймаута на прием данных
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000;  // 500 мс
    if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        std::cerr << "Setsockopt failed: " << strerror(errno) << std::endl;
    }

    return true;
}

bool Eth_Socket::SendWheelSpeeds(const double speeds[2])
{
    if (!IsValid()) {
        return false;
    }

    struct wheel_state_message wm;
    // Преобразуем линейную и угловую скорости в скорости колес
    for (int i = 0; i < 3; ++i) {
        wm.wheel_speeds[i] = (speeds[0] - speeds[1]) / 2.0;    // Левые колеса
        wm.wheel_speeds[i+3] = (speeds[0] + speeds[1]) / 2.0;  // Правые колеса
    }
    wm.operating_mode = 0x01;

    if (sendto(sock_, &wm, sizeof(wm), 0,
        (struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0) {
        std::cerr << "Send failed: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool Eth_Socket::GetWheelStates(double wheel_speeds[6], double wheel_positions[6])
{
    if (!IsValid()) {
        return false;
    }

    struct wheel_state_message wm;
    socklen_t len = sizeof(cliaddr_);
    
    int received = recvfrom(sock_, &wm, sizeof(wm), 0,
                         (struct sockaddr *)&cliaddr_, &len);

    if (received == sizeof(wm)) {
        memcpy(wheel_speeds, wm.wheel_speeds, sizeof(wm.wheel_speeds));
        memcpy(wheel_positions, wm.wheel_positions, sizeof(wm.wheel_positions));
        return true;
    } else {
        if (received < 0) {
            std::cerr << "Receive failed: " << strerror(errno) << std::endl;
        } else {
            std::cerr << "Incomplete data received" << std::endl;
        }
        memset(wheel_speeds, 0, sizeof(double) * 6);
        memset(wheel_positions, 0, sizeof(double) * 6);
        return false;
    }
}