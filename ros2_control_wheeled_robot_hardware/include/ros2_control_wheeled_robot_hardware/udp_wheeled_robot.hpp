#ifndef UDP_WHEELED_ROBOT_HPP_
#define UDP_WHEELED_ROBOT_HPP_

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

// Структура для передачи данных о состояниях колес
struct wheel_state_message {
    double wheel_speeds[6];    // Скорости колес (рад/с)
    double wheel_positions[6]; // Позиции колес (рад)
    uint8_t operating_mode;    // Режим работы
};

class Eth_Socket
{
public:
    Eth_Socket();
    ~Eth_Socket();

    bool Initialize(const std::string& ip, int port, int local_port);
    bool IsValid() const { return sock_ != -1; }

    bool SendWheelSpeeds(const double speeds[2]);
    bool GetWheelStates(double speeds[6], double positions[6]);

private:
    int sock_ = -1;
    struct sockaddr_in cliaddr_;
    struct sockaddr_in servaddr_;
};

#endif  // UDP_WHEELED_ROBOT_HPP_