#include "sp_udp_communication/udp_manipulator.hpp"
#include <stdexcept>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

namespace sp_udp_communication {

constexpr int CLIENT_PORT = 5001;
constexpr int SERVER_PORT = 5002;
constexpr const char* SERVER_IP = "127.0.0.1";

Eth_Socket::Message::Message()
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

Eth_Socket::Eth_Socket() : sock_(-1), flag_(false)
{
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ == -1) {
        throw std::runtime_error("Failed to create socket");
    }

    memset(&cliaddr_, 0, sizeof(cliaddr_));
    cliaddr_.sin_family = AF_INET;
    cliaddr_.sin_port = htons(CLIENT_PORT);
    cliaddr_.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock_, (struct sockaddr *)&cliaddr_, sizeof(cliaddr_)) < 0) {
        close(sock_);
        throw std::runtime_error("Failed to bind socket");
    }

    memset(&servaddr_, 0, sizeof(servaddr_));
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(SERVER_PORT);
    servaddr_.sin_addr.s_addr = inet_addr(SERVER_IP);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 500000;
    if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        close(sock_);
        throw std::runtime_error("Failed to set socket options");
    }
}

Eth_Socket::~Eth_Socket()
{
    if (sock_ != -1) {
        close(sock_);
    }
}

bool Eth_Socket::move(double linear_vel, double angular_vel, 
                     double vel_by_wheel_fb[6], 
                     double torque_by_wheel_fb[6],
                     double odom_by_wheel_fb[6])
{
    int16_t linear_vel_scaled = static_cast<int16_t>(linear_vel * 1000);
    int16_t angular_vel_scaled = static_cast<int16_t>(angular_vel * 1000);

    rosm_.linear_vel = htons(linear_vel_scaled);
    rosm_.angular_vel = htons(angular_vel_scaled);

    unsigned char w_pak[sizeof(rosm_)];
    memcpy(w_pak, &rosm_, sizeof(rosm_));

    int sendm = sendto(sock_, w_pak, sizeof(rosm_), MSG_CONFIRM, 
                      (struct sockaddr *)&servaddr_, sizeof(servaddr_));
    if (sendm < 0) {
        return false;
    }

    unsigned char r_pak[sizeof(manr_)];
    int received = recvfrom(sock_, r_pak, sizeof(manr_), MSG_WAITALL, nullptr, nullptr);
    if (received < 0) {
        return false;
    }

    memcpy(&manr_, r_pak, sizeof(manr_));

    for (int i = 0; i < 6; ++i) {
        torque_by_wheel_fb[i] = static_cast<double>(ntohs(manr_.torque[i])) / 1000.0;
        vel_by_wheel_fb[i] = static_cast<double>(ntohs(manr_.velocity[i])) / 1000.0;
        odom_by_wheel_fb[i] = static_cast<double>(ntohs(manr_.odom[i])) / 1000.0;
    }

    return true;
}

void Eth_Socket::SendCommand(double vel_command[2])
{
    double vel_fb[6], torque_fb[6], odom_fb[6];
    move(vel_command[0], vel_command[1], vel_fb, torque_fb, odom_fb);
}

void Eth_Socket::GetWheelStates(double wheel_velocities[6], double wheel_positions[6])
{
    double vel_fb[6], torque_fb[6], odom_fb[6];
    if (move(0.0, 0.0, vel_fb, torque_fb, odom_fb)) {
        for (int i = 0; i < 6; ++i) {
            wheel_velocities[i] = vel_fb[i];
            wheel_positions[i] = odom_fb[i];
        }
    }
}

} // namespace sp_udp_communication