#ifndef SP_UDP_COMMUNICATION__UDP_MANIPULATOR_HPP_
#define SP_UDP_COMMUNICATION__UDP_MANIPULATOR_HPP_

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <memory>

namespace sp_udp_communication {

class Eth_Socket
{
public:
    Eth_Socket();
    ~Eth_Socket();

    bool move(double linear_vel, double angular_vel, 
              double vel_by_wheel_fb[6], 
              double torque_by_wheel_fb[6], 
              double odom_by_wheel_fb[6]);
    void SendCommand(double vel_command[2]);
    void GetWheelStates(double wheel_velocities[6], double wheel_positions[6]);

private:
    struct sockaddr_in servaddr_, cliaddr_;
    int sock_;
    bool flag_;

    #pragma pack(push, 1)
    struct Message
    {
        uint8_t number_device;    // 0
        uint8_t operating_mode;   // 1
        uint8_t work_device;      // 2
        uint8_t null_array_1[11]; // 3-13
        int16_t linear_vel;       // 14-15
        int16_t angular_vel;      // 16-17
        uint8_t null_array_2[46]; // 18-63
        int16_t torque[6];        // 64-75
        uint8_t __RES2[2];        // 76-77
        int16_t velocity[6];      // 78-89
        uint8_t __RES3[2];        // 90-91
        int16_t odom[6];          // 92-103
        uint8_t null_array_3[23]; // 104-126
        uint8_t sender_addres;    // 127

        Message();
    };
    #pragma pack(pop)

    Message rosm_;
    Message manr_;
};

} // namespace sp_udp_communication

#endif // SP_UDP_COMMUNICATION__UDP_MANIPULATOR_HPP_