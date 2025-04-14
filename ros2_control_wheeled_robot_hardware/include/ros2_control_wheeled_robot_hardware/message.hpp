#pragma once
#include <cstdint>
#include <cstring>

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

    // Конструктор с инициализацией
    Message() : 
        number_device(0x23),
        operating_mode(0x01),
        work_device(0x00),
        linear_vel(0),
        angular_vel(0),
        sender_addres(0xAA) 
    {
        // Обнуляем массивы
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