#ifndef PACKETS_H
#define PACKETS_H

#include <stdio.h>
#include <string.h>

#define MB1_ID  11
#define MB2_ID  22

typedef struct
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
} imu_acc_t;

typedef struct
{
    int16_t dps_x;
    int16_t dps_y;
    int16_t dps_z;
} imu_dps_t;

typedef struct
{
    //int cont;
    /* Mangue Telemetry Struct */
    imu_acc_t imu_acc;
    imu_dps_t imu_dps;
    uint16_t rpm;
    uint16_t speed;
    uint8_t temperature;
    uint8_t flags; // MSB - BOX | BUFFER FULL | NC | NC | FUEL_LEVEL | SERVO_ERROR | CHK | RUN - LSB
    uint8_t SOC;
    uint8_t cvt;
    float volt;
    double latitude;
    double longitude;
    //uint16_t fuel_level;
    uint32_t timestamp;
    uint8_t sat;
} radio_packet_t;

// Packet constantly saved
radio_packet_t volatile_packet;

#endif