#ifndef PACKETS_H_
#define PACKETS_H_

#include <stdio.h>
#include <string.h>

#define CLEAR(x) memset(&x, 0x00, 8)

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
    /* REAR DATAS */
    float volt;
    uint8_t SOC;
    uint8_t cvt;
    uint8_t temperature;
    /* FRONT DATAS */
    uint8_t flags; // MSB - BOX | BUFFER FULL | NC | NC | FUEL_LEVEL | SERVO_ERROR | CHK | RUN - LSB
    imu_dps_t imu_dps;
    imu_acc_t imu_acc;
    uint16_t rpm;
    uint16_t speed;
    /* GPS DATAS */
    double latitude;
    double longitude;
    /* DEBUG DATA */
    uint32_t timestamp;
} radio_packet_t;

// Packet constantly saved
radio_packet_t volatile_packet;

#endif