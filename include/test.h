#ifndef test_h
#define test_h

#include <Arduino.h>

typedef struct
{
    bool s = true;

    char as[2] = {'o','i'};
    String p = "Pai ta on";

    uint8_t e = 253;
    uint16_t r = 6000;
    uint32_t y = 245462;
    uint64_t sd = 132345567;

    float c = 1.34;
    double v = 1.33423;
} DATA;

#endif