#ifndef MPU_DEFS_H
#define MPU_DEFS_H

#include <Arduino.h>
#include "packets.h"
#include "can_defs.h"

/* State Machines */
typedef enum {
    IDLE_ST, 
    RADIO_ST, 
    GPS_ST, 
    DEBUG_ST
} state_t;

typedef struct {uint8_t hour=0, minute=0, second=0;} Time_acq_t;
typedef struct {uint16_t day=0, month=0, year=0;} Date_acq_t;

//typedef enum {SENDING, LISTENING} connectivity_states;

unsigned long timer;

#endif