#ifndef HARD_DEFS_H
#define HARD_DEFS_H

/*============================*/
    #include <Arduino.h>
    #include <driver/gpio.h>
/*============================*/

#define EMBEDDED_LED LED_BUILTIN

/* LoRa definitions */
#define PIN_M0  GPIO_NUM_14
#define PIN_M1  GPIO_NUM_13
#define PIN_AUX GPIO_NUM_4
#define LORA_RX GPIO_NUM_16   // Serial2 RX (connect this to the EBYTE Tx pin)
#define LORA_TX GPIO_NUM_17   // Serial2 TX pin (connect this to the EBYTE Rx pin)
#define LoRaUART Serial2

/* CAN definitions */
#define CAN_TX_id GPIO_NUM_12
#define CAN_RX_id GPIO_NUM_15

#endif