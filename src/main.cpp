#include <Arduino.h>
/* Tools Libraries */
#include "esp32_can.h"
#include <EBYTE.h>
#include <Ticker.h>
#include <CircularBuffer.h>
#include <TinyGPSPlus.h>
/* User Libraries */
#include "hard_defs.h"
#include "can_defs.h"
#include "MPU_defs.h"

#define GPS_DEBUG

#define MB1 // Uncomment a line if it is your car choice
//#define MB2 // Uncomment a line if it is your car choice

#ifdef MB1
  #define CAR_ID MB1_ID
#endif

#ifdef MB2
  #define CAR_ID MB2_ID
#endif

/* GPS tool */
TinyGPSPlus gps;

/* Radio LoRa tool */
EBYTE LoRa(&LoRaUART, PIN_M0, PIN_M1, PIN_AUX);

/* ESP TOOLS */
CAN_FRAME txMsg;
CircularBuffer<state_t, BUFFER_SIZE/2> state_buffer;
Ticker ticker500mHz;
Ticker ticker1Hz;

/* Debug variables */
bool buffer_full = false;
bool GPS_ok = false;
bool mode = false;
/* Global variables */
state_t current_state = IDLE_ST;
typedef struct {uint16_t day=0, month=0, year=0;} Date_acq_t; 
Date_acq_t Date_acq;
//const int Channel = 0x0F;
 
/* Interrupts routine */
void canISR(CAN_FRAME *rxMsg);
void ticker500mHzISR();
void ticker1HzISR();
/* Setup Functions */
void setupVolatilePacket();
void pinConfig();
void RadioInit();
/* Global Functions */
bool gpsInfo();

void setup() 
{
  Serial.begin(115200);
  GPS_uart.begin(GPS_Baud_Rate, SERIAL_8N1, GPS_TX, GPS_RX);
  LoRaUART.begin(LoRa_Baud_Rate);

  pinConfig(); // Setup Pin
  RadioInit(); // Start the LoRa module

  /* CAN-BUS initialize */
  CAN.setCANPins((gpio_num_t)CAN_RX_id, (gpio_num_t)CAN_TX_id);
  CAN.begin(CAN_BPS_1000K);
  CAN.watchFor();
  CAN.setCallback(0, canISR);
  txMsg.length = 8;
  txMsg.extended = 0; 
  txMsg.rtr = 0;

  setupVolatilePacket(); // volatile packet default values

  ticker500mHz.attach(2.0, ticker500mHzISR);
  ticker1Hz.attach(1.0, ticker1HzISR);
}

void loop()
{
  if(state_buffer.isFull())
  {
    //buffer_full = true;
    current_state = state_buffer.pop();
  } else {
    //buffer_full = false;
    if(!state_buffer.isEmpty())
      current_state = state_buffer.pop();
    else
      current_state = IDLE_ST;
  }

  switch(current_state)
  {
    case IDLE_ST:
     //Serial.println("i");
      break;

    case RADIO_ST:
      //Serial.println("Radio");

      LoRa.SendStruct(&volatile_packet, sizeof(volatile_packet));
      //if(LoRa.SendStruct(&volatile_packet, sizeof(volatile_packet)))
        //Serial.println("ok");

      break;

    case GPS_ST:
      //Serial.println("GPS");

      while(GPS_uart.available() > 0)
      {
        if(gps.encode(GPS_uart.read()))
          GPS_ok = gpsInfo();         
      }


      #ifdef GPS_DEBUG
        /* Send latitude message */
        txMsg.id = LAT_ID;
        txMsg.data.uint8[0] = volatile_packet.latitude;
        if(CAN.sendFrame(txMsg))
        {
          /* Send longitude message if latitude is successful */
          txMsg.id = LNG_ID;
          txMsg.data.uint8[0] = volatile_packet.longitude;
          CAN.sendFrame(txMsg);
        }
      #else
        if(GPS_ok)
        {
          /* Send latitude message */
          txMsg.id = LAT_ID;
          txMsg.data.uint8[0] = volatile_packet.latitude;
          if(CAN.sendFrame(txMsg))
          {
            /* Send longitude message if latitude is successful */
            txMsg.id = LNG_ID;
            txMsg.data.uint8[0] = volatile_packet.longitude;
            CAN.sendFrame(txMsg);
          }

          GPS_ok = false;
        }
      #endif
      
      break;

    case DEBUG_ST:
      //Serial.println("Debug state");
      //Serial.printf("Latitude (LAT) = %lf\r\n", volatile_packet.latitude);
      //Serial.printf("Longitude (LNG) = %lf\r\n", volatile_packet.longitude);
      //Serial.println("\n\n");
      break;
  }
}

/* Setup Functions */
void pinConfig()
{
  // Pins 
  pinMode(EMBEDDED_LED, OUTPUT);

  return;
}

void RadioInit()
{
  if(!LoRa.init()) 
  {
    Serial.println("LoRa ERROR!!!!");
    return;
  }

  LoRa.SetAddressH(1);
  LoRa.SetAddressL(1);
  LoRa.SetChannel(CAR_ID);
  LoRa.SetAirDataRate(ADR_1200); 
  LoRa.SetTransmitPower(OPT_TP30); 
  LoRa.SetMode(MODE_NORMAL);
  LoRa.SetUARTBaudRate(UDR_9600);
  LoRa.SetFECMode(OPT_FECENABLE);
  LoRa.SaveParameters(PERMANENT);
  //LoRa.PrintParameters();
}

void setupVolatilePacket()
{
  //volatile_packet.cont          = 0;
  volatile_packet.imu_acc.acc_x = 0;
  volatile_packet.imu_acc.acc_y = 0;
  volatile_packet.imu_acc.acc_z = 0;
  volatile_packet.imu_dps.dps_x = 0;
  volatile_packet.imu_dps.dps_y = 0;
  volatile_packet.imu_dps.dps_z = 0;
  volatile_packet.rpm           = 0;
  volatile_packet.speed         = 0;
  volatile_packet.temperature   = 0;
  volatile_packet.flags         = 0;
  volatile_packet.SOC           = 0;
  volatile_packet.cvt           = 0;
  volatile_packet.volt          = 0;
  volatile_packet.latitude      = 0; 
  volatile_packet.longitude     = 0; 
  volatile_packet.timestamp     = 0;
}

/* Global Functions */
bool gpsInfo()
{
  bool status = false;

  if(gps.location.isValid())
  {
    volatile_packet.latitude = gps.location.lat();
    volatile_packet.longitude = gps.location.lng();

    status = true;
  }

  else
  {
    volatile_packet.latitude = 0;
    volatile_packet.longitude = 0;

    status = false;
  }

  if(gps.date.isValid())
  {
    Date_acq.year = gps.date.year();
    Date_acq.month = gps.date.month();
    Date_acq.day = gps.date.day();
  }

  return status;
}

/* Interrupts routine */
void canISR(CAN_FRAME *rxMsg)
{
  mode = !mode;
  digitalWrite(EMBEDDED_LED, mode);

  volatile_packet.timestamp = millis();

  if(rxMsg->id==IMU_ACC_ID)
  {
    memcpy(&volatile_packet.imu_acc, (imu_acc_t *)rxMsg->data.uint8, sizeof(imu_acc_t));
    //Serial.printf("ACC Z = %f\r\n", (float)((volatile_packet.imu_acc.acc_z*0.061)/1000));
    //Serial.printf("ACC X = %f\r\n", (float)((volatile_packet.imu_acc.acc_x*0.061)/1000));
    //Serial.printf("ACC Y = %f\r\n", (float)((volatile_packet.imu_acc.acc_y*0.061)/1000));
  }

  if(rxMsg->id==IMU_DPS_ID)
  {
    memcpy(&volatile_packet.imu_dps, (imu_dps_t *)rxMsg->data.uint8, sizeof(imu_dps_t));
    //Serial.printf("DPS X = %d\r\n", volatile_packet.imu_dps.dps_x);
    //Serial.printf("DPS Y = %d\r\n", volatile_packet.imu_dps.dps_y);
    //Serial.printf("DPS Z = %d\r\n", volatile_packet.imu_dps.dps_z);
  }

  if(rxMsg->id==RPM_ID)
  {
    memcpy(&volatile_packet.rpm, (uint16_t *)rxMsg->data.uint8, sizeof(uint16_t));
    //Serial.printf("RPM = %d\r\n", volatile_packet.rpm);
  }

  if(rxMsg->id==SPEED_ID)
  {
    memcpy(&volatile_packet.speed, (uint16_t *)rxMsg->data.uint8, sizeof(uint16_t));
    //Serial.printf("Speed = %d\r\n", volatile_packet.speed);
  }

  if(rxMsg->id==TEMPERATURE_ID)
  {
    memcpy(&volatile_packet.temperature, (uint8_t *)rxMsg->data.uint8, sizeof(uint8_t));
    //Serial.printf("Motor = %d\r\n", volatile_packet.temperature);
  }

  if(rxMsg->id==FLAGS_ID)
  {
    memcpy(&volatile_packet.flags, (uint8_t *)rxMsg->data.uint8, sizeof(uint8_t));
    //Serial.printf("Flags = %d\r\n", volatile_packet.flags);
  }

  if(rxMsg->id==SOC_ID)
  {
    memcpy(&volatile_packet.SOC, (uint8_t *)rxMsg->data.uint8, sizeof(uint8_t));
    //Serial.printf("SOC = %d\r\n", volatile_packet.SOC);
  }

  if(rxMsg->id==CVT_ID)
  {
    memcpy(&volatile_packet.cvt, (uint8_t *)rxMsg->data.uint8, sizeof(uint8_t));
    //Serial.printf("CVT = %d\r\n", volatile_packet.cvt);
  }

  if(rxMsg->id==VOLTAGE_ID)
  {
    memcpy(&volatile_packet.volt, (float *)rxMsg->data.uint8, sizeof(float));
    //Serial.printf("Volt = %f\r\n", volatile_packet.volt);
  }  
}

void ticker500mHzISR()
{
  state_buffer.push(GPS_ST);
  //state_buffer.push(DEBUG_ST);
}

void ticker1HzISR()
{
  state_buffer.push(RADIO_ST);
} 
