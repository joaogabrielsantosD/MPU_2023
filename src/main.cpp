#include <Arduino.h>
/* Tools Libraries */
#include <EBYTE.h>
#include <Ticker.h>
#include <CircularBuffer.h>
#include <TinyGPSPlus.h>
/* User Libraries */
#include "hard_defs.h"
#include "MPU_defs.h"

#define MB1 // Uncomment a line if it is your car choice
//#define MB2 // Uncomment a line if it is your car choice

#ifdef MB1
  #define CAR_ID MB1_ID
#endif

#ifdef MB2
  #define CAR_ID MB2_ID
#endif

/*GPS TOOLS*/
TinyGPSPlus gps;

/* ESP TOOLS */
CircularBuffer<state_t, BUFFER_SIZE/2> state_buffer;
Ticker ticker400mHz;
Ticker ticker40Hz;

EBYTE LoRa(&LoRaUART, PIN_M0, PIN_M1, PIN_AUX);

/* Debug variables */
bool mode = false;
/* Global variables */
state_t current_state = IDLE_ST;
typedef struct {uint16_t day=0, month=0, year=0;} Date_acq_t; 
Date_acq_t Date_acq;
bool _radio_flag = false;
//const int Channel = 0x0F;
 
/* State Machines */
void GPS_StateMachine(void *pvParameters);
void RadioStateMachine(void *pvParameters);
/* Interrupts routine */
void ticker400mHzISR();
void ticker40HzISR();
/* Setup Functions */
void setupVolatilePacket();
void pinConfig();
void TaskSetup();
/* GPS State Machine Functions */
void RingBuffer_state(CAN_frame_t txMsg); 
void gpsinfo();
void canFilter(CAN_frame_t rxMsg);
/* Radio State Machine Functions */
void Radio_Config(int ID_Channel);

void setup() 
{
  Serial.begin(115200);
  GPS_uart.begin(GPS_Baud_Rate, SERIAL_8N1, GPS_TX, GPS_RX);
  LoRaUART.begin(LoRa_Baud_Rate);

  pinConfig(); // Setup Pin

  /* CAN-BUS initialize */
  CAN_cfg.speed     = CAN_SPEED_1000KBPS;
  CAN_cfg.tx_pin_id = CAN_TX_id;
  CAN_cfg.rx_pin_id = CAN_RX_id;
  CAN_cfg.rx_queue  = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t)); // Create a queue for data receive

  if(ESP32Can.CANInit()!=CAN_OK)
  {
    Serial.println(F("CAN ERROR!!!"));
    ESP.restart();
  }

  setupVolatilePacket(); // volatile packet default values
  TaskSetup(); // Tasks Init

  ticker400mHz.attach(2.5, ticker400mHzISR);
  ticker40Hz.attach(0.025, ticker40HzISR);
}

void loop() {/**/}

/* Setup Functions */
void setupVolatilePacket()
{
  volatile_packet.cont          = 0;
  volatile_packet.imu_acc.acc_x = 1;
  volatile_packet.imu_acc.acc_y = 2;
  volatile_packet.imu_acc.acc_z = 3;
  volatile_packet.imu_dps.dps_x = 4;
  volatile_packet.imu_dps.dps_y = 5;
  volatile_packet.imu_dps.dps_z = 6;
  volatile_packet.rpm           = 7;
  volatile_packet.speed         = 8;
  volatile_packet.temperature   = 9;
  volatile_packet.flags         = 10;
  volatile_packet.SOC           = 11;
  volatile_packet.cvt           = 12;
  volatile_packet.volt          = 13;
  volatile_packet.latitude      = 14; 
  volatile_packet.longitude     = 15; 
  volatile_packet.timestamp     = 16;
}

void pinConfig()
{
  // Pins 
  pinMode(EMBEDDED_LED, OUTPUT);

  return;
}

void TaskSetup()
{
  xTaskCreatePinnedToCore(GPS_StateMachine, "GPS/CAN_LoggingMachine", 10000, NULL, 5, NULL, 0);
  // This state machine is responsible for the Basic CAN logging and GPS logging
  xTaskCreatePinnedToCore(RadioStateMachine, "RadioConectivityStateMachine", 10000, NULL, 5, NULL, 1);
  // This state machine is responsible for the Radio communitation and possible bluetooth connection
}

/* GPS State Machine Functions */
void GPS_StateMachine(void *pvParameters)
{
  /* Create a variable to send the message */
  CAN_frame_t tx_frame; 

  /* Determinate the CAN sender type and length */
  tx_frame.FIR.B.FF = CAN_frame_std;
  tx_frame.FIR.B.DLC = 8;

  /* Create a variable to read the message */
  CAN_frame_t rx_frame;

  while(1)
  {
    RingBuffer_state(tx_frame); 

    while(GPS_uart.available() > 0)
    {
      if(gps.encode(GPS_uart.read())) 
        gpsinfo();
    }

    canFilter(rx_frame);

    vTaskDelay(1);
  }
}

void RingBuffer_state(CAN_frame_t txMsg)
{
  static bool buffer_full = false;

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
    
    case GPS_ST:
      //Serial.println("GPS");
      txMsg.data.u8[0] = volatile_packet.latitude; // 8 byte

      /* Send Latitude message */
      txMsg.MsgID = LAT_ID;

      if(ESP32Can.CANWriteFrame(&txMsg)==CAN_OK)
      {
        //Serial.print("Lat: ");
        //Serial.println(volatile_packet.latitude);

        CLEAR(txMsg.data.u8);

        /* Send Longitude data if the latitude is successful */
        txMsg.data.u8[0] = volatile_packet.longitude;
        txMsg.MsgID = LNG_ID;

        if(ESP32Can.CANWriteFrame(&txMsg)==CAN_OK)
        {
          //Serial.print("Lng: ");
          //Serial.println(volatile_packet.longitude);

          CLEAR(txMsg.data.u8);
        }
      }

      break;

    case DEBUG_ST:
      //Serial.println("d");
      //Serial.printf("\r\nSOT = %d\r\n", volatile_packet.SOT);
      //Serial.printf("\r\nLatitude = %lf\r\n", volatile_packet.latitude);
      //Serial.printf("\r\nLongitude = %lf\r\n", volatile_packet.longitude);
      break;
  }
}

void gpsinfo()
{
  //Serial.print("Location: "); 
  if(gps.location.isValid())
  {
    volatile_packet.latitude = gps.location.lat();
    volatile_packet.longitude = gps.location.lng();
    
    //Serial.print(gps.location.lat(), 6);
    //Serial.print(",");
    //Serial.print(gps.location.lng(), 6);
  }

  else 
  {
    //Serial.print(F("INVALID"));
    volatile_packet.latitude  = 0 /* Pode ser qualquer valor aqui! */;
    volatile_packet.longitude = 0 /* Pode ser qualquer valor aqui! */;
  }

  //if(gps.satellites.isValid())
  //{
  //  Serial.print(F("\tSatelites: "));
  //  Serial.println(gps.satellites.value());
  //} else {
  //  Serial.println(F("INVALID"));
  //}

  if(gps.date.isValid())
  {
    Date_acq.year = gps.date.year();
    Date_acq.month = gps.date.month();
    Date_acq.day = gps.date.day();
  }

  else
  {
    //Serial.println("DATE INVALID");
  }
  

  //Serial.println();
}

void canFilter(CAN_frame_t rxMsg)
{
  while(xQueueReceive(CAN_cfg.rx_queue, &rxMsg, 4*portTICK_PERIOD_MS)==pdTRUE)
  {
    mode = !mode; digitalWrite(EMBEDDED_LED, mode);

    /* Read the ID message */
    uint32_t messageId = rxMsg.MsgID;

    /* Debug data */
    volatile_packet.timestamp = millis();

    /* Battery management DATA */
    if(messageId == VOLTAGE_ID)
    {
      memcpy(&volatile_packet.volt, (float *)rxMsg.data.u8, sizeof(float)); 
      //Serial.printf("\r\nVoltage = %f\r\n", volatile_packet.volt);
    }

    if(messageId == SOC_ID)
    {
      memcpy(&volatile_packet.SOC, (uint8_t *)rxMsg.data.u8, sizeof(uint8_t));
      //Serial.printf("\r\nState Of Charge = %d\r\n", volatile_packet.SOC);
    }

    // if(messageId == CURRENT_ID)
    // {
    //   memcpy(&volatile_packet.current, (float *)rxMsg.data.u8, len);
    //   //Serial.printf("\r\nCurrent = %f\r\n", volatile_packet.current);
    // }

    /* Rear DATA */
    if(messageId == CVT_ID) // Old BMU
    {
      memcpy(&volatile_packet.cvt, (uint8_t *)rxMsg.data.u8, sizeof(uint8_t));
      //Serial.printf("\r\nCVT temperature = %d\r\n", volatile_packet.cvt);
    }

    // if(messageId == FUEL_ID) // Old BMU
    // {
    //   memcpy(&volatile_packet.fuel, (uint16_t *)rxMsg.data.u8, len);
    //   //Serial.printf("\r\nFuel Level = %d\r\n", volatile_packet.fuel);
    // }

    if(messageId == TEMPERATURE_ID)
    {
      mempcpy(&volatile_packet.temperature, (uint8_t *)rxMsg.data.u8, sizeof(uint8_t));
      //Serial.printf("\r\nMotor temperature = %d\r\n", volatile_packet.temperature);
    } 

    if(messageId == FLAGS_ID)
    {
      mempcpy(&volatile_packet.flags, (uint8_t *)rxMsg.data.u8, sizeof(uint8_t));
      //Serial.printf("\r\nflags = %d\r\n", volatile_packet.flags);
    }

    if(messageId == RPM_ID)
    {
      mempcpy(&volatile_packet.rpm, (uint16_t *)rxMsg.data.u8, sizeof(uint16_t));
      //Serial.printf("\r\nRPM = %d\r\n", volatile_packet.rpm);
    }
    
    /* Front DATA */
    if(messageId == SPEED_ID)
    {
      mempcpy(&volatile_packet.speed, (uint8_t *)rxMsg.data.u8, sizeof(uint8_t));
      //Serial.printf("\r\nSpeed = %d\r\n", volatile_packet.speed);
    }  

    if(messageId == IMU_ACC_ID)
    {
      memcpy(&volatile_packet.imu_acc, (imu_acc_t *)rxMsg.data.u8, sizeof(imu_acc_t));
      //Debug_accx = ((float)volatile_packet.imu_acc.acc_x*0.061)/1000.00;
      //Serial.printf("\r\nAccx = %.1f\r\n", (float)((volatile_packet.imu_acc.acc_x*0.061)/1000));
      //Serial.printf("\r\nAccy = %.1f\r\n", (float)((volatile_packet.imu_acc.acc_y*0.061)/1000));
      //Serial.printf("\r\nAccz = %.1f\r\n", (float)((volatile_packet.imu_acc.acc_z*0.061)/1000));
    }

    if(messageId == IMU_DPS_ID)
    {
      memcpy(&volatile_packet.imu_dps, (imu_dps_t *)rxMsg.data.u8, sizeof(imu_dps_t));
      //Serial.printf("\r\nDPSx = %d\r\n", volatile_packet.imu_dps.dps_x);
      //Serial.printf("\r\nDPSy = %d\r\n", volatile_packet.imu_dps.dps_y);
      //Serial.printf("\r\nDPS  = %d\r\n", volatile_packet.imu_dps.dps_z);
    }

    // if(messageId == ANGLE_ID)
    // {
    //   memcpy(&volatile_packet.Angle, (Angle_t *)rxMsg.data.u8, len);
    //   //Serial.printf("\r\nAngle Roll = %d\r\n", volatile_packet.Angle.Roll);
    //   //Serial.printf("\r\nAngle Pitch = %d\r\n", volatile_packet.Angle.Pitch);
    // }

    /* GPS/TELEMETRY DATA */
    //if(messageId == LAT_ID)
    //{ 
    //  memcpy(&volatile_packet.latitude, (double *)rxMsg.data.u8, sizeof(double));
    //  //Serial.println(volatile_packet.latitude);
    //}

    //if(messageId == LNG_ID)
    //{
    //  memcpy(&volatile_packet.longitude, (double *)rxMsg.data.u8, sizeof(double));
    //  //Serial.println(volatile_packet.longitude);
    //}

    // if(messageId == SOT_ID)
    // {
    //   memcpy(&volatile_packet.SOT, (uint8_t *)rxMsg.data.u8, len);
    //   //Serial.println(volatile_packet.SOT);
    // }
    //int t2 = micros();
    //Serial.printf("Recieve by CAN: id 0x%08X\t", rxMsg.MsgID);

    /* Print for debug */
    //if(rxMsg.MsgID==VOLTAGE_ID)
    //{
    //  for(int i = 0; i < rxMsg.FIR.B.DLC; i++)
    //  {
    //    Serial.printf("0x%02X ", rxMsg.data.u8[i]);
    //  }
    //}
  }  
}

/* Radio State Machine Functions */
void RadioStateMachine(void *pvParameters)
{
  if(!LoRa.init()) 
  {
    Serial.println("LoRa ERROR!!!!");
    return;
  }

  #ifdef MB1
    Radio_Config(MB1_ID);
  #endif

  #ifdef MB2
    Radio_Config(MB2_ID);
  #endif

  while(1)
  {
    //Serial.println("Radio state");
    if(_radio_flag)
    {
      LoRa.SendStruct(&volatile_packet, sizeof(volatile_packet));
      volatile_packet.cont++;

      _radio_flag = false;
    }

    vTaskDelay(1);
  }
}

void Radio_Config(int ID_Channel)
{
  LoRa.SetAddressH(1);
  LoRa.SetAddressL(1);
  LoRa.SetChannel(ID_Channel);
  LoRa.SetAirDataRate(ADR_1200); 
  LoRa.SetTransmitPower(OPT_TP30); 
  LoRa.SetMode(MODE_NORMAL);
  LoRa.SetUARTBaudRate(UDR_9600);
  LoRa.SetFECMode(OPT_FECENABLE);
  LoRa.SaveParameters(PERMANENT);
  //LoRa.PrintParameters();
}

/* Interrupts routine */
void ticker400mHzISR()
{
  state_buffer.push(GPS_ST);
}

void ticker40HzISR()
{
  _radio_flag = true;
}