#include <Arduino.h>
/* Tools Libraries */
#include <EBYTE.h>
#include <Ticker.h>
#include <CircularBuffer.h>
/* User Libraries */
#include "hard_defs.h"
#include "MPU_defs.h"

/* ESP TOOLS */
CircularBuffer<state_t, BUFFER_SIZE/2> state_buffer;
Ticker ticker1Hz;
EBYTE LoRa(&LoRaUART, PIN_M0, PIN_M1, PIN_AUX);

/* Debug variables */
bool mode = false;
/* Global variables */
state_t current_state = IDLE_ST;
const int Channel = 0x0F;
 
/* State Machines */
void GPS_StateMachine(void *pvParameters);
void RadioStateMachine(void *pvParameters);
/* Setup Functions */
void setupVolatilePacket();
void pinConfig();
void TaskSetup();
/* GPS State Machine Functions */
void RingBuffer_state(CAN_frame_t txMsg); 
void canFilter(CAN_frame_t rxMsg);
/* Radio State Machine Functions */

/* Interrupts routine */
void ticker1HzISR();

void setup() 
{
  Serial.begin(115200);

  pinConfig(); // Setup Pin

  /* CAN-BUS initialize */
  CAN_cfg.speed = CAN_SPEED_1000KBPS;
  CAN_cfg.tx_pin_id = CAN_TX_id;
  CAN_cfg.rx_pin_id = CAN_RX_id;
  CAN_cfg.rx_queue  = xQueueCreate(rx_queue_size, 1024); // Create a queue for data receive

  if(ESP32Can.CANInit()!=OK)
  {
    Serial.println(F("CAN ERROR!!!"));
    ESP.restart();
  }

  setupVolatilePacket(); // volatile packet default values
  TaskSetup(); // Tasks Init

  ticker1Hz.attach(1.0, ticker1HzISR);
}

void loop() {/**/}

/* Setup Functions */
void setupVolatilePacket()
{
  volatile_packet.imu_acc.acc_x = 0;
  volatile_packet.imu_acc.acc_y = 0;
  volatile_packet.imu_acc.acc_z = 0;
  volatile_packet.imu_dps.dps_x = 0;
  volatile_packet.imu_dps.dps_y = 0;
  volatile_packet.imu_dps.dps_z = 0;
  volatile_packet.Angle.Roll = 0;
  volatile_packet.Angle.Pitch = 0;
  volatile_packet.rpm = 0;
  volatile_packet.speed = 0;
  volatile_packet.temperature = 0;
  volatile_packet.flags = 0;
  volatile_packet.SOC = 0;
  volatile_packet.cvt = 0;
  volatile_packet.fuel = 0;
  volatile_packet.volt = 0;
  volatile_packet.current = 0;
  volatile_packet.latitude = -12.70814; 
  volatile_packet.longitude = -38.1732; 
  volatile_packet.timestamp = 0;
  volatile_packet.SOT = 0x00;
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
    Serial.println("GPS State");
    RingBuffer_state(tx_frame); 

    canFilter(rx_frame);
    vTaskDelay(1000);
  }
}

void RingBuffer_state(CAN_frame_t txMsg)
{
  static bool buffer_full = false;

  if(state_buffer.isFull())
  {
    buffer_full=true;
    current_state = state_buffer.pop();
  } else {
    buffer_full=false;
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
      Serial.println("GPS");
      //txMsg.data.u8[0] = volatile_packet.SOT; // 1 byte

      ///* Send State of Telemetry message */
      //txMsg.MsgID = SOT_ID;

      //if(ESP32Can.CANWriteFrame(&txMsg)==OK)
      //{
      //  CLEAR(txMsg.data.u8);
      //  //Serial.println(volatile_packet.SOT);
      //}

      break;

    case DEBUG_ST:
      //Serial.println("d");
      //Serial.printf("\r\nSOT = %d\r\n", volatile_packet.SOT);
      //Serial.printf("\r\nLatitude = %lf\r\n", volatile_packet.latitude);
      //Serial.printf("\r\nLongitude = %lf\r\n", volatile_packet.longitude);
      break;
  }
}

void canFilter(CAN_frame_t rxMsg)
{
  while(xQueueReceive(CAN_cfg.rx_queue, &rxMsg, 3*portTICK_PERIOD_MS)==pdTRUE)
  {
    mode = !mode; digitalWrite(EMBEDDED_LED, mode);

    /* Read the ID message */
    uint32_t messageId = rxMsg.MsgID;

    /* Length of the message */
    uint8_t len = 8;

    /* Debug data */
    volatile_packet.timestamp = millis();

    /* Battery management DATA */
    if(messageId == VOLTAGE_ID)
    {
      memcpy(&volatile_packet.volt, (float *)rxMsg.data.u8, len); 
      //Serial.printf("\r\nVoltage = %f\r\n", volatile_packet.volt);
    }

    if(messageId == SOC_ID)
    {
      memcpy(&volatile_packet.SOC, (uint8_t *)rxMsg.data.u8, len);
      //Serial.printf("\r\nState Of Charge = %d\r\n", volatile_packet.SOC);
    }

    if(messageId == CURRENT_ID)
    {
      memcpy(&volatile_packet.current, (float *)rxMsg.data.u8, len);
      //Serial.printf("\r\nCurrent = %f\r\n", volatile_packet.current);
    }

    /* Rear DATA */
    if(messageId == CVT_ID) // Old BMU
      {
      memcpy(&volatile_packet.cvt, (uint8_t *)rxMsg.data.u8, len);
      //Serial.printf("\r\nCVT temperature = %d\r\n", volatile_packet.cvt);
    }

    if(messageId == FUEL_ID) // Old BMU
    {
      memcpy(&volatile_packet.fuel, (uint16_t *)rxMsg.data.u8, len);
      //Serial.printf("\r\nFuel Level = %d\r\n", volatile_packet.fuel);
    }

    if(messageId == TEMPERATURE_ID)
    {
      mempcpy(&volatile_packet.temperature, (uint8_t *)rxMsg.data.u8, len);
      //Serial.printf("\r\nMotor temperature = %d\r\n", volatile_packet.temperature);
    } 

    if(messageId == FLAGS_ID)
    {
      mempcpy(&volatile_packet.flags, (uint8_t *)rxMsg.data.u8, len);
      //Serial.printf("\r\nflags = %d\r\n", volatile_packet.flags);
    }

    if(messageId == RPM_ID)
    {
      mempcpy(&volatile_packet.rpm, (uint16_t *)rxMsg.data.u8, len);
      //Serial.printf("\r\nRPM = %d\r\n", volatile_packet.rpm);
    }
    
    /* Front DATA */
    if(messageId == SPEED_ID)
    {
      mempcpy(&volatile_packet.speed, (uint16_t *)rxMsg.data.u8, len);
      //Serial.printf("\r\nSpeed = %d\r\n", volatile_packet.speed);
    }  

    if(messageId == IMU_ACC_ID)
    {
      memcpy(&volatile_packet.imu_acc, (imu_acc_t *)rxMsg.data.u8, len);
      //Debug_accx = ((float)volatile_packet.imu_acc.acc_x*0.061)/1000.00;
      //Serial.printf("\r\nAccx = %.1f\r\n", (float)((volatile_packet.imu_acc.acc_x*0.061)/1000));
      //Serial.printf("\r\nAccy = %.1f\r\n", (float)((volatile_packet.imu_acc.acc_y*0.061)/1000));
      //Serial.printf("\r\nAccz = %.1f\r\n", (float)((volatile_packet.imu_acc.acc_z*0.061)/1000));
    }

    if(messageId == IMU_DPS_ID)
    {
      memcpy(&volatile_packet.imu_dps, (imu_dps_t *)rxMsg.data.u8, len);
      //Serial.printf("\r\nDPSx = %d\r\n", volatile_packet.imu_dps.dps_x);
      //Serial.printf("\r\nDPSy = %d\r\n", volatile_packet.imu_dps.dps_y);
      //Serial.printf("\r\nDPS  = %d\r\n", volatile_packet.imu_dps.dps_z);
    }

    if(messageId == ANGLE_ID)
    {
      memcpy(&volatile_packet.Angle, (Angle_t *)rxMsg.data.u8, len);
      //Serial.printf("\r\nAngle Roll = %d\r\n", volatile_packet.Angle.Roll);
      //Serial.printf("\r\nAngle Pitch = %d\r\n", volatile_packet.Angle.Pitch);
    }

    /* GPS/TELEMETRY DATA */
    if(messageId == LAT_ID)
    {
      memcpy(&volatile_packet.latitude, (double *)rxMsg.data.u8, len);
      //Serial.println(volatile_packet.latitude);
    }

    if(messageId == LNG_ID)
    {
      memcpy(&volatile_packet.longitude, (double *)rxMsg.data.u8, len);
      //Serial.println(volatile_packet.longitude);
    }

    if(messageId == SOT_ID)
    {
      memcpy(&volatile_packet.SOT, (uint8_t *)rxMsg.data.u8, len);
      //Serial.println(volatile_packet.SOT);
    }

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
  Serial.println(LoRa.init());
  //LoRa.init(); 

  //LoRa.Reset();
  LoRa.SetAirDataRate(ADR_1K);
  LoRa.SetAddress(0x01);
  LoRa.SetChannel(Channel);
  LoRa.SaveParameters(PERMANENT);

  LoRa.PrintParameters();
  LoRa.SetMode(MODE_NORMAL);

  while(1)
  {
    Serial.println("Radio state");
    vTaskDelay(1000);
  }
}

/* Interrupts routine */
void ticker1HzISR()
{
  state_buffer.push(GPS_ST);
}