#include <Arduino.h>
#include <EBYTE.h>
#include <Ticker.h>
#include "test.h"

#define PIN_RX 16   // Serial2 RX (connect this to the EBYTE Tx pin)
#define PIN_TX 17   // Serial2 TX pin (connect this to the EBYTE Rx pin)
#define PIN_M0 14    
#define PIN_M1 13   
#define PIN_AX 4   

#define LoRaUART Serial2

Ticker tic;
EBYTE LoRa(&LoRaUART, PIN_M0, PIN_M1, PIN_AX);

const int Channel = 0x0F; // 15 in HEX
bool sus = false;
DATA mydata;
 
void LoRa_Config();
void ticISR();

void setup() 
{
  Serial.begin(115200);
  LoRaUART.begin(115200); 
  LoRa_Config();

  tic.attach(2,ticISR);
}

void loop() 
{
  static int count = 0;
  if(sus)
  {
    LoRa.SendStruct(&mydata, sizeof(DATA));
    Serial.printf("Sending packet number: %d\n", count);
    sus = false;
  }
}

void LoRa_Config() 
{
  Serial.println(LoRa.init());

  LoRa.SetAirDataRate(ADR_1K); 
  LoRa.SetAddress(1);
  LoRa.SetChannel(Channel);
  LoRa.SaveParameters(PERMANENT);

  LoRa.PrintParameters();
  LoRa.SetMode(MODE_NORMAL);
}

void ticISR()
{
  sus = true;
}