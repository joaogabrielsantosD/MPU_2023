#include <Arduino.h>
#include <EBYTE.h>
#include "test.h"

#define PIN_RX 16   // Serial2 RX (connect this to the EBYTE Tx pin)
#define PIN_TX 17   // Serial2 TX pin (connect this to the EBYTE Rx pin)
#define PIN_M0 14    
#define PIN_M1 13   
#define PIN_AX 4   

#define LoRaUART Serial2

EBYTE LoRa(&LoRaUART, PIN_M0, PIN_M1, PIN_AX);

const int Channel = 0x0F; // 15 in HEX
DATA mydata;
 
void LoRa_Config();
void ticISR();

void setup() 
{
  Serial.begin(115200);
  LoRaUART.begin(115200); 

  LoRa_Config();
}

void loop() 
{
  uint64_t last = 0;
  static int count = 0;
  if(LoRaUART.available())
  {
    LoRa.GetStruct(&mydata, sizeof(DATA));
    last = millis();
    count++;

    Serial.println("==========================================================");
    Serial.println("Bool e Characteres:");
    Serial.printf("\t%d\n", mydata.s);
    Serial.printf("\t%c%c\n", mydata.as[0], mydata.as[1]);
    Serial.printf("\t%s\n\n", mydata.p);

    Serial.println("Inteiros:");
    Serial.printf("\t%d\n", mydata.e);
    Serial.printf("\t%d\n", mydata.r);
    Serial.printf("\t%d\n", mydata.y);
    Serial.printf("\t%d\n\n", mydata.sd);

    Serial.println("Flutuantes:");
    Serial.printf("\t%f\n", mydata.c);
    Serial.printf("\t%lf\n", mydata.v);
    Serial.println("==========================================================");

    Serial.printf("Ultima aquisição do pacote %d foi em %d\n\n", count, last);
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