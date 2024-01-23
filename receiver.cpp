#include <Arduino.h>
#include <Ticker.h>
#include <SD.h>
#include <EBYTE.h>
#include <TinyGPS++.h>
#include "hard_defs_rec.h"
#include "packets.h"

#define MB1 // Uncomment a line if it is your car choice
// #define MB2 // Uncomment a line if it is your car choice

#ifdef MB1
  #define CAR_ID MB1_ID
#endif

#ifdef MB2
  #define CAR_ID MB2_ID
#endif

// Ticker ticker1Hz;

// void SdStateMachine(void *pvParameters);
// void RadioStateMachine(void *pvParameters);

// String packetToString(bool err = true);
// void sdConfig();
// void sdSave();
// int countFiles(File dir);
// void TaskSetup();
// void ticker1HzISR();

unsigned long Last;

EBYTE Lora(&LoRaUART, PIN_M0, PIN_M1, PIN_AUX);

// SD variables
char file_name[20];
File root;
File dataFile;
bool saveFlag                = 0x00;
uint16_t contador            = 0;
uint16_t counter_acc_x       = 0;
uint16_t counter_acc_y       = 0;
uint16_t counter_acc_z       = 0;
uint16_t counter_dps_x       = 0;
uint16_t counter_dps_y       = 0;
uint16_t counter_dps_z       = 0;
uint16_t counter_rpm         = 0;
uint16_t counter_speed       = 0;
uint16_t counter_temperature = 0;
uint16_t counter_flags       = 0;
uint16_t counter_SOC         = 0;
uint16_t counter_cvt         = 0;
uint16_t counter_voltage     = 0;
uint16_t counter_latitude    = 0;
uint16_t counter_longitude   = 0;
uint16_t counter_timestamp   = 0;
uint16_t Loss;

void setup()
{
  Serial.begin(115200);
  LoRaUART.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  if (Lora.init())
  {
    Serial.println("LoRa OK.");
  }

  Lora.SetAddressH(1);
  Lora.SetAddressL(1);
  Lora.SetChannel(MB1_ID);
  Lora.SetAirDataRate(ADR_1200);
  Lora.SetTransmitPower(OPT_TP30);
  Lora.SetMode(MODE_NORMAL);
  Lora.SetUARTBaudRate(UDR_9600);
  Lora.SetFECMode(OPT_FECENABLE);
  Lora.SaveParameters(PERMANENT);
  Lora.PrintParameters();
  // TaskSetup(); // Tasks Init
  //  ticker1Hz.attach(1, ticker1HzISR);
}

void loop()
{
  if (LoRaUART.available())
  {
    Lora.GetStruct(&volatile_packet, sizeof(volatile_packet));
    saveFlag = 0x01;

    digitalWrite(LED_BUILTIN, HIGH);

    Loss = volatile_packet.cont;

    Serial.println("------------");
    Serial.print("Acc X: ");          Serial.println(volatile_packet.imu_acc.acc_x);
    Serial.print("Acc Y: ");          Serial.println(volatile_packet.imu_acc.acc_y);
    Serial.print("Acc Z: ");          Serial.println(volatile_packet.imu_acc.acc_z);
    Serial.print("DPS X: ");          Serial.println(volatile_packet.imu_dps.dps_x);
    Serial.print("DPS Y: ");          Serial.println(volatile_packet.imu_dps.dps_y);
    Serial.print("DPS Z: ");          Serial.println(volatile_packet.imu_dps.dps_z);
    Serial.print("RPM: ");            Serial.println(volatile_packet.rpm);
    Serial.print("SPEED: ");          Serial.println(volatile_packet.speed);
    Serial.print("TEMPERATURE: ");    Serial.println(volatile_packet.temperature);
    Serial.print("FLAGS: ");          Serial.println(volatile_packet.flags);
    Serial.print("SOC: ");            Serial.println(volatile_packet.SOC);
    Serial.print("CVT: ");            Serial.println(volatile_packet.cvt);
    Serial.print("VOLT: ");           Serial.println(volatile_packet.volt);
    Serial.print("LATITUDE: ");       Serial.println(volatile_packet.latitude);
    Serial.print("LONGITUDE: ");      Serial.println(volatile_packet.longitude);
    Serial.print("Times: ");          Serial.println(volatile_packet.timestamp);
    Serial.print("Cont. Transm: ");   Serial.println(volatile_packet.cont);
    Serial.print("Cont. Rec: ");      Serial.println(contador);

    contador++;

    Serial.println("------------");
    if (volatile_packet.imu_acc.acc_x != 1)
    {
      counter_acc_x++;
      Serial.print("acc_x: ");        Serial.println(counter_acc_x);
    }

    if (volatile_packet.imu_acc.acc_y != 2)
    {
      counter_acc_y++;
      Serial.print("acc_y: ");        Serial.println(counter_acc_y);
    }

    if (volatile_packet.imu_acc.acc_z != 3)
    {
      counter_acc_z++;
      Serial.print("acc_z: ");        Serial.println(counter_acc_z);
    }

    if (volatile_packet.imu_dps.dps_x != 4)
    {
      counter_dps_x++;
      Serial.print("dps_x: ");        Serial.println(counter_dps_x);
    }

    if (volatile_packet.imu_dps.dps_y != 5)
    {
      counter_dps_y++;
      Serial.print("dps_y: ");        Serial.println(counter_dps_y);
    }

    if (volatile_packet.imu_dps.dps_z != 6)
    {
      counter_dps_z++;
      Serial.print("dps_z: ");        Serial.println(counter_dps_z);
    }

    if (volatile_packet.rpm != 7)
    {
      counter_rpm++;
      Serial.print("rpm: ");          Serial.println(counter_rpm);
    }

    if (volatile_packet.speed != 8)
    {
      counter_speed++;
      Serial.print("speed: ");        Serial.println(counter_speed);
    }

    if (volatile_packet.temperature != 9)
    {
      counter_temperature++;
      Serial.print("temperature: ");  Serial.println(counter_temperature);
    }

    if (volatile_packet.flags != 10)
    {
      counter_flags++;
      Serial.print("flags: ");        Serial.println(counter_flags);
    }

    if (volatile_packet.SOC != 11)
    {
      counter_SOC++;
      Serial.print("SOC: ");          Serial.println(counter_SOC);
    }

    if (volatile_packet.cvt != 12)
    {
      counter_cvt++;
      Serial.print("cvt: ");          Serial.println(counter_cvt);
    }

    if (volatile_packet.volt != 13)
    {
      counter_voltage++;
      Serial.print("volt: ");         Serial.println(counter_voltage);
    }

    if (volatile_packet.latitude != 14)
    {
      counter_latitude++;
      Serial.print("latitude: ");     Serial.println(counter_latitude);
    }

    if (volatile_packet.longitude != 15)
    {
      counter_longitude++;
      Serial.print("longitude: ");    Serial.println(counter_longitude);
    }

    if (volatile_packet.timestamp != 16)
    {
      counter_timestamp++;
      Serial.print("timestamp: ");    Serial.println(counter_timestamp);
    }

    digitalWrite(LED_BUILTIN, LOW);
    Last = millis();
  }
  else
  {
    if ((millis() - Last) > 1000)
    {
      Serial.println("\n------------");
      Serial.print("Searching: ");
      Last = millis();
      saveFlag ? Loss++ : 0;
      Serial.println(Loss);
      Serial.print("Cont. Rec: ");
      Serial.println(contador);
      Serial.println("------------");
    }
  }
}

// void TaskSetup()
// {
//   xTaskCreatePinnedToCore(SdStateMachine, "SDStateMachine", 10000, NULL, 5, NULL, 0);
//   // This state machine is responsible for the Basic CAN logging and GPS logging
//   xTaskCreatePinnedToCore(RadioStateMachine, "RadioConectivityStateMachine", 10000, NULL, 5, NULL, 1);
//   // This state machine is responsible for the Radio communitation and possible bluetooth connection
// }

// //SD Functions
// void sdConfig()
// {
//   static bool mounted = false; // SD mounted flag

//   if(!mounted)
//   {
//     if(!SD.begin(SD_CS)) { return; }

//     root = SD.open("/");
//     int num_files = countFiles(root);
//     sprintf(file_name, "/%s%d.csv", "data", num_files + 1);

//     dataFile = SD.open(file_name, FILE_APPEND);

//     if(dataFile)
//     {
//       dataFile.println(packetToString(mounted));
//       dataFile.close();
//     } else {
//       Serial.println(F("FAIL TO OPEN THE FILE"));
//     }
//     mounted = true;
//   }
//   sdSave();
// }

// int countFiles(File dir)
// {
//   int fileCountOnSD = 0; // for counting files
//   for(;;)
//   {
//     File entry = dir.openNextFile();
//     if (!entry)
//     {
//       // no more files
//       break;
//     }
//     // for each file count it
//     fileCountOnSD++;
//     entry.close();
//   }

//   return fileCountOnSD - 1;
// }

// void sdSave()
// {
//   dataFile = SD.open(file_name, FILE_APPEND);

//   if(dataFile)
//   {
//     dataFile.println(packetToString());
//     dataFile.close();
//   } else {
//     Serial.println(F("falha no save"));
//   }
// }

// String packetToString(bool err)
// {
//   String dataString = "";
//     if(!err)
//     {
//       dataString += "Cont";
//       dataString += ",";
//       dataString += "ACCX";
//       dataString += ",";
//       dataString += "ACCY";
//       dataString += ",";
//       dataString += "ACCZ";
//       dataString += ",";
//       dataString += "DPSX";
//       dataString += ",";
//       dataString += "DPSY";
//       dataString += ",";
//       dataString += "DPSZ";
//       dataString += ",";
//       dataString += "RPM";
//       dataString += ",";
//       dataString += "VEL";
//       dataString += ",";
//       dataString += "TEMP_MOTOR";
//       dataString += ",";
//       dataString += "FLAGS";
//       dataString += ",";
//       dataString += "SOC";
//       dataString += ",";
//       dataString += "TEMP_CVT";
//       dataString += ",";
//       dataString += "VOLT";
//       dataString += ",";
//       dataString += "LATITUDE";
//       dataString += ",";
//       dataString += "LONGITUDE";
//       dataString += ",";
//       dataString += "TIMESTAMP";
//       dataString += ",";
//       dataString += "ID=" + String(CAR_ID);
//     }

//     else
//     {
//       // imu,
//       dataString += String(volatile_packet.cont);
//       dataString += String(volatile_packet.imu_acc.acc_x);
//       dataString += ",";
//       dataString += String(volatile_packet.imu_acc.acc_y);
//       dataString += ",";
//       dataString += String(volatile_packet.imu_acc.acc_z);
//       dataString += ",";
//       dataString += String(volatile_packet.imu_dps.dps_x);
//       dataString += ",";
//       dataString += String(volatile_packet.imu_dps.dps_y);
//       dataString += ",";
//       dataString += String(volatile_packet.imu_dps.dps_z);
//       dataString += ",";

//       dataString += String(volatile_packet.rpm);
//       dataString += ",";
//       dataString += String(volatile_packet.speed);
//       dataString += ",";
//       dataString += String(volatile_packet.temperature);
//       dataString += ",";
//       dataString += String(volatile_packet.SOC);
//       dataString += ",";
//       dataString += String(volatile_packet.cvt);
//       dataString += ",";
//       dataString += String(volatile_packet.volt);
//       dataString += ",";
//       dataString += String(volatile_packet.flags);
//       dataString += ",";
//       dataString += String(volatile_packet.latitude);
//       dataString += ",";
//       dataString += String(volatile_packet.longitude);
//       dataString += ",";
//       dataString += String(volatile_packet.timestamp);
//     }

//   return dataString;
// }

// /* SD State Machine */
// void SdStateMachine(void *pvParameters)
// {
//   while(1)
//   {
//     if(saveFlag)
//     {
//       sdConfig();
//       saveFlag = false;
//     }
//     vTaskDelay(1);
//   }
// }

// void RadioStateMachine(void *pvParameters)
// {
//   while(1)
//   {
//     if (Serial2.available()) {
//     Lora.GetStruct(&volatile_packet, sizeof(volatile_packet));

//     digitalWrite(LED_BUILTIN, HIGH);

//     Serial.println("------------");
//     Serial.print("Acc X: ");          Serial.println(volatile_packet.imu_acc.acc_x);
//     Serial.print("Acc Y: ");          Serial.println(volatile_packet.imu_acc.acc_y);
//     Serial.print("Acc Z: ");          Serial.println(volatile_packet.imu_acc.acc_z);
//     Serial.print("DPS X: ");          Serial.println(volatile_packet.imu_dps.dps_x);
//     Serial.print("DPS Y: ");          Serial.println(volatile_packet.imu_dps.dps_y);
//     Serial.print("DPS Z: ");          Serial.println(volatile_packet.imu_dps.dps_z);
//     Serial.print("RPM: ");            Serial.println(volatile_packet.rpm);
//     Serial.print("SPEED: ");          Serial.println(volatile_packet.speed);
//     Serial.print("TEMPERATURE: ");    Serial.println(volatile_packet.temperature);
//     Serial.print("FLAGS: ");          Serial.println(volatile_packet.flags);
//     Serial.print("SOC: ");            Serial.println(volatile_packet.SOC);
//     Serial.print("CVT: ");            Serial.println(volatile_packet.cvt);
//     Serial.print("VOLT: ");           Serial.println(volatile_packet.volt);
//     Serial.print("LATITUDE: ");       Serial.println(volatile_packet.latitude);
//     Serial.print("LONGITUDE: ");      Serial.println(volatile_packet.longitude);
//     Serial.print("Times: ");          Serial.println(volatile_packet.timestamp);

//     digitalWrite(LED_BUILTIN, LOW);
//     Last = millis();
//   }
//   else {
//     if ((millis() - Last) > 1000) {
//       Serial.println("Searching: ");
//       Last = millis();
//     }
//   }
//     vTaskDelay(1000);
//   }
// }

// void ticker1HzISR()
// {
//   saveFlag = true;
// }