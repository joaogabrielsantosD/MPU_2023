#include <Arduino.h>
#include <Ticker.h>
#include <SD.h>
#include <EBYTE.h>
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

EBYTE Lora(&LoRaUART, PIN_M0, PIN_M1, PIN_AUX);

/* Global variables */
File root, dataFile;
uint8_t data[sizeof(radio_packet_t)];
char file_name[20];
unsigned long Last = 0;

/* Global Functions */
bool sdConfig();
int countFiles(File dir);
void sdSave();
String packetToString();

void setup()
{
  Serial.begin(115200);
  LoRaUART.begin(LoRa_Baud_Rate);

  pinMode(LED_BUILTIN, OUTPUT);

  if(!Lora.init() || !sdConfig())
  {
    Serial.println("LoRa/SD ERROR !!!");
    return;
  }

  Lora.SetAddressH(1);
  Lora.SetAddressL(1);
  Lora.SetChannel(CAR_ID);
  Lora.SetAirDataRate(ADR_1200);
  Lora.SetTransmitPower(OPT_TP30);
  Lora.SetMode(MODE_NORMAL);
  Lora.SetUARTBaudRate(UDR_9600);
  Lora.SetFECMode(OPT_FECENABLE);
  Lora.SaveParameters(PERMANENT);
  //Lora.PrintParameters();

  memset(&volatile_packet, 0x00, sizeof(volatile_packet));
}

void loop()
{
  if(LoRaUART.available())
  {
    digitalWrite(LED_BUILTIN, HIGH);

    Lora.GetStruct(&volatile_packet, sizeof(volatile_packet));

    /* Write in Serial */
    memcpy(&data, (uint8_t *)&volatile_packet, sizeof(volatile_packet));

    Serial.write(CAR_ID);

    for(int i = 0; i < sizeof(data); i++)
    {
      Serial.write(data[i]);
    }

    Serial.write(0xff); // Flag to end the packet

    sdSave();

    /*
    Serial.println("----------------------------------------");
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
    //Serial.print("FUEL LEVEL: ");     Serial.println(volatile_packet.fuel_level);
    Serial.print("Timestamp: ");      Serial.println(volatile_packet.timestamp);
    Serial.println("----------------------------------------");
    */

    vTaskDelay(10);
  }

  else
  {
    if((millis() - Last) > 1000)
    {
      //Serial.printf("Searching: \n");
      Last = millis();
    }
    digitalWrite(LED_BUILTIN, LOW);
  }
}

/* Global Functions */
bool sdConfig()
{
  if(!SD.begin(SD_CS)) return false;

  root = SD.open("/");
  int num_files = countFiles(root);
  sprintf(file_name, "/%s%d.csv", "data", num_files+1);

  dataFile = SD.open(file_name, FILE_APPEND);

  if(dataFile)
  {
    dataFile.println("ACCx,ACCy,ACCz,DPSx,DPSy,DPSz,rpm,speed,motor,flags,SOC,cvt,volt,LAT,LNG,timestamp");
    dataFile.close();
    return true;
  } 
  
  else return false;
}

int countFiles(File dir)
{
  int fileCountOnSD = 0; // for counting files

  for(;;)
  {
    File entry = dir.openNextFile();
  
    if(!entry)
    {
      // no more files
      break;
    }
    // for each file count it
    fileCountOnSD++;
    entry.close();
  }

  return fileCountOnSD-1;
}

void sdSave()
{
  dataFile = SD.open(file_name, FILE_APPEND);

  dataFile.println(packetToString());
  dataFile.close();
}

String packetToString()
{
  String str = "";
    str += String((volatile_packet.imu_acc.acc_x*0.061)/1000);
    str += ",";
    str += String((volatile_packet.imu_acc.acc_y*0.061)/1000);
    str += ",";
    str += String((volatile_packet.imu_acc.acc_z*0.061)/1000);
    str += ",";
    str += String(volatile_packet.imu_dps.dps_x);
    str += ",";
    str += String(volatile_packet.imu_dps.dps_y);
    str += ",";
    str += String(volatile_packet.imu_dps.dps_z);
    str += ",";
    str += String(volatile_packet.rpm);
    str += ",";
    str += String(volatile_packet.speed);
    str += ",";
    str += String(volatile_packet.temperature);
    str += ",";
    str += String(volatile_packet.flags);
    str += ",";
    str += String(volatile_packet.SOC);
    str += ",";
    str += String(volatile_packet.cvt);
    str += ",";
    str += String(volatile_packet.volt);
    str += ",";
    str += String(volatile_packet.latitude);
    str += ",";
    str += String(volatile_packet.longitude);
    str += ",";
    str += String(volatile_packet.timestamp);
    
    return str;
}
