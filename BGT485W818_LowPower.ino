#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#include <ArduinoJson.h>

#include <EEPROM.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Wire.h>
#include <ModbusMaster.h>
#include "DOSensor.h"
#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"
#include <TaskScheduler.h>

#define _TASK_TIMECRITICAL

HardwareSerial modbus(2);
HardwareSerial_NB_BC95 AISnb;

BluetoothSerial SerialBT;


#define battPIN  34
#define donePIN  25



String HOSTNAME = "";
String deviceToken = "xx";
String serverIP = "147.50.151.130"; // Your Server IP;
String serverPort = "19956"; // Your Server Port;
String json = "";

ModbusMaster node;

#define trigWDTPin    32
#define ledHeartPIN   0


struct Meter
{
  String SO2;
  String NO2;
  String CO;
  String O3;
  String PM2_5;
  String PM10;
  String temp;
  String hum;

  float bat;

};
Meter meter;
signal meta;
uint16_t dataWeather[8];

unsigned long currentMillis;
unsigned long previousMillis;
int interval = 15; // Interval Time
int intervalSleep = 30; // Interval Time
//unsigned int previous_check = 0;
boolean waitSleep = 0;
unsigned long previousMillisSleep;


float Batt = 0.0;
int countSend = 0;

void setup()
{

  Serial.begin(115200);

  modbus.begin(9600, SERIAL_8N1, 16, 17);


  SerialBT.begin(HOSTNAME); //Bluetooth


  SerialBT.begin(HOSTNAME); //Bluetooth device name
  SerialBT.println(HOSTNAME);
  AISnb.debug = true;
  AISnb.setupDevice(serverPort);

  deviceToken = AISnb.getNCCID();
  HOSTNAME = deviceToken;
  Serial.print("nccid:");
  Serial.println(deviceToken);

  Serial.println();
  Serial.println(F("***********************************"));

  Serial.println("Initialize...");


}

void sendViaNBIOT()
{
  meta = AISnb.getSignal();

  Serial.print("RSSI:"); Serial.println(meta.rssi);

  json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);

  json.concat("\",\"SO2\":");
  json.concat(meter.SO2);
  json.concat(",\"NO2\":");
  json.concat(meter.NO2);
  json.concat(",\"CO\":");
  json.concat(meter.CO);
  json.concat(",\"O3\":");
  json.concat(meter.O3);
  json.concat(",\"PM2_5\":");
  json.concat(meter.PM2_5);
  json.concat(",\"PM10\":");
  json.concat(meter.PM10);
  json.concat(",\"temp\":");
  json.concat(meter.temp);

  json.concat(",\"hum\":");
  json.concat(meter.hum);
  json.concat(",\"bat\":");
  json.concat(meter.bat);

  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat(",\"csq\":");
  json.concat(meta.csq);
  json.concat("}");
  Serial.println(json);
  SerialBT.println(json);
  //
  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  UDPReceive resp = AISnb.waitResponse();
  Serial.print("rssi:");
  Serial.println(meta.rssi);
  SerialBT.print("rssi:");
  SerialBT.println(meta.rssi);
}



float Read_Batt()
{
  unsigned int vRAW = 0;
  float Vout = 0.0;
  float Vin = 0.0;
  float R1 = 15000.0;
  float R2 = 3900.0;
  int bitRes = 4096;
  int16_t adc = 0;
  Serial.println("Read_Batt()");
  for (int a = 0; a < 20; a++)
  {
    adc  += analogRead(battPIN);
    Serial.print(adc);
    delay(1);
  }

  vRAW = adc / 20;
  Vout = (vRAW * 3.3) / bitRes;
  Vin = Vout / (R2 / (R1 + R2));
  if (Vin < 0.05)
  {
    Vin = 0.0;
  }
  Serial.println("end.Read_Batt()");
  return Vin;
}


void readMeter()
{
  readWeather(_SO2);


  Serial.print("meter.SO2:"); Serial.println( meter.SO2);
  Serial.print("meter.NO2:"); Serial.println( meter.NO2);
  Serial.print("meter.CO:"); Serial.println( meter.CO);
  Serial.print("meter.O3:"); Serial.println( meter.O3);
  Serial.print("meter.PM25:"); Serial.println( meter.PM2_5);
  Serial.print("meter.PM10:"); Serial.println( meter.PM10);
  Serial.print("meter.temp:"); Serial.println( meter.temp);
  Serial.print("meter.hum:"); Serial.println( meter.hum);


  Serial.println("");

}

void t1CallgetMeter() {     // Update read all data
  readMeter();
  meter.bat = Read_Batt();
  sendViaNBIOT();
  
}



void readWeather(uint16_t  REG)
{
  static uint32_t i;
  uint16_t j, result;

  uint32_t value = 0;
  float val = 0.0;

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(ID_Meter, modbus);

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, 8);
  Serial.print("result:"); Serial.print(result); Serial.print(" node.ku8MBSuccess:"); Serial.println(node.ku8MBSuccess);

  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 8; j++)
    {
      dataWeather[j] = node.getResponseBuffer(j);
      SerialBT.print(REG); SerialBT.print(":"); SerialBT.print(j); SerialBT.print(":");  SerialBT.println(dataWeather[j]);
      Serial.print(REG); Serial.print(":"); Serial.print(j); Serial.print(":");  Serial.println(dataWeather[j]);
    }

    meter.SO2 = dataWeather[0];
    meter.NO2 = dataWeather[1];
    meter.CO = dataWeather[2];
    meter.O3 = dataWeather[3];
    meter.PM2_5 = dataWeather[4];
    meter.PM10 = dataWeather[5];
    meter.temp = dataWeather[6] / 100 - 40;

    meter.hum = dataWeather[7] / 100;

  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug

  }
}



void doneProcess()
{
  Serial.println("!!!!!! Done ready to Sleep ~10 Min (TPL5110) !!!!!!");
  pinMode(donePIN, OUTPUT);
  digitalWrite(donePIN, HIGH);
  delay(100);
}

void loop()
{
  unsigned long currentMillis = millis();



  currentMillis = millis() / 1000;

  if ((currentMillis - previousMillis >= interval) && (waitSleep == 0))
  {

    
    t1CallgetMeter();



    
    previousMillis = millis() / 1000;

    countSend++;

    if (countSend >= 4)
    {
      waitSleep = 1;
      previousMillisSleep = millis() / 1000;
    }
  }

  if (waitSleep == 1)
  {
    if (currentMillis - previousMillisSleep >= intervalSleep)
    {
      
      countSend = 0;
      waitSleep = 0;
      delay(2000);
      doneProcess();
    }
  }



}
