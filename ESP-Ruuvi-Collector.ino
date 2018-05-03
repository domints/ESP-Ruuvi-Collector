#include <SimpleBLE.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WiFi.h>
#include "InfluxArduino.hpp"

InfluxArduino influx;
SimpleBLE ble;


const char WIFI_NAME[] = "SzymenowaSiec";
const char WIFI_PASS[] = "pomidor1";

const char INFLUX_DATABASE[] = "ruuvi";
const char INFLUX_IP[] = "vps.dszymanski.pl";
const char INFLUX_USER[] = "dominik";
const char INFLUX_PASS[] = "r1Xx7qtpPUdT";
const char INFLUX_MEASUREMENT[] = "ruuviTag";

char formatString[] = "temperature=%0.3f,pressure=%i,humidity=%0.3f,accel_x=%i,accel_y=%i,accel_z=%i,milivolts=%i";

short getShort(byte* data, int index)
{
  return (short)((data[index] << 8) + (data[index + 1]));
}

unsigned short getUShort(byte* data, int index) 
{
  return (unsigned short)((data[index] << 8) + (data[index + 1]));
}

void DecodeV5(byte* data)
{
  short tempRaw = getShort(data, 3);
  double temperature = (double)tempRaw * 0.005;
  unsigned short humRaw = getUShort(data, 5);
  double humidity = (double)humRaw * 0.0025;
  unsigned int pressure = (getUShort(data, 7) + 50000);
  short accelX = getShort(data, 9);
  short accelY = getShort(data, 11);
  short accelZ = getShort(data, 13);

  unsigned short voltRaw = data[15] << 3 | data[16] >> 5;
  unsigned char tPowRaw = data[16] && 0x1F;
  unsigned short voltage = voltRaw + 1600;
  char power = tPowRaw* 2 - 40;
  
  char fields[128];
  sprintf(fields, formatString, temperature, pressure, humidity, accelX, accelY, accelZ, voltage);
  Serial.println("writing influx...");
  bool writeSuccessful = influx.write(INFLUX_MEASUREMENT, "device=esp", fields);
  if (!writeSuccessful)
  {
    Serial.print("error: ");
    Serial.println(influx.getResponse());
  }
  Serial.println("write done.");
}

class AdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      byte* mData = (byte*)advertisedDevice.getManufacturerData().data();
      Serial.print("Found something... ");
      Serial.print(mData[0], HEX);
      Serial.println(mData[1], HEX);
      if(mData[0] = 0x99 && mData[1] == 0x04)
      {
        Serial.println("found tag.");
        if(mData[2] == 0x05)
          DecodeV5(mData);
      }
   }
};

BLEScan* pBLEScan;

void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_NAME, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected!");
  ble.begin("ESP32 SimpleBLE");
  Serial.println("Connecting to influx");
  influx.configure(INFLUX_DATABASE, INFLUX_IP);
  Serial.println("Authorizing influx...");
  influx.authorize(INFLUX_USER, INFLUX_PASS);
  Serial.println("influx ready.");
  
  BLEDevice::init("ESP-Ruuvi-Collector");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
}

void loop() {
  Serial.println("BLE Scanning...");
  BLEScanResults foundDevices = pBLEScan->start(1);
}
