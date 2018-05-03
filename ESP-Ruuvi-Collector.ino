#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WiFi.h>
#include "InfluxArduino.hpp"

InfluxArduino influx;

const char WIFI_NAME[] = "";
const char WIFI_PASS[] = "";

const char INFLUX_DATABASE[] = "";
const char INFLUX_IP[] = "";
const char INFLUX_USER[] = "";
const char INFLUX_PASS[] = "";
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
  digitalWrite(5, LOW);
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
  bool writeSuccessful = influx.write(INFLUX_MEASUREMENT, "device=esp", fields);
  if (!writeSuccessful)
  {
    Serial.print("error: ");
    Serial.println(influx.getResponse());
  }
  digitalWrite(5, HIGH);
}

class AdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      byte* mData = (byte*)advertisedDevice.getManufacturerData().data();
      if(mData[0] = 0x99 && mData[1] == 0x04)
      {
        if(mData[2] == 0x05)
          DecodeV5(mData);
      }
   }
};

BLEScan* pBLEScan;

void setup() {
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  Serial.begin(115200);

  WiFi.begin(WIFI_NAME, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    digitalWrite(5, LOW);
    Serial.print(".");
    digitalWrite(5, HIGH);
  }
  Serial.println("WiFi connected!");
  influx.configure(INFLUX_DATABASE, INFLUX_IP);
  influx.authorize(INFLUX_USER, INFLUX_PASS);
  
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
}

void loop() {
  BLEScanResults foundDevices = pBLEScan->start(1);
}
