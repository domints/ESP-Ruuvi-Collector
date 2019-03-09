#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WiFi.h>
#include <esp_system.h>
#include "InfluxArduino.hpp"

const int wdtTimeout = 30000;  //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

void IRAM_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

InfluxArduino influx;

const char WIFI_NAME[] = "<your wifi>";
const char WIFI_PASS[] = "<your pass>";

const char INFLUX_DATABASE[] = "<your influx database name>";
const char INFLUX_IP[] = "<your influx server ip>";
const char INFLUX_USER[] = "<influx user>";
const char INFLUX_PASS[] = "<influx pass>";
const char INFLUX_MEASUREMENT[] = "<influx measrement name>";

char formatStringFive[] = "temperature=%0.3f,pressure=%i,humidity=%0.3f,accel_x=%i,accel_y=%i,accel_z=%i,milivolts=%i";
char formatStringThree[] = "temperature=%0.3f,pressure=%i,humidity=%i,milivolts=%i";

// 16 bit
short getShort(byte* data, int index)
{
  return (short)((data[index] << 8) + (data[index + 1]));
}

short getShortone(byte* data, int index)
{
  return (short)((data[index]));
}

// 16 bit
unsigned short getUShort(byte* data, int index)
{
  return (unsigned short)((data[index] << 8) + (data[index + 1]));
}

unsigned short getUShortone(byte* data, int index)
{
  return (unsigned short)((data[index]));
}


void DecodeV3(byte* data)
{
  yield();  // reset esp watchdog
  digitalWrite(22, HIGH);
  short tempRaw = getUShortone(data, 4) & 0b01111111 - 2;
  short tempRawsign = getUShortone(data, 4) & 0b10000000;
  short tempRawdec = getUShortone(data, 5);
  double temperature = (double)tempRaw + (double)tempRawdec / 100;
  if (tempRawsign==128){
  temperature = temperature * -1;
  }
  byte humRaw = getUShortone(data, 3);
  short humidity = humRaw / 2;
  unsigned int pressure = (getUShort(data, 6) + 50000);
  unsigned int voltageraw = getUShort(data, 14);
  short voltage = (short)voltageraw;

  char fields[256];
  sprintf(fields, formatStringThree, temperature, pressure, humidity, voltage);
  bool writeSuccessful = influx.write(INFLUX_MEASUREMENT, "device=esp_test", fields);
  if (!writeSuccessful)
  {
    Serial.print("error: ");
    Serial.println(influx.getResponse());
  }
 
}

  void DecodeV5(byte* data)
  {
  yield();  // reset esp watchdog
  digitalWrite(22, LOW);
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

  char fields[256];
  sprintf(fields, formatStringFive, temperature, pressure, humidity, accelX, accelY, accelZ, voltage);
  bool writeSuccessful = influx.write(INFLUX_MEASUREMENT, "device=esp", fields);
  if (!writeSuccessful)
  {
    Serial.print("error: ");
    Serial.println(influx.getResponse());
  }
}



class AdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      byte* mData = (byte*)advertisedDevice.getManufacturerData().data();
      yield();  // reset esp watchdog
      if (mData[0] == 0x99 && mData[1] == 0x04 && mData[2] == 0x03)
    {
      if (mData[2] == 0x03)
      {
       /* Serial.print (mData[0]);
        Serial.print (" ; ");
        Serial.print (mData[1]);
        Serial.print (" ; ");
        Serial.print (mData[2]);
        Serial.println (" ; ");
        */
        DecodeV3(mData);
      }
      if (mData[2] == 0x05)
      {
       /* Serial.print (mData[0]);
        Serial.print (" ; ");
        Serial.print (mData[1]);
        Serial.print (" ; ");
        Serial.print (mData[2]);
        Serial.println (" ; ");
        */
        DecodeV5(mData);
      }
      yield();  // reset esp watchdog
    }
   }
};

BLEScan* pBLEScan;

void setup() {
  
  //WDT Stuff
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt

  
  pinMode(22, OUTPUT);
  digitalWrite(22, HIGH);
  Serial.begin(115200);

  WiFi.begin(WIFI_NAME, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    digitalWrite(22, LOW);
    Serial.print(".");
    digitalWrite(22, HIGH);
    yield();  // reset esp watchdog
  }
  Serial.println("WiFi connected!");
  influx.configure(INFLUX_DATABASE, INFLUX_IP);
  influx.authorize(INFLUX_USER, INFLUX_PASS);

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
}

long loopTimeMax=0;

void loop() {
  timerWrite(timer, 0); //reset timer (feed watchdog)
  long loopTime = millis();
  BLEScanResults foundDevices = pBLEScan->start(1);
  yield();
  digitalWrite(22, LOW);
  loopTime = millis() - loopTime;
  if (loopTimeMax < loopTime) loopTimeMax=loopTime;
  
  Serial.print("current loop time (ms): ");
  Serial.print(loopTime); //should be under 20000
  Serial.print(" / MaxTime: ");
  Serial.println(loopTimeMax); //should be under 20000
}
