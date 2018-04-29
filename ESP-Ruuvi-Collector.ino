#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

void array_to_string(byte array[], unsigned int len, char buffer[])
{
    for (unsigned int i = 0; i < len; i++)
    {
        byte nib1 = (array[i] >> 4) & 0x0F;
        byte nib2 = (array[i] >> 0) & 0x0F;
        buffer[i*2+0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
        buffer[i*2+1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
    }
    buffer[len*2] = '\0';
}

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
  unsigned int pressure = getUShort(data, 7) + 50000;
  short accelX = getShort(data, 9);
  short accelY = getShort(data, 11);
  short accelZ = getShort(data, 13);
  
  
  Serial.println(temperature, 2);
  Serial.println(humidity, 4);
  Serial.println(pressure);
  Serial.println();
}

class AdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      byte* mData = (byte*)advertisedDevice.getManufacturerData().data();
      if(mData[0] = 0x99 && mData[1] == 0x04)
      {
        if(mData[2] == 0x05)
          DecodeV5(mData);
        //char str[sizeof(mData) * 4] = "";
        //array_to_string(mData, sizeof(mData) * 2, str);
        //Serial.println(str);
      }
      //Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
    }
};

BLEScan* pBLEScan;

void setup() {
  Serial.begin(115200);
  Serial.println("Scanning...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);

}

void loop() {
  BLEScanResults foundDevices = pBLEScan->start(1);

}
