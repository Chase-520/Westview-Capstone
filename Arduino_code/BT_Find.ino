#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

#define TARGET_MAC "14:cb:65:fe:a0:e8"

void setup() {
  Serial.begin(115200);
  Serial.println("BLE Device Scanner");
  Serial.print("Looking for: ");
  Serial.println(TARGET_MAC);
}

void loop() {
  Serial.println("Starting scan...");
  
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  
  // Correct: start() returns BLEScanResults* (pointer)
  BLEScanResults* pResults = pBLEScan->start(5);
  
  bool found = false;
  
  for (int i = 0; i < pResults->getCount(); i++) {
    BLEAdvertisedDevice device = pResults->getDevice(i);
    String deviceMac = device.getAddress().toString().c_str();
    Serial.print("Device Mac is "); Serial.println(deviceMac);
    if (deviceMac == TARGET_MAC) {
      found = true;
      Serial.print("Found device: ");
      Serial.print(deviceMac);
      
      if (device.haveName()) {
        Serial.print(" Name: ");
        Serial.print(device.getName().c_str());
      }
      
      if (device.haveRSSI()) {
        Serial.print(" RSSI: ");
        Serial.print(device.getRSSI());
        Serial.print("dBm");
      }
      
      Serial.println();
      break;
    }
  }
  
  if (!found) {
    Serial.println("Target device not found");
  }
  
  pBLEScan->clearResults();
  
  Serial.println("Waiting 3 seconds...");
  delay(3000);
}
