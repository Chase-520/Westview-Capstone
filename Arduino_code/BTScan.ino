#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 BLE MAC Address Scanner ===");
  Serial.println("This will find Bluetooth devices and show their MAC addresses");
}

void loop() {
  Serial.println("\n🔍 Scanning for Bluetooth devices (15 seconds)...");
  Serial.println("Turn on your gamepad/controller in pairing mode!\n");
  
  // Initialize BLE
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  
  // Start scan - store as pointer
  BLEScanResults* foundDevices = pBLEScan->start(15, false);
  
  // Display results
  Serial.println("\n=== SCAN RESULTS ===");
  int deviceCount = foundDevices->getCount();
  Serial.print("Total devices found: ");
  Serial.println(deviceCount);
  
  if (deviceCount > 0) {
    Serial.println("\nList of devices (with MAC addresses):");
    Serial.println("---------------------------------------");
    
    for (int i = 0; i < deviceCount; i++) {
      BLEAdvertisedDevice device = foundDevices->getDevice(i);
      
      Serial.print(i + 1);
      Serial.print(". ");
      
      // Device name
      if (device.haveName()) {
        Serial.print("\"");
        Serial.print(device.getName().c_str());
        Serial.print("\"");
      } else {
        Serial.print("[No Name]");
      }
      
      // MAC Address (this is what you need!)
      Serial.print(" - MAC: ");
      Serial.print(device.getAddress().toString().c_str());
      
      // Signal strength
      if (device.haveRSSI()) {
        Serial.print(" (Signal: ");
        Serial.print(device.getRSSI());
        Serial.print("dBm)");
      }
      
      // Check if it's a gamepad
      String name = device.getName().c_str();
      name.toUpperCase();
      
      if (name.indexOf("XBOX") != -1) {
        Serial.print(" 🎮 XBOX CONTROLLER!");
      } else if (name.indexOf("PLAYSTATION") != -1 || name.indexOf("PS4") != -1 || name.indexOf("PS5") != -1) {
        Serial.print(" 🎮 PLAYSTATION CONTROLLER!");
      } else if (name.indexOf("CONTROLLER") != -1 || name.indexOf("GAMEPAD") != -1) {
        Serial.print(" 🎮 CONTROLLER!");
      }
      
      Serial.println();
    }
    
    Serial.println("\n📝 To connect to a gamepad:");
    Serial.println("1. Copy the MAC address (format: XX:XX:XX:XX:XX:XX)");
    Serial.println("2. Use it in your connection code");
    Serial.println("3. Make sure the gamepad stays in pairing mode");
  } else {
    Serial.println("\nNo devices found. Make sure:");
    Serial.println("1. Your gamepad is in pairing mode");
    Serial.println("2. Gamepad Bluetooth is ON");
    Serial.println("3. You're within 10 meters of ESP32");
    Serial.println("\nGamepad pairing instructions:");
    Serial.println("• Xbox: Hold sync button until light flashes");
    Serial.println("• PS4: Hold Share + PS button until light flashes");
    Serial.println("• PS5: Hold Create + PS button until light flashes");
  }
  
  pBLEScan->clearResults();   // Clear results
  Serial.println("\n=== Next scan in 20 seconds ===");
  delay(20000);
}
