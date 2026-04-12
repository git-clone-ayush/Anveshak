#include <Arduino.h>
#include <Wire.h>

// I2C_SCANNER - Scans for I2C devices on the bus
// Compile: Comment out main.cpp include in platformio.ini, uncomment this one
// Or create a separate config

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\nI2C Scanner Starting...");
  Serial.println("Scanning I2C bus on GPIO 21 (SDA) and GPIO 22 (SCL)...\n");

  Wire.begin(21, 22);  // ESP32 I2C pins
  Wire.setClock(400000);

  Serial.println("I2C Address | HEX  | Device");
  Serial.println("------------|------|-------");

  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      nDevices++;
      Serial.printf("0x%02X       | %3d  | Device Found\n", address, address);

      // Specific device checks
      if (address == 0x46 || address == 0x47)
      {
        Serial.println("           |      | ↑ Likely BMP585!");
      }
      if (address == 0x68 || address == 0x69)
      {
        Serial.println("           |      | ↑ Likely MPU6050!");
      }
    }
    else if (error == 4)
    {
      Serial.printf("0x%02X       | %3d  | Unknown error at this address\n", address, address);
    }
  }

  Serial.println("\n------------|------|-------");
  Serial.printf("Scan complete! Found %d device(s)\n\n", nDevices);

  if (nDevices == 0)
  {
    Serial.println("⚠ No I2C devices found!");
    Serial.println("  Check: Power supply, pull-up resistors, wire connections");
  }
}

void loop()
{
  // Rescan every 10 seconds
  delay(10000);

  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      nDevices++;
    }
  }

  Serial.printf("[Rescan] Found %d device(s) on I2C bus\n", nDevices);
}
