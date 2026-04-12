#include <Arduino.h>
#include <Wire.h>

// Direct I2C communication with BMP585 at register level
// This bypasses the Adafruit library and talks directly to the sensor

#define BMP585_ADDR 0x47
#define BMP585_CHIP_ID_REG 0x01

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n=== BMP585 Raw I2C Register Test ===\n");

  Wire.begin(21, 22);
  Wire.setClock(400000);
  Serial.println("I2C initialized (GPIO 21=SDA, GPIO 22=SCL, 400kHz)\n");

  // Step 1: Read chip ID register
  Serial.println("Step 1: Reading Chip ID Register (0x01)...");
  
  Wire.beginTransmission(BMP585_ADDR);
  Wire.write(BMP585_CHIP_ID_REG);
  byte error = Wire.endTransmission();

  if (error != 0)
  {
    Serial.printf("  ✗ Error writing to sensor: %d\n", error);
    return;
  }

  // Request 1 byte
  Wire.requestFrom(BMP585_ADDR, 1);
  if (Wire.available())
  {
    byte chipID = Wire.read();
    Serial.printf("  ✓ Chip ID: 0x%02X\n", chipID);

    // BMP585 should return 0x50
    if (chipID == 0x50)
    {
      Serial.println("    ✓✓ CORRECT! This is a BMP585!\n");
    }
    else
    {
      Serial.printf("    ⚠ Unexpected value. BMP585 should be 0x50\n");
      Serial.printf("    Possible: Wrong address or wrong device\n\n");
    }
  }
  else
  {
    Serial.println("  ✗ No response from sensor\n");
  }

  // List all readable registers
  Serial.println("Step 2: Reading all accessible registers...");
  Serial.println("\nAddress | Hex  | Decimal | Notes");
  Serial.println("--------|------|---------|-------");

  for (int reg = 0x00; reg <= 0x7F; reg++)
  {
    Wire.beginTransmission(BMP585_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission() == 0)
    {
      Wire.requestFrom(BMP585_ADDR, 1);
      if (Wire.available())
      {
        byte value = Wire.read();
        Serial.printf("0x%02X    | 0x%02X | %3d      | ", reg, value, value);

        // Annotate known registers
        if (reg == 0x01)
          Serial.println("CHIP_ID");
        else if (reg == 0x00)
          Serial.println("CHIP_REVISION");
        else if (reg >= 0x04 && reg <= 0x09)
          Serial.println("Status/Data");
        else if (reg >= 0x0A && reg <= 0x0F)
          Serial.println("ADC Data");
        else if (reg >= 0x10 && reg <= 0x1F)
          Serial.println("Config");
        else
          Serial.println("");
      }
    }
  }

  Serial.println("\n=== Test Complete ===\n");
}

void loop()
{
  delay(1000);
}
