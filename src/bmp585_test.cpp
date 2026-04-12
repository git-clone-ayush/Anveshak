#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_BMP58X.h"

#include "bmp585_test.h"

namespace
{
  constexpr int sdaPin = 21;
  constexpr int sclPin = 22;
  constexpr unsigned long printFriendlyReadIntervalMs = 100;

  // Create sensor object using I2C
  DFRobot_BMP58X_I2C bmp(&Wire, 0x47);  // I2C address 0x47
  
  bool sensorReady = false;
  unsigned long lastReadMs = 0;
  Bmp585Test::Sample latestBmpSample;
}

namespace Bmp585Test
{
  void begin()
  {
    Serial.begin(115200);
    delay(1000);  // Wait for serial stabilization

    Serial.println("\n\n=== BMP585 Sensor Test (DFRobot) ===");
    Serial.println("Initializing I2C...");

    Wire.begin(sdaPin, sclPin);
    Wire.setClock(400000);

    Serial.printf("I2C configured - SDA:%d SCL:%d, Clock:400kHz\n", sdaPin, sclPin);
    Serial.printf("Attempting BMP585 initialization at address 0x47...\n\n");

    // Initialize sensor
    if (!bmp.begin())
    {
      Serial.println("✗ Failed to initialize BMP585!");
      Serial.println("  Troubleshooting:");
      Serial.println("  - Check power supply (3.3V)");
      Serial.println("  - Check I2C pull-up resistors");
      Serial.println("  - Verify I2C wiring");
      Serial.println("  - Run I2C scanner\n");
      return;
    }

    Serial.println("✓ BMP585 initialization successful!\n");

    // Configure sensor
    Serial.println("Configuring sensor...");
    
    // Set measurement mode to normal
    if (bmp.setMeasureMode(bmp.eNormal) != 0)
    {
      Serial.println("✗ Failed to set measurement mode");
      return;
    }

    // Set output data rate (ODR)
    if (bmp.setODR(bmp.eOdr50Hz) != 0)
    {
      Serial.println("✗ Failed to set ODR");
      return;
    }

    // Set oversampling: 2x for temperature, 16x for pressure
    if (bmp.setOSR(bmp.eOverSampling2, bmp.eOverSampling16) != 0)
    {
      Serial.println("✗ Failed to set oversampling");
      return;
    }

    // Configure IIR filter
    if (bmp.configIIR(bmp.eFilter3, bmp.eFilter3) != 0)
    {
      Serial.println("✗ Failed to configure IIR filter");
      return;
    }

    sensorReady = true;
    Serial.println("✓ Sensor configuration complete!\n");
  }

  void loop()
  {
    if (!sensorReady)
    {
      return;
    }

    const unsigned long now = millis();
    if (now - lastReadMs < printFriendlyReadIntervalMs)
    {
      return;
    }

    lastReadMs = now;

    // Read sensor data
    float temperature = bmp.readTempC();
    float pressure = bmp.readPressPa();
    float altitude = bmp.readAltitudeM();

    // Update sample
    latestBmpSample.temperatureC = temperature;
    latestBmpSample.pressureHpa = pressure / 100.0f;  // Convert Pa to hPa
    latestBmpSample.altitudeM = altitude;
    latestBmpSample.timestampMs = now;
    latestBmpSample.valid = true;
  }

  bool isReady()
  {
    return sensorReady && latestBmpSample.valid;
  }

  Sample latestSample()
  {
    return latestBmpSample;
  }
}
