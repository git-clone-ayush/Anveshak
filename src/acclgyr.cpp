#include <Arduino.h>
#include <Wire.h>

#include "acclgyr.h"

namespace
{
  constexpr uint8_t mpuAddress = 0x68;
  constexpr uint8_t whoAmIRegister = 0x75;
  constexpr uint8_t powerManagementRegister = 0x6B;
  constexpr uint8_t accelDataStartRegister = 0x3B;

  constexpr int sdaPin = 21;
  constexpr int sclPin = 22;
  constexpr unsigned long sampleIntervalMs = 1000;

  bool sensorReady = false;
  unsigned long lastSampleMs = 0;

  bool writeRegister(uint8_t reg, uint8_t value)
  {
    Wire.beginTransmission(mpuAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission(true) == 0;
  }

  bool readRegisters(uint8_t startReg, uint8_t* buffer, size_t length)
  {
    Wire.beginTransmission(mpuAddress);
    Wire.write(startReg);

    if (Wire.endTransmission(false) != 0)
    {
      return false;
    }

    const size_t received = Wire.requestFrom(static_cast<int>(mpuAddress), static_cast<int>(length), static_cast<int>(true));
    if (received != length)
    {
      while (Wire.available())
      {
        Wire.read();
      }
      return false;
    }

    for (size_t i = 0; i < length; ++i)
    {
      buffer[i] = Wire.read();
    }

    return true;
  }

  int16_t toInt16(uint8_t highByte, uint8_t lowByte)
  {
    return static_cast<int16_t>((highByte << 8) | lowByte);
  }
}

namespace AcclGyr
{
  void begin()
  {
    Serial.begin(115200);
    delay(200);

    Wire.begin(sdaPin, sclPin);
    Wire.setClock(400000);

    Serial.println("MPU6050 startup");
    Serial.printf("I2C -> SDA:%d SCL:%d address:0x%02X\n", sdaPin, sclPin, mpuAddress);

    uint8_t whoAmI = 0;
    if (!readRegisters(whoAmIRegister, &whoAmI, 1))
    {
      Serial.println("Failed to read WHO_AM_I from MPU6050");
      return;
    }

    Serial.printf("WHO_AM_I=0x%02X\n", whoAmI);

    if (whoAmI != 0x68 && whoAmI != 0x69)
    {
      Serial.println("Unexpected WHO_AM_I value. Check wiring, address pin AD0, and power.");
      return;
    }

    if (!writeRegister(powerManagementRegister, 0x00))
    {
      Serial.println("Failed to wake MPU6050");
      return;
    }

    delay(100);
    sensorReady = true;
    Serial.println("MPU6050 ready");
  }

  void loop()
  {
    if (!sensorReady)
    {
      delay(250);
      return;
    }

    const unsigned long now = millis();
    if (now - lastSampleMs < sampleIntervalMs)
    {
      delay(5);
      return;
    }

    lastSampleMs = now;

    uint8_t rawData[14];
    if (!readRegisters(accelDataStartRegister, rawData, sizeof(rawData)))
    {
      Serial.println("Failed to read accelerometer/gyro data");
      return;
    }

    const int16_t accelerometerX = toInt16(rawData[0], rawData[1]);
    const int16_t accelerometerY = toInt16(rawData[2], rawData[3]);
    const int16_t accelerometerZ = toInt16(rawData[4], rawData[5]);
    const int16_t temperatureRaw = toInt16(rawData[6], rawData[7]);
    const int16_t gyroX = toInt16(rawData[8], rawData[9]);
    const int16_t gyroY = toInt16(rawData[10], rawData[11]);
    const int16_t gyroZ = toInt16(rawData[12], rawData[13]);

    const float temperatureC = (temperatureRaw / 340.0f) + 36.53f;

    Serial.printf("accelX=%d accelY=%d accelZ=%d tempC=%.2f gyroX=%d gyroY=%d gyroZ=%d\n",
                  accelerometerX,
                  accelerometerY,
                  accelerometerZ,
                  temperatureC,
                  gyroX,
                  gyroY,
                  gyroZ);
  }
}
