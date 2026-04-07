#include <Arduino.h>
#include <SPI.h>

#include "optical_flow_test.h"

namespace
{
  const uint8_t flowCsPin = 5;
  const uint8_t flowSckPin = 18;
  const uint8_t flowMisoPin = 19;
  const uint8_t flowMosiPin = 23;
  const int8_t flowResetPin = 12; // Set to your ESP32 GPIO if RESET is wired, for example 16
  const int8_t flowNpdPin = 13;   // Set to your ESP32 GPIO if NPD is wired, for example 17

  const uint8_t regProductId = 0x00;
  const uint8_t regMotion = 0x02;
  const uint8_t regDeltaX = 0x03;
  const uint8_t regDeltaY = 0x04;
  const uint8_t regSqual = 0x05;
  const uint8_t regConfigurationBits = 0x0A;
  const uint8_t regMotionClear = 0x12;
  const uint8_t regPowerUpReset = 0x3A;

  const uint8_t productIdExpected = 0x17;
  const uint8_t motionBit = 0x80;
  const uint8_t overflowBit = 0x10;
  const uint8_t resolution1600cpiBit = 0x10;

  SPIClass flowSpi(VSPI);
  SPISettings flowSpiSettings(500000, MSBFIRST, SPI_MODE3);
  unsigned long lastReadMs = 0;
  unsigned long lastStatusMs = 0;
  bool sensorHealthy = false;
  int32_t totalX = 0;
  int32_t totalY = 0;
  float velocityX = 0.0f;
  float velocityY = 0.0f;
  float filteredVelocityX = 0.0f;
  float filteredVelocityY = 0.0f;
  const float velocityFilterAlpha = 0.25f;
  const uint8_t minUsableSqual = 8;
  OpticalFlowTest::Sample latestFlowSample;

  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);

  void hardwareReset()
  {
    if (flowNpdPin >= 0)
    {
      pinMode(flowNpdPin, OUTPUT);
      digitalWrite(flowNpdPin, HIGH); // NPD is active low, so HIGH keeps the sensor awake
      delayMicroseconds(50);
    }

    if (flowResetPin >= 0)
    {
      pinMode(flowResetPin, OUTPUT);
      digitalWrite(flowResetPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(flowResetPin, LOW);
      delayMicroseconds(500);
      return;
    }

    writeRegister(regPowerUpReset, 0x5A);
    delay(50);
  }

  uint8_t readRegister(uint8_t reg)
  {
    flowSpi.beginTransaction(flowSpiSettings);
    digitalWrite(flowCsPin, LOW);
    flowSpi.transfer(reg & 0x7F);
    delayMicroseconds(75);
    uint8_t value = flowSpi.transfer(0x00);
    digitalWrite(flowCsPin, HIGH);
    flowSpi.endTransaction();
    delayMicroseconds(1);
    return value;
  }

  void writeRegister(uint8_t reg, uint8_t value)
  {
    flowSpi.beginTransaction(flowSpiSettings);
    digitalWrite(flowCsPin, LOW);
    flowSpi.transfer(reg | 0x80);
    flowSpi.transfer(value);
    digitalWrite(flowCsPin, HIGH);
    flowSpi.endTransaction();
    delayMicroseconds(50);
  }

  void clearMotion()
  {
    writeRegister(regMotionClear, 0xFF);
    totalX = 0;
    totalY = 0;
    velocityX = 0.0f;
    velocityY = 0.0f;
    filteredVelocityX = 0.0f;
    filteredVelocityY = 0.0f;
    latestFlowSample.flow.velocityX = 0.0f;
    latestFlowSample.flow.velocityY = 0.0f;
    latestFlowSample.flow.quality = 0.0f;
    latestFlowSample.flow.valid = false;
  }
}

namespace OpticalFlowTest
{
  void begin()
  {
    Serial.begin(115200);
    delay(200);

    pinMode(flowCsPin, OUTPUT);
    digitalWrite(flowCsPin, HIGH);

    flowSpi.begin(flowSckPin, flowMisoPin, flowMosiPin, flowCsPin);
    delay(50);

    hardwareReset();

    const uint8_t productId = readRegister(regProductId);

    Serial.println("Optical flow test started");
    Serial.printf("Pins -> CS:%u SCK:%u MISO:%u MOSI:%u\n", flowCsPin, flowSckPin, flowMisoPin, flowMosiPin);
    Serial.printf("Pins -> RESET:%d NPD:%d\n", flowResetPin, flowNpdPin);
    Serial.printf("Product ID: 0x%02X\n", productId);

    if (productId != productIdExpected)
    {
      Serial.println("ADNS3080 not detected. Check wiring, logic level, RESET, and NPD.");
      Serial.println("If RESET/NPD are wired, set flowResetPin and flowNpdPin in optical_flow_test.cpp.");
      return;
    }

    sensorHealthy = true;

    uint8_t configBits = readRegister(regConfigurationBits);
    configBits &= ~resolution1600cpiBit;
    writeRegister(regConfigurationBits, configBits);
    delayMicroseconds(50);

    clearMotion();

    Serial.printf("Config bits: 0x%02X\n", readRegister(regConfigurationBits));
    Serial.println("ADNS3080 detected");
    Serial.println("Resolution set to 400 CPI");
  }

  void loop()
  {
    if (!sensorHealthy)
    {
      return;
    }

    if (millis() - lastReadMs < 20)
    {
      return;
    }

    const unsigned long previousReadMs = lastReadMs;
    lastReadMs = millis();

    const uint8_t motion = readRegister(regMotion);
    const uint8_t squal = readRegister(regSqual);
    const unsigned long now = lastReadMs;

    if (motion & overflowBit)
    {
      clearMotion();
      latestFlowSample.timestampMs = now;
      latestFlowSample.healthy = false;
      return;
    }

    if ((motion & motionBit) && squal >= minUsableSqual)
    {
      const int8_t deltaX = static_cast<int8_t>(readRegister(regDeltaX));
      const int8_t deltaY = static_cast<int8_t>(readRegister(regDeltaY));
      const float dtSeconds = (previousReadMs == 0) ? 0.02f : (now - previousReadMs) / 1000.0f;

      totalX += deltaX;
      totalY += deltaY;
      velocityX = deltaX / dtSeconds;
      velocityY = deltaY / dtSeconds;
      filteredVelocityX += velocityFilterAlpha * (velocityX - filteredVelocityX);
      filteredVelocityY += velocityFilterAlpha * (velocityY - filteredVelocityY);
    }
    else if (squal >= minUsableSqual)
    {
      velocityX = 0.0f;
      velocityY = 0.0f;
      filteredVelocityX += velocityFilterAlpha * (velocityX - filteredVelocityX);
      filteredVelocityY += velocityFilterAlpha * (velocityY - filteredVelocityY);
    }
    else
    {
      filteredVelocityX = 0.0f;
      filteredVelocityY = 0.0f;
    }

    latestFlowSample.flow.velocityX = filteredVelocityX;
    latestFlowSample.flow.velocityY = filteredVelocityY;
    latestFlowSample.flow.quality = static_cast<float>(squal);
    latestFlowSample.flow.valid = (squal >= minUsableSqual);
    latestFlowSample.totalX = totalX;
    latestFlowSample.totalY = totalY;
    latestFlowSample.timestampMs = now;
    latestFlowSample.healthy = latestFlowSample.flow.valid;

    if (now - lastStatusMs >= 1000)
    {
      lastStatusMs = now;
      Serial.printf("flow quality=%u valid=%s vx=%.2f vy=%.2f totalX=%ld totalY=%ld\n",
                    squal,
                    latestFlowSample.flow.valid ? "yes" : "no",
                    latestFlowSample.flow.velocityX,
                    latestFlowSample.flow.velocityY,
                    totalX,
                    totalY);
    }
  }

  bool isHealthy()
  {
    return sensorHealthy && latestFlowSample.healthy;
  }

  Sample latestSample()
  {
    return latestFlowSample;
  }
}
