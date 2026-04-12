#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#include "tof_distance.h"

namespace
{
  constexpr int sdaPin = 21;
  constexpr int sclPin = 22;
  constexpr uint8_t tofAddress = 0x29;
  constexpr unsigned long readIntervalMs = 60;
  constexpr uint16_t sensorTimeoutMs = 200;
  constexpr uint32_t measurementTimingBudgetUs = 50000;
  constexpr uint16_t minimumValidDistanceMm = 30;
  constexpr uint16_t maximumValidDistanceMm = 2000;
  constexpr size_t filterWindowSize = 5;

  VL53L0X tofSensor;
  bool sensorReady = false;
  unsigned long lastReadMs = 0;
  TofDistance::Sample latestDistanceSample;
  uint16_t distanceHistory[filterWindowSize] = {};
  size_t historyCount = 0;
  size_t historyIndex = 0;

  void addDistanceReading(uint16_t distanceMm)
  {
    distanceHistory[historyIndex] = distanceMm;
    historyIndex = (historyIndex + 1) % filterWindowSize;
    if (historyCount < filterWindowSize)
    {
      historyCount++;
    }
  }

  uint16_t filteredDistanceMm()
  {
    if (historyCount == 0)
    {
      return 0;
    }

    uint16_t sorted[filterWindowSize] = {};
    for (size_t i = 0; i < historyCount; ++i)
    {
      sorted[i] = distanceHistory[i];
    }

    for (size_t i = 0; i < historyCount; ++i)
    {
      for (size_t j = i + 1; j < historyCount; ++j)
      {
        if (sorted[j] < sorted[i])
        {
          const uint16_t temp = sorted[i];
          sorted[i] = sorted[j];
          sorted[j] = temp;
        }
      }
    }

    return sorted[historyCount / 2];
  }
}

namespace TofDistance
{
  void begin()
  {
    Serial.begin(115200);
    delay(200);

    Wire.begin(sdaPin, sclPin);
    Wire.setClock(400000);

    Serial.println("VL53L0X ToF startup");
    Serial.printf("I2C -> SDA:%d SCL:%d address:0x%02X\n", sdaPin, sclPin, tofAddress);

    tofSensor.setTimeout(sensorTimeoutMs);
    if (!tofSensor.init())
    {
      Serial.println("Failed to initialize VL53L0X. Check wiring, power, and sensor model.");
      return;
    }

    tofSensor.setMeasurementTimingBudget(measurementTimingBudgetUs);
    tofSensor.startContinuous(readIntervalMs);

    sensorReady = true;
    Serial.println("VL53L0X ready");
  }

  void loop()
  {
    if (!sensorReady)
    {
      return;
    }

    const unsigned long now = millis();
    if (now - lastReadMs < readIntervalMs)
    {
      return;
    }

    lastReadMs = now;

    const uint16_t distanceMm = tofSensor.readRangeContinuousMillimeters();
    const bool timeout = tofSensor.timeoutOccurred();

    latestDistanceSample.timestampMs = now;
    latestDistanceSample.timeout = timeout;

    if (timeout)
    {
      latestDistanceSample.valid = false;
      Serial.println("VL53L0X read timeout");
      return;
    }

    if (distanceMm < minimumValidDistanceMm || distanceMm > maximumValidDistanceMm)
    {
      latestDistanceSample.valid = false;
      Serial.printf("Ignoring out-of-range reading: %u mm\n", distanceMm);
      return;
    }

    addDistanceReading(distanceMm);

    const uint16_t smoothedDistanceMm = filteredDistanceMm();
    latestDistanceSample.distanceMm = smoothedDistanceMm;
    latestDistanceSample.distanceCm = static_cast<float>(smoothedDistanceMm) / 10.0f;
    latestDistanceSample.valid = true;
  }

  bool isReady()
  {
    return sensorReady && latestDistanceSample.valid;
  }

  Sample latestSample()
  {
    return latestDistanceSample;
  }
}
