#include <Arduino.h>

#include "bmp585_test.h"

namespace
{
  constexpr unsigned long printIntervalMs = 500;
  unsigned long lastPrintMs = 0;
  int readCount = 0;
}

void setup()
{
  Bmp585Test::begin();
}

void loop()
{
  Bmp585Test::loop();

  const unsigned long now = millis();
  if (now - lastPrintMs < printIntervalMs)
  {
    return;
  }

  lastPrintMs = now;

  const Bmp585Test::Sample sample = Bmp585Test::latestSample();
  
  if (!sample.valid)
  {
    readCount++;
    if (readCount % 10 == 0)  // Print status every 5 seconds
    {
      Serial.printf("[%lu ms] Waiting for valid BMP585 reading... (attempt %d)\n", now, readCount / 10);
    }
    return;
  }

  readCount = 0;  // Reset counter on valid read
  Serial.printf("Temp: %.2f °C | Pressure: %.2f hPa | Altitude: %.2f m\n",
                sample.temperatureC,
                sample.pressureHpa,
                sample.altitudeM);
}

