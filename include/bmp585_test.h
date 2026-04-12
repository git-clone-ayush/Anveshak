#ifndef BMP585_TEST_H
#define BMP585_TEST_H

#include <Arduino.h>

namespace Bmp585Test
{
  struct Sample
  {
    Sample()
      : temperatureC(0.0f), pressureHpa(0.0f), altitudeM(0.0f), timestampMs(0), valid(false)
    {
    }

    float temperatureC;
    float pressureHpa;
    float altitudeM;
    unsigned long timestampMs;
    bool valid;
  };

  void begin();
  void loop();
  bool isReady();
  Sample latestSample();
}

#endif
