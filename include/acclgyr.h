#ifndef ACCLGYR_H
#define ACCLGYR_H

#include <Arduino.h>

#include "filter.h"

namespace AcclGyr
{
  struct Sample
  {
    Sample()
      : temperatureC(0.0f), timestampMs(0), valid(false)
    {
    }

    Filter::ImuSample imu;
    float temperatureC;
    unsigned long timestampMs;
    bool valid;
  };

  void begin();
  void loop();
  bool isReady();
  Sample latestSample();
}

#endif
