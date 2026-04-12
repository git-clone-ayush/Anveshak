#ifndef TOF_DISTANCE_H
#define TOF_DISTANCE_H

#include <Arduino.h>

namespace TofDistance
{
  struct Sample
  {
    Sample()
      : distanceMm(0), distanceCm(0.0f), timestampMs(0), valid(false), timeout(false)
    {
    }

    uint16_t distanceMm;
    float distanceCm;
    unsigned long timestampMs;
    bool valid;
    bool timeout;
  };

  void begin();
  void loop();
  bool isReady();
  Sample latestSample();
}

#endif
