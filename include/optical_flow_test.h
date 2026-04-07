#ifndef OPTICAL_FLOW_TEST_H
#define OPTICAL_FLOW_TEST_H

#include <Arduino.h>

#include "filter.h"

namespace OpticalFlowTest
{
  struct Sample
  {
    Sample()
      : totalX(0), totalY(0), timestampMs(0), healthy(false)
    {
    }

    Filter::OpticalFlowSample flow;
    int32_t totalX;
    int32_t totalY;
    unsigned long timestampMs;
    bool healthy;
  };

  void begin();
  void loop();
  bool isHealthy();
  Sample latestSample();
}

#endif
