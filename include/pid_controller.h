#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

#include "filter.h"

namespace PidController
{
  struct Gains
  {
    Gains()
      : kp(0.0f), ki(0.0f), kd(0.0f),
        integratorMin(-200.0f), integratorMax(200.0f),
        outputMin(-400.0f), outputMax(400.0f)
    {
    }

    Gains(float kpValue, float kiValue, float kdValue,
          float integratorMinValue, float integratorMaxValue,
          float outputMinValue, float outputMaxValue)
      : kp(kpValue), ki(kiValue), kd(kdValue),
        integratorMin(integratorMinValue), integratorMax(integratorMaxValue),
        outputMin(outputMinValue), outputMax(outputMaxValue)
    {
    }

    float kp;
    float ki;
    float kd;
    float integratorMin;
    float integratorMax;
    float outputMin;
    float outputMax;
  };

  struct HoverTargets
  {
    HoverTargets()
      : rollDeg(0.0f), pitchDeg(0.0f), yawRateDegPerSec(0.0f),
        velocityX(0.0f), velocityY(0.0f),
        takeoffThrottle(1180.0f), takeoffEnabled(false)
    {
    }

    float rollDeg;
    float pitchDeg;
    float yawRateDegPerSec;
    float velocityX;
    float velocityY;
    float takeoffThrottle;
    bool takeoffEnabled;
  };

  struct HoverOutput
  {
    HoverOutput()
      : throttleUs(1000.0f), pitchCorrection(0.0f), rollCorrection(0.0f),
        yawCorrection(0.0f), opticalFlowUsed(false)
    {
    }

    float throttleUs;
    float pitchCorrection;
    float rollCorrection;
    float yawCorrection;
    bool opticalFlowUsed;
  };

  class AxisPid
  {
  public:
    explicit AxisPid(const Gains& gains = Gains{});

    void setGains(const Gains& gains);
    void reset();
    float update(float target, float measurement, float dtSeconds);

  private:
    Gains gains_;
    float integrator_;
    float previousError_;
    bool initialized_;
  };

  class HoverPid
  {
  public:
    HoverPid();

    void reset();
    void configureRateGains(const Gains& rollPitchGains, const Gains& yawGains);
    void configureVelocityGains(const Gains& velocityGains);
    void configureTakeoff(float idleThrottleUs, float hoverThrottleUs, float takeoffRampRateUsPerSec);
    HoverOutput update(const HoverTargets& targets, const Filter::HoverEstimate& estimate, float dtSeconds);

  private:
    AxisPid velocityXPid_;
    AxisPid velocityYPid_;
    AxisPid rollPid_;
    AxisPid pitchPid_;
    AxisPid yawPid_;

    float idleThrottleUs_;
    float hoverThrottleUs_;
    float takeoffRampRateUsPerSec_;
    float currentThrottleUs_;
  };
}

#endif
