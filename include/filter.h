#ifndef FILTER_H
#define FILTER_H

#include <Arduino.h>

namespace Filter
{
  struct ImuSample
  {
    ImuSample()
      : accelX(0.0f), accelY(0.0f), accelZ(0.0f),
        gyroXDegPerSec(0.0f), gyroYDegPerSec(0.0f), gyroZDegPerSec(0.0f)
    {
    }

    float accelX;
    float accelY;
    float accelZ;
    float gyroXDegPerSec;
    float gyroYDegPerSec;
    float gyroZDegPerSec;
  };

  struct OpticalFlowSample
  {
    OpticalFlowSample()
      : velocityX(0.0f), velocityY(0.0f), quality(0.0f), valid(false)
    {
    }

    float velocityX;
    float velocityY;
    float quality;
    bool valid;
  };

  struct AttitudeEstimate
  {
    AttitudeEstimate()
      : rollDeg(0.0f), pitchDeg(0.0f), yawRateDegPerSec(0.0f)
    {
    }

    float rollDeg;
    float pitchDeg;
    float yawRateDegPerSec;
  };

  struct HoverEstimate
  {
    HoverEstimate()
      : velocityX(0.0f), velocityY(0.0f), accelerationZ(0.0f), opticalFlowHealthy(false)
    {
    }

    AttitudeEstimate attitude;
    float velocityX;
    float velocityY;
    float accelerationZ;
    bool opticalFlowHealthy;
  };

  class LowPassFilter
  {
  public:
    explicit LowPassFilter(float alpha = 0.15f);

    void setAlpha(float alpha);
    void reset(float value = 0.0f);
    float update(float input);
    float value() const;

  private:
    float alpha_;
    float value_;
    bool initialized_;
  };

  class ComplementaryAngleFilter
  {
  public:
    explicit ComplementaryAngleFilter(float gyroWeight = 0.98f);

    void setGyroWeight(float gyroWeight);
    void reset();
    void update(const ImuSample& sample, float dtSeconds);

    float rollDeg() const;
    float pitchDeg() const;

  private:
    float gyroWeight_;
    float rollDeg_;
    float pitchDeg_;
    bool initialized_;
  };

  class HoverStateFilter
  {
  public:
    HoverStateFilter();

    void reset();
    void configure(float gyroWeight, float velocityAlpha, float accelZAlpha);
    HoverEstimate update(const ImuSample& imuSample, const OpticalFlowSample& flowSample, float dtSeconds);

  private:
    ComplementaryAngleFilter angleFilter_;
    LowPassFilter velocityXFilter_;
    LowPassFilter velocityYFilter_;
    LowPassFilter accelZFilter_;
  };
}

#endif
