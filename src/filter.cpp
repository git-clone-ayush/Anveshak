#include "filter.h"

#include <math.h>

namespace
{
  constexpr float kRadiansToDegrees = 57.2957795f;

  float clampUnit(float value)
  {
    if (value < 0.0f)
    {
      return 0.0f;
    }

    if (value > 1.0f)
    {
      return 1.0f;
    }

    return value;
  }
}

namespace Filter
{
  LowPassFilter::LowPassFilter(float alpha)
    : alpha_(clampUnit(alpha)), value_(0.0f), initialized_(false)
  {
  }

  void LowPassFilter::setAlpha(float alpha)
  {
    alpha_ = clampUnit(alpha);
  }

  void LowPassFilter::reset(float value)
  {
    value_ = value;
    initialized_ = true;
  }

  float LowPassFilter::update(float input)
  {
    if (!initialized_)
    {
      value_ = input;
      initialized_ = true;
      return value_;
    }

    value_ += alpha_ * (input - value_);
    return value_;
  }

  float LowPassFilter::value() const
  {
    return value_;
  }

  ComplementaryAngleFilter::ComplementaryAngleFilter(float gyroWeight)
    : gyroWeight_(gyroWeight), rollDeg_(0.0f), pitchDeg_(0.0f), initialized_(false)
  {
  }

  void ComplementaryAngleFilter::setGyroWeight(float gyroWeight)
  {
    gyroWeight_ = gyroWeight;
  }

  void ComplementaryAngleFilter::reset()
  {
    rollDeg_ = 0.0f;
    pitchDeg_ = 0.0f;
    initialized_ = false;
  }

  void ComplementaryAngleFilter::update(const ImuSample& sample, float dtSeconds)
  {
    const float accelRollDeg = atan2f(sample.accelY, sample.accelZ) * kRadiansToDegrees;
    const float accelPitchDeg = atan2f(-sample.accelX, sqrtf((sample.accelY * sample.accelY) + (sample.accelZ * sample.accelZ))) * kRadiansToDegrees;

    if (!initialized_)
    {
      rollDeg_ = accelRollDeg;
      pitchDeg_ = accelPitchDeg;
      initialized_ = true;
      return;
    }

    const float predictedRoll = rollDeg_ + (sample.gyroXDegPerSec * dtSeconds);
    const float predictedPitch = pitchDeg_ + (sample.gyroYDegPerSec * dtSeconds);
    const float accelWeight = 1.0f - gyroWeight_;

    rollDeg_ = (gyroWeight_ * predictedRoll) + (accelWeight * accelRollDeg);
    pitchDeg_ = (gyroWeight_ * predictedPitch) + (accelWeight * accelPitchDeg);
  }

  float ComplementaryAngleFilter::rollDeg() const
  {
    return rollDeg_;
  }

  float ComplementaryAngleFilter::pitchDeg() const
  {
    return pitchDeg_;
  }

  HoverStateFilter::HoverStateFilter()
    : angleFilter_(0.98f),
      velocityXFilter_(0.2f),
      velocityYFilter_(0.2f),
      accelZFilter_(0.15f)
  {
  }

  void HoverStateFilter::reset()
  {
    angleFilter_.reset();
    velocityXFilter_.reset(0.0f);
    velocityYFilter_.reset(0.0f);
    accelZFilter_.reset(0.0f);
  }

  void HoverStateFilter::configure(float gyroWeight, float velocityAlpha, float accelZAlpha)
  {
    angleFilter_.setGyroWeight(gyroWeight);
    velocityXFilter_.setAlpha(velocityAlpha);
    velocityYFilter_.setAlpha(velocityAlpha);
    accelZFilter_.setAlpha(accelZAlpha);
  }

  HoverEstimate HoverStateFilter::update(const ImuSample& imuSample, const OpticalFlowSample& flowSample, float dtSeconds)
  {
    angleFilter_.update(imuSample, dtSeconds);

    HoverEstimate estimate;
    estimate.attitude.rollDeg = angleFilter_.rollDeg();
    estimate.attitude.pitchDeg = angleFilter_.pitchDeg();
    estimate.attitude.yawRateDegPerSec = imuSample.gyroZDegPerSec;
    estimate.accelerationZ = accelZFilter_.update(imuSample.accelZ);

    if (flowSample.valid)
    {
      estimate.velocityX = velocityXFilter_.update(flowSample.velocityX);
      estimate.velocityY = velocityYFilter_.update(flowSample.velocityY);
      estimate.opticalFlowHealthy = true;
    }
    else
    {
      estimate.velocityX = velocityXFilter_.value();
      estimate.velocityY = velocityYFilter_.value();
      estimate.opticalFlowHealthy = false;
    }

    return estimate;
  }
}
