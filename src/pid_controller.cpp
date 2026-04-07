#include "pid_controller.h"

namespace
{
  float clampValue(float value, float minimum, float maximum)
  {
    if (value < minimum)
    {
      return minimum;
    }

    if (value > maximum)
    {
      return maximum;
    }

    return value;
  }

  PidController::Gains makeGains(float kp, float ki, float kd,
                                 float integratorMin, float integratorMax,
                                 float outputMin, float outputMax)
  {
    PidController::Gains gains;
    gains.kp = kp;
    gains.ki = ki;
    gains.kd = kd;
    gains.integratorMin = integratorMin;
    gains.integratorMax = integratorMax;
    gains.outputMin = outputMin;
    gains.outputMax = outputMax;
    return gains;
  }
}

namespace PidController
{
  AxisPid::AxisPid(const Gains& gains)
    : gains_(gains), integrator_(0.0f), previousError_(0.0f), initialized_(false)
  {
  }

  void AxisPid::setGains(const Gains& gains)
  {
    gains_ = gains;
  }

  void AxisPid::reset()
  {
    integrator_ = 0.0f;
    previousError_ = 0.0f;
    initialized_ = false;
  }

  float AxisPid::update(float target, float measurement, float dtSeconds)
  {
    if (dtSeconds <= 0.0f)
    {
      return 0.0f;
    }

    const float error = target - measurement;
    integrator_ += error * dtSeconds * gains_.ki;
    integrator_ = clampValue(integrator_, gains_.integratorMin, gains_.integratorMax);

    float derivative = 0.0f;
    if (initialized_)
    {
      derivative = (error - previousError_) / dtSeconds;
    }
    else
    {
      initialized_ = true;
    }

    previousError_ = error;

    const float output = (gains_.kp * error) + integrator_ + (gains_.kd * derivative);
    return clampValue(output, gains_.outputMin, gains_.outputMax);
  }

  HoverPid::HoverPid()
    : velocityXPid_(makeGains(0.12f, 0.02f, 0.01f, -5.0f, 5.0f, -8.0f, 8.0f)),
      velocityYPid_(makeGains(0.12f, 0.02f, 0.01f, -5.0f, 5.0f, -8.0f, 8.0f)),
      rollPid_(makeGains(4.5f, 0.2f, 0.15f, -50.0f, 50.0f, -250.0f, 250.0f)),
      pitchPid_(makeGains(4.5f, 0.2f, 0.15f, -50.0f, 50.0f, -250.0f, 250.0f)),
      yawPid_(makeGains(1.8f, 0.05f, 0.02f, -30.0f, 30.0f, -120.0f, 120.0f)),
      idleThrottleUs_(1000.0f),
      hoverThrottleUs_(1180.0f),
      takeoffRampRateUsPerSec_(140.0f),
      currentThrottleUs_(1000.0f)
  {
  }

  void HoverPid::reset()
  {
    velocityXPid_.reset();
    velocityYPid_.reset();
    rollPid_.reset();
    pitchPid_.reset();
    yawPid_.reset();
    currentThrottleUs_ = idleThrottleUs_;
  }

  void HoverPid::configureRateGains(const Gains& rollPitchGains, const Gains& yawGains)
  {
    rollPid_.setGains(rollPitchGains);
    pitchPid_.setGains(rollPitchGains);
    yawPid_.setGains(yawGains);
  }

  void HoverPid::configureVelocityGains(const Gains& velocityGains)
  {
    velocityXPid_.setGains(velocityGains);
    velocityYPid_.setGains(velocityGains);
  }

  void HoverPid::configureTakeoff(float idleThrottleUs, float hoverThrottleUs, float takeoffRampRateUsPerSec)
  {
    idleThrottleUs_ = idleThrottleUs;
    hoverThrottleUs_ = hoverThrottleUs;
    takeoffRampRateUsPerSec_ = takeoffRampRateUsPerSec;
    currentThrottleUs_ = clampValue(currentThrottleUs_, idleThrottleUs_, hoverThrottleUs_);
  }

  HoverOutput HoverPid::update(const HoverTargets& targets, const Filter::HoverEstimate& estimate, float dtSeconds)
  {
    HoverOutput output;

    if (targets.takeoffEnabled)
    {
      currentThrottleUs_ += takeoffRampRateUsPerSec_ * dtSeconds;
      currentThrottleUs_ = clampValue(currentThrottleUs_, idleThrottleUs_, targets.takeoffThrottle);
    }
    else
    {
      currentThrottleUs_ = clampValue(targets.baseThrottleUs, idleThrottleUs_, 2000.0f);
    }

    float desiredRollDeg = targets.rollDeg;
    float desiredPitchDeg = targets.pitchDeg;

    if (estimate.opticalFlowHealthy)
    {
      desiredRollDeg += velocityYPid_.update(targets.velocityY, estimate.velocityY, dtSeconds);
      desiredPitchDeg += velocityXPid_.update(targets.velocityX, estimate.velocityX, dtSeconds);
      output.opticalFlowUsed = true;
    }

    output.rollCorrection = rollPid_.update(desiredRollDeg, estimate.attitude.rollDeg, dtSeconds);
    output.pitchCorrection = pitchPid_.update(desiredPitchDeg, estimate.attitude.pitchDeg, dtSeconds);
    output.yawCorrection = yawPid_.update(targets.yawRateDegPerSec, estimate.attitude.yawRateDegPerSec, dtSeconds);
    output.throttleUs = clampValue(currentThrottleUs_, idleThrottleUs_, 2000.0f);

    return output;
  }
}
