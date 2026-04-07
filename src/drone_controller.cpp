#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

#include "acclgyr.h"
#include "drone_controller.h"
#include "filter.h"
#include "optical_flow_test.h"
#include "pid_controller.h"

namespace
{
  const char* ssid = "ESP32_DRONE";
  const char* password = "12345678";

  WebServer server(80);
  Preferences preferences;
  PidController::HoverPid hoverPid;
  Filter::HoverStateFilter hoverStateFilter;
  Filter::HoverEstimate hoverEstimate;
  bool hoverEstimateValid = false;

  int escPins[4] = {14, 27, 26, 25};
  int pwmChannel[4] = {0, 1, 2, 3};

  const int pwmFreq = 50;
  const int pwmResolution = 16;
  const unsigned long controlTimeoutMs = 750;
  const unsigned long stabilizationLoopIntervalMs = 20;
  const float maxStickAngleDeg = 10.0f;
  const float maxStickVelocity = 140.0f;
  const float maxYawRateDegPerSec = 120.0f;
  const int stickDeadband = 12;
  const float opticalFlowForwardFromSensorY = -1.0f;
  const float opticalFlowRightFromSensorX = 1.0f;
  const float imuRollToControllerSign = 1.0f;
  const float imuPitchToControllerSign = -1.0f;
  const float imuYawRateToControllerSign = 1.0f;

  int throttle = 1000;
  int pitch = 0;
  int roll = 0;
  int yaw = 0;
  bool isArmed = false;
  bool killSwitchActive = false;
  bool failsafeTriggered = false;
  unsigned long lastControlMs = 0;
  unsigned long lastStabilizationLoopMs = 0;

  int motorSpeed[4];

  struct StoredPidSettings
  {
    PidController::Gains rollPitch;
    PidController::Gains yaw;
    PidController::Gains velocity;
    float idleThrottleUs;
    float hoverThrottleUs;
    float takeoffRampRateUsPerSec;
    float takeoffThrottleUs;
  };

  float clampFloat(float value, float minimum, float maximum)
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

  StoredPidSettings makeDefaultPidSettings()
  {
    StoredPidSettings defaults;
    defaults.rollPitch = makeGains(4.5f, 0.2f, 0.15f, -50.0f, 50.0f, -250.0f, 250.0f);
    defaults.yaw = makeGains(1.8f, 0.05f, 0.02f, -30.0f, 30.0f, -120.0f, 120.0f);
    defaults.velocity = makeGains(0.12f, 0.02f, 0.01f, -5.0f, 5.0f, -8.0f, 8.0f);
    defaults.idleThrottleUs = 1000.0f;
    defaults.hoverThrottleUs = 1180.0f;
    defaults.takeoffRampRateUsPerSec = 140.0f;
    defaults.takeoffThrottleUs = 1180.0f;
    return defaults;
  }

  StoredPidSettings pidSettings = makeDefaultPidSettings();

  void applyPidSettings()
  {
    hoverPid.configureRateGains(pidSettings.rollPitch, pidSettings.yaw);
    hoverPid.configureVelocityGains(pidSettings.velocity);
    hoverPid.configureTakeoff(pidSettings.idleThrottleUs,
                              pidSettings.hoverThrottleUs,
                              pidSettings.takeoffRampRateUsPerSec);
  }

  void loadPidSettings()
  {
    const StoredPidSettings defaults = makeDefaultPidSettings();
    pidSettings = defaults;

    if (!preferences.begin("drone-pid", true))
    {
      applyPidSettings();
      return;
    }

    pidSettings.rollPitch.kp = preferences.getFloat("rpkp", defaults.rollPitch.kp);
    pidSettings.rollPitch.ki = preferences.getFloat("rpki", defaults.rollPitch.ki);
    pidSettings.rollPitch.kd = preferences.getFloat("rpkd", defaults.rollPitch.kd);
    pidSettings.yaw.kp = preferences.getFloat("ykp", defaults.yaw.kp);
    pidSettings.yaw.ki = preferences.getFloat("yki", defaults.yaw.ki);
    pidSettings.yaw.kd = preferences.getFloat("ykd", defaults.yaw.kd);
    pidSettings.velocity.kp = preferences.getFloat("vkp", defaults.velocity.kp);
    pidSettings.velocity.ki = preferences.getFloat("vki", defaults.velocity.ki);
    pidSettings.velocity.kd = preferences.getFloat("vkd", defaults.velocity.kd);
    pidSettings.idleThrottleUs = preferences.getFloat("idle", defaults.idleThrottleUs);
    pidSettings.hoverThrottleUs = preferences.getFloat("hover", defaults.hoverThrottleUs);
    pidSettings.takeoffRampRateUsPerSec = preferences.getFloat("ramp", defaults.takeoffRampRateUsPerSec);
    pidSettings.takeoffThrottleUs = preferences.getFloat("takeoff", defaults.takeoffThrottleUs);

    preferences.end();

    pidSettings.rollPitch.kp = clampFloat(pidSettings.rollPitch.kp, 0.0f, 50.0f);
    pidSettings.rollPitch.ki = clampFloat(pidSettings.rollPitch.ki, 0.0f, 10.0f);
    pidSettings.rollPitch.kd = clampFloat(pidSettings.rollPitch.kd, 0.0f, 10.0f);
    pidSettings.yaw.kp = clampFloat(pidSettings.yaw.kp, 0.0f, 20.0f);
    pidSettings.yaw.ki = clampFloat(pidSettings.yaw.ki, 0.0f, 10.0f);
    pidSettings.yaw.kd = clampFloat(pidSettings.yaw.kd, 0.0f, 10.0f);
    pidSettings.velocity.kp = clampFloat(pidSettings.velocity.kp, 0.0f, 10.0f);
    pidSettings.velocity.ki = clampFloat(pidSettings.velocity.ki, 0.0f, 10.0f);
    pidSettings.velocity.kd = clampFloat(pidSettings.velocity.kd, 0.0f, 10.0f);
    pidSettings.idleThrottleUs = clampFloat(pidSettings.idleThrottleUs, 1000.0f, 1400.0f);
    pidSettings.hoverThrottleUs = clampFloat(pidSettings.hoverThrottleUs, pidSettings.idleThrottleUs, 2000.0f);
    pidSettings.takeoffThrottleUs = clampFloat(pidSettings.takeoffThrottleUs, pidSettings.idleThrottleUs, 2000.0f);
    pidSettings.takeoffRampRateUsPerSec = clampFloat(pidSettings.takeoffRampRateUsPerSec, 1.0f, 1000.0f);

    applyPidSettings();
  }

  bool savePidSettings()
  {
    if (!preferences.begin("drone-pid", false))
    {
      return false;
    }

    preferences.putFloat("rpkp", pidSettings.rollPitch.kp);
    preferences.putFloat("rpki", pidSettings.rollPitch.ki);
    preferences.putFloat("rpkd", pidSettings.rollPitch.kd);
    preferences.putFloat("ykp", pidSettings.yaw.kp);
    preferences.putFloat("yki", pidSettings.yaw.ki);
    preferences.putFloat("ykd", pidSettings.yaw.kd);
    preferences.putFloat("vkp", pidSettings.velocity.kp);
    preferences.putFloat("vki", pidSettings.velocity.ki);
    preferences.putFloat("vkd", pidSettings.velocity.kd);
    preferences.putFloat("idle", pidSettings.idleThrottleUs);
    preferences.putFloat("hover", pidSettings.hoverThrottleUs);
    preferences.putFloat("ramp", pidSettings.takeoffRampRateUsPerSec);
    preferences.putFloat("takeoff", pidSettings.takeoffThrottleUs);
    preferences.end();
    return true;
  }

  float argToFloat(const char* name, float fallback)
  {
    if (!server.hasArg(name))
    {
      return fallback;
    }

    return server.arg(name).toFloat();
  }

  String pidSettingsJson()
  {
    String json = "{";
    json += "\"rollPitchKp\":" + String(pidSettings.rollPitch.kp, 4) + ",";
    json += "\"rollPitchKi\":" + String(pidSettings.rollPitch.ki, 4) + ",";
    json += "\"rollPitchKd\":" + String(pidSettings.rollPitch.kd, 4) + ",";
    json += "\"yawKp\":" + String(pidSettings.yaw.kp, 4) + ",";
    json += "\"yawKi\":" + String(pidSettings.yaw.ki, 4) + ",";
    json += "\"yawKd\":" + String(pidSettings.yaw.kd, 4) + ",";
    json += "\"velocityKp\":" + String(pidSettings.velocity.kp, 4) + ",";
    json += "\"velocityKi\":" + String(pidSettings.velocity.ki, 4) + ",";
    json += "\"velocityKd\":" + String(pidSettings.velocity.kd, 4) + ",";
    json += "\"idleThrottleUs\":" + String(pidSettings.idleThrottleUs, 2) + ",";
    json += "\"hoverThrottleUs\":" + String(pidSettings.hoverThrottleUs, 2) + ",";
    json += "\"takeoffRampRateUsPerSec\":" + String(pidSettings.takeoffRampRateUsPerSec, 2) + ",";
    json += "\"takeoffThrottleUs\":" + String(pidSettings.takeoffThrottleUs, 2) + ",";
    json += "\"armed\":" + String(isArmed ? "true" : "false") + ",";
    json += "\"killSwitchActive\":" + String(killSwitchActive ? "true" : "false") + ",";
    json += "\"failsafeTriggered\":" + String(failsafeTriggered ? "true" : "false");
    json += "}";
    return json;
  }

  float normalizedStick(int value)
  {
    if (abs(value) <= stickDeadband)
    {
      return 0.0f;
    }

    return clampFloat(static_cast<float>(value) / 300.0f, -1.0f, 1.0f);
  }

  uint32_t usToDuty(int us)
  {
    return (us * 65535) / 20000;
  }

  void mixMotors(int baseThrottleUs, int pitchCommand, int rollCommand, int yawCommand)
  {
    motorSpeed[0] = baseThrottleUs - pitchCommand - rollCommand - yawCommand;
    motorSpeed[1] = baseThrottleUs - pitchCommand + rollCommand + yawCommand;
    motorSpeed[2] = baseThrottleUs + pitchCommand - rollCommand + yawCommand;
    motorSpeed[3] = baseThrottleUs + pitchCommand + rollCommand - yawCommand;

    for (int i = 0; i < 4; i++)
    {
      motorSpeed[i] = constrain(motorSpeed[i], 1000, 2000);
      ledcWrite(pwmChannel[i], usToDuty(motorSpeed[i]));
    }
  }

  void writeAllMotors(int pulseUs)
  {
    for (int i = 0; i < 4; i++)
    {
      motorSpeed[i] = pulseUs;
      ledcWrite(pwmChannel[i], usToDuty(pulseUs));
    }
  }

  void resetControls()
  {
    throttle = 1000;
    pitch = 0;
    roll = 0;
    yaw = 0;
  }

  void disarmMotors()
  {
    isArmed = false;
    resetControls();
    hoverPid.reset();
    hoverStateFilter.reset();
    hoverEstimateValid = false;
    lastStabilizationLoopMs = 0;
    writeAllMotors(1000);
  }

  void killMotors()
  {
    killSwitchActive = true;
    lastControlMs = 0;
    disarmMotors();
  }

  bool armMotors()
  {
    if (throttle != 1000)
    {
      return false;
    }

    killSwitchActive = false;
    failsafeTriggered = false;
    lastControlMs = millis();
    hoverPid.reset();
    hoverStateFilter.reset();
    hoverEstimateValid = false;
    lastStabilizationLoopMs = 0;
    isArmed = true;
    writeAllMotors(1000);
    return true;
  }

  void updateMotors()
  {
    if (!isArmed || killSwitchActive)
    {
      writeAllMotors(1000);
      return;
    }

    mixMotors(throttle, pitch, roll, yaw);
  }

  void runStabilizedControlLoop()
  {
    if (!isArmed || killSwitchActive)
    {
      return;
    }

    const unsigned long now = millis();
    if (lastStabilizationLoopMs != 0 && (now - lastStabilizationLoopMs) < stabilizationLoopIntervalMs)
    {
      return;
    }

    const float dtSeconds = (lastStabilizationLoopMs == 0)
      ? (static_cast<float>(stabilizationLoopIntervalMs) / 1000.0f)
      : (static_cast<float>(now - lastStabilizationLoopMs) / 1000.0f);
    lastStabilizationLoopMs = now;

    const AcclGyr::Sample imuSample = AcclGyr::latestSample();
    if (!imuSample.valid)
    {
      updateMotors();
      return;
    }

    const OpticalFlowTest::Sample rawFlowSample = OpticalFlowTest::latestSample();
    Filter::OpticalFlowSample mappedFlowSample;
    mappedFlowSample.valid = rawFlowSample.flow.valid;
    mappedFlowSample.quality = rawFlowSample.flow.quality;
    mappedFlowSample.velocityX = rawFlowSample.flow.velocityY * opticalFlowForwardFromSensorY;
    mappedFlowSample.velocityY = rawFlowSample.flow.velocityX * opticalFlowRightFromSensorX;

    hoverEstimate = hoverStateFilter.update(imuSample.imu, mappedFlowSample, dtSeconds);
    hoverEstimate.attitude.rollDeg *= imuRollToControllerSign;
    hoverEstimate.attitude.pitchDeg *= imuPitchToControllerSign;
    hoverEstimate.attitude.yawRateDegPerSec *= imuYawRateToControllerSign;
    hoverEstimateValid = true;

    PidController::HoverTargets targets;
    targets.rollDeg = normalizedStick(roll) * maxStickAngleDeg;
    targets.pitchDeg = normalizedStick(pitch) * maxStickAngleDeg;
    targets.yawRateDegPerSec = normalizedStick(yaw) * maxYawRateDegPerSec;
    targets.velocityX = normalizedStick(pitch) * maxStickVelocity;
    targets.velocityY = normalizedStick(roll) * maxStickVelocity;
    targets.baseThrottleUs = static_cast<float>(throttle);
    targets.takeoffThrottle = pidSettings.takeoffThrottleUs;
    targets.takeoffEnabled = false;

    const PidController::HoverOutput output = hoverPid.update(targets, hoverEstimate, dtSeconds);

    mixMotors(static_cast<int>(output.throttleUs),
              static_cast<int>(output.pitchCorrection),
              static_cast<int>(output.rollCorrection),
              static_cast<int>(output.yawCorrection));
  }

  String webpage()
  {
    return R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <style>
    :root {
      --bg: #0f172a;
      --panel-border: #334155;
      --text: #e5e7eb;
      --muted: #94a3b8;
      --arm: #15803d;
      --disarm: #475569;
      --kill: #b91c1c;
      --apply: #0369a1;
      --save: #0f766e;
      --reload: #475569;
    }

    * {
      box-sizing: border-box;
      -webkit-tap-highlight-color: transparent;
    }

    body {
      margin: 0;
      min-height: 100vh;
      font-family: Arial, sans-serif;
      background:
        radial-gradient(circle at top, rgba(56, 189, 248, 0.18), transparent 28%),
        linear-gradient(180deg, #020617 0%, #0f172a 100%);
      color: var(--text);
      overflow-x: hidden;
      overflow-y: auto;
    }

    .layout {
      display: flex;
      flex-direction: column;
      min-height: 100vh;
      padding: 14px;
      gap: 12px;
    }

    .topbar,
    .value-panel,
    .joystick-card,
    .tuning-card {
      background: rgba(15, 23, 42, 0.86);
      border: 1px solid var(--panel-border);
      border-radius: 18px;
      backdrop-filter: blur(10px);
    }

    .topbar {
      padding: 14px;
    }

    .title {
      margin: 0 0 8px 0;
      font-size: 24px;
      text-align: center;
      letter-spacing: 1px;
    }

    .status {
      text-align: center;
      font-weight: bold;
      color: #f8fafc;
      margin-bottom: 12px;
    }

    .button-row {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
    }

    button {
      border: none;
      border-radius: 14px;
      padding: 14px 10px;
      color: white;
      font-size: 15px;
      font-weight: bold;
      letter-spacing: 0.5px;
      touch-action: manipulation;
    }

    .arm-btn { background: var(--arm); }
    .disarm-btn { background: var(--disarm); }
    .kill-btn { background: var(--kill); }
    .apply-btn { background: var(--apply); }
    .save-btn { background: var(--save); }
    .reload-btn { background: var(--reload); }

    .value-panel {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 8px;
      padding: 12px;
      text-align: center;
    }

    .value-box {
      padding: 8px 4px;
      border-radius: 12px;
      background: rgba(30, 41, 59, 0.72);
    }

    .value-box span {
      display: block;
    }

    .value-label {
      color: var(--muted);
      font-size: 12px;
      margin-bottom: 4px;
      text-transform: uppercase;
    }

    .value-number {
      font-size: 18px;
      font-weight: bold;
    }

    .sticks {
      flex: 1;
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
      align-items: stretch;
    }

    .joystick-card {
      padding: 12px;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      gap: 10px;
    }

    .joystick-title {
      font-size: 14px;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 1px;
    }

    .joystick {
      position: relative;
      width: min(42vw, 260px);
      height: min(42vw, 260px);
      max-width: 260px;
      max-height: 260px;
      border-radius: 50%;
      border: 2px solid rgba(148, 163, 184, 0.35);
      background:
        radial-gradient(circle, rgba(56, 189, 248, 0.12), rgba(15, 23, 42, 0.8) 62%),
        linear-gradient(180deg, rgba(15, 23, 42, 0.95), rgba(2, 6, 23, 0.95));
      touch-action: none;
      overflow: hidden;
    }

    .joystick::before,
    .joystick::after {
      content: "";
      position: absolute;
      background: rgba(148, 163, 184, 0.22);
    }

    .joystick::before {
      top: 50%;
      left: 12%;
      right: 12%;
      height: 1px;
      transform: translateY(-50%);
    }

    .joystick::after {
      left: 50%;
      top: 12%;
      bottom: 12%;
      width: 1px;
      transform: translateX(-50%);
    }

    .knob {
      position: absolute;
      left: 50%;
      top: 50%;
      width: 34%;
      height: 34%;
      border-radius: 50%;
      transform: translate(-50%, -50%);
      background:
        radial-gradient(circle at 30% 30%, #f8fafc, #38bdf8 38%, #0f172a 100%);
      box-shadow: 0 0 22px rgba(56, 189, 248, 0.45);
    }

    .hint {
      font-size: 12px;
      color: var(--muted);
      text-align: center;
      line-height: 1.4;
    }

    .tuning-card {
      padding: 14px;
    }

    .tuning-title {
      font-size: 14px;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-bottom: 10px;
    }

    .tuning-note,
    .pid-status {
      font-size: 12px;
      color: var(--muted);
      line-height: 1.4;
    }

    .pid-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
      margin-top: 12px;
    }

    .pid-group {
      padding: 12px;
      border-radius: 14px;
      background: rgba(2, 6, 23, 0.35);
      border: 1px solid rgba(148, 163, 184, 0.15);
    }

    .pid-group-title {
      font-size: 13px;
      margin-bottom: 10px;
      color: #f8fafc;
    }

    .field-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 8px;
    }

    .field-grid.two {
      grid-template-columns: repeat(2, minmax(0, 1fr));
    }

    .field {
      display: flex;
      flex-direction: column;
      gap: 6px;
    }

    .field label {
      font-size: 11px;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.6px;
    }

    .field input {
      width: 100%;
      border-radius: 10px;
      border: 1px solid rgba(148, 163, 184, 0.2);
      background: rgba(15, 23, 42, 0.95);
      color: var(--text);
      padding: 10px;
      font-size: 14px;
    }

    .pid-actions {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 10px;
      margin-top: 14px;
    }

    @media (max-width: 820px) {
      .sticks,
      .pid-grid,
      .field-grid,
      .field-grid.two,
      .pid-actions,
      .button-row {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <div class="layout">
    <div class="topbar">
      <h2 class="title">ESP32 Drone RC</h2>
      <div class="status" id="status">DISARMED</div>
      <div class="button-row">
        <button class="arm-btn" onclick="sendCommand('arm')">ARM</button>
        <button class="disarm-btn" onclick="sendCommand('disarm')">DISARM</button>
        <button class="kill-btn" onclick="sendCommand('kill')">KILL SWITCH</button>
      </div>
    </div>

    <div class="value-panel">
      <div class="value-box">
        <span class="value-label">Throttle</span>
        <span class="value-number" id="throttleValue">1000</span>
      </div>
      <div class="value-box">
        <span class="value-label">Yaw</span>
        <span class="value-number" id="yawValue">0</span>
      </div>
      <div class="value-box">
        <span class="value-label">Pitch</span>
        <span class="value-number" id="pitchValue">0</span>
      </div>
      <div class="value-box">
        <span class="value-label">Roll</span>
        <span class="value-number" id="rollValue">0</span>
      </div>
    </div>

    <div class="sticks">
      <div class="joystick-card">
        <div class="joystick-title">Left Stick</div>
        <div class="joystick" id="leftPad">
          <div class="knob" id="leftKnob"></div>
        </div>
        <div class="hint">Throttle stays where you leave it. Yaw returns to center.</div>
      </div>

      <div class="joystick-card">
        <div class="joystick-title">Right Stick</div>
        <div class="joystick" id="rightPad">
          <div class="knob" id="rightKnob"></div>
        </div>
        <div class="hint">Pitch and roll return to center when released.</div>
      </div>
    </div>

    <div class="tuning-card">
      <div class="tuning-title">PID And Takeoff Tuning</div>
      <div class="tuning-note">Assumed frame setup: optical flow sensor facing downward, MPU X-axis facing forward. Roll tuning is left/right stabilization. Pitch tuning is forward/back stabilization. Verify axis signs with props removed before flight.</div>

      <div class="pid-grid">
        <div class="pid-group">
          <div class="pid-group-title">Roll / Pitch</div>
          <div class="field-grid">
            <div class="field">
              <label for="rollPitchKp">KP</label>
              <input id="rollPitchKp" type="number" step="0.01">
            </div>
            <div class="field">
              <label for="rollPitchKi">KI</label>
              <input id="rollPitchKi" type="number" step="0.01">
            </div>
            <div class="field">
              <label for="rollPitchKd">KD</label>
              <input id="rollPitchKd" type="number" step="0.01">
            </div>
          </div>
        </div>

        <div class="pid-group">
          <div class="pid-group-title">Yaw</div>
          <div class="field-grid">
            <div class="field">
              <label for="yawKp">KP</label>
              <input id="yawKp" type="number" step="0.01">
            </div>
            <div class="field">
              <label for="yawKi">KI</label>
              <input id="yawKi" type="number" step="0.01">
            </div>
            <div class="field">
              <label for="yawKd">KD</label>
              <input id="yawKd" type="number" step="0.01">
            </div>
          </div>
        </div>

        <div class="pid-group">
          <div class="pid-group-title">Optical Flow Velocity</div>
          <div class="field-grid">
            <div class="field">
              <label for="velocityKp">KP</label>
              <input id="velocityKp" type="number" step="0.01">
            </div>
            <div class="field">
              <label for="velocityKi">KI</label>
              <input id="velocityKi" type="number" step="0.01">
            </div>
            <div class="field">
              <label for="velocityKd">KD</label>
              <input id="velocityKd" type="number" step="0.01">
            </div>
          </div>
        </div>

        <div class="pid-group">
          <div class="pid-group-title">Throttle / Takeoff</div>
          <div class="field-grid two">
            <div class="field">
              <label for="idleThrottleUs">Idle (us)</label>
              <input id="idleThrottleUs" type="number" step="1">
            </div>
            <div class="field">
              <label for="hoverThrottleUs">Hover (us)</label>
              <input id="hoverThrottleUs" type="number" step="1">
            </div>
            <div class="field">
              <label for="takeoffRampRateUsPerSec">Ramp (us/s)</label>
              <input id="takeoffRampRateUsPerSec" type="number" step="1">
            </div>
            <div class="field">
              <label for="takeoffThrottleUs">Takeoff Target (us)</label>
              <input id="takeoffThrottleUs" type="number" step="1">
            </div>
          </div>
        </div>
      </div>

      <div class="pid-actions">
        <button class="apply-btn" onclick="applyPidTuning()">APPLY</button>
        <button class="save-btn" onclick="savePidTuning()">SAVE</button>
        <button class="reload-btn" onclick="loadPidTuning(true)">RELOAD</button>
      </div>
      <div class="pid-status" id="pidStatus">Stored values load automatically on boot.</div>
    </div>
  </div>

  <script>
    const leftState = { x: 0, y: 1 };
    const rightState = { x: 0, y: 0 };
    let requestInFlight = false;
    let resendNeeded = false;

    function clamp(value, min, max) {
      return Math.min(max, Math.max(min, value));
    }

    function normalize(value) {
      return Math.round(value * 1000) / 1000;
    }

    function getControlValues() {
      return {
        throttle: clamp(Math.round(1000 + ((1 - leftState.y) * 500)), 1000, 2000),
        yaw: clamp(Math.round(leftState.x * 300), -300, 300),
        pitch: clamp(Math.round(-rightState.y * 300), -300, 300),
        roll: clamp(Math.round(rightState.x * 300), -300, 300)
      };
    }

    function updateReadout() {
      const controls = getControlValues();
      document.getElementById('throttleValue').innerText = controls.throttle;
      document.getElementById('yawValue').innerText = controls.yaw;
      document.getElementById('pitchValue').innerText = controls.pitch;
      document.getElementById('rollValue').innerText = controls.roll;
    }

    function renderStick(padId, knobId, state) {
      const pad = document.getElementById(padId);
      const knob = document.getElementById(knobId);
      const radius = (pad.clientWidth - knob.clientWidth) / 2;
      const translateX = state.x * radius;
      const translateY = state.y * radius;
      knob.style.transform = 'translate(calc(-50% + ' + translateX + 'px), calc(-50% + ' + translateY + 'px))';
    }

    function scheduleControlSend() {
      if (requestInFlight) {
        resendNeeded = true;
        return;
      }

      const controls = getControlValues();
      requestInFlight = true;
      fetch('/control?throttle=' + controls.throttle + '&pitch=' + controls.pitch + '&roll=' + controls.roll + '&yaw=' + controls.yaw)
        .catch(() => {})
        .finally(() => {
          requestInFlight = false;
          if (resendNeeded) {
            resendNeeded = false;
            scheduleControlSend();
          }
        });
    }

    function renderAll() {
      renderStick('leftPad', 'leftKnob', leftState);
      renderStick('rightPad', 'rightKnob', rightState);
      updateReadout();
      scheduleControlSend();
    }

    function resetJoysticks() {
      leftState.x = 0;
      leftState.y = 1;
      rightState.x = 0;
      rightState.y = 0;
      renderAll();
    }

    function setStatus(text) {
      document.getElementById('status').innerText = text;
    }

    function setPidStatus(text) {
      document.getElementById('pidStatus').innerText = text;
    }

    function inputValue(id) {
      return Number(document.getElementById(id).value);
    }

    function setInputValue(id, value, decimals) {
      document.getElementById(id).value = Number(value).toFixed(decimals);
    }

    function populatePidForm(data) {
      setInputValue('rollPitchKp', data.rollPitchKp, 3);
      setInputValue('rollPitchKi', data.rollPitchKi, 3);
      setInputValue('rollPitchKd', data.rollPitchKd, 3);
      setInputValue('yawKp', data.yawKp, 3);
      setInputValue('yawKi', data.yawKi, 3);
      setInputValue('yawKd', data.yawKd, 3);
      setInputValue('velocityKp', data.velocityKp, 3);
      setInputValue('velocityKi', data.velocityKi, 3);
      setInputValue('velocityKd', data.velocityKd, 3);
      setInputValue('idleThrottleUs', data.idleThrottleUs, 0);
      setInputValue('hoverThrottleUs', data.hoverThrottleUs, 0);
      setInputValue('takeoffRampRateUsPerSec', data.takeoffRampRateUsPerSec, 0);
      setInputValue('takeoffThrottleUs', data.takeoffThrottleUs, 0);
    }

    function pidPayload() {
      return new URLSearchParams({
        rollPitchKp: inputValue('rollPitchKp'),
        rollPitchKi: inputValue('rollPitchKi'),
        rollPitchKd: inputValue('rollPitchKd'),
        yawKp: inputValue('yawKp'),
        yawKi: inputValue('yawKi'),
        yawKd: inputValue('yawKd'),
        velocityKp: inputValue('velocityKp'),
        velocityKi: inputValue('velocityKi'),
        velocityKd: inputValue('velocityKd'),
        idleThrottleUs: inputValue('idleThrottleUs'),
        hoverThrottleUs: inputValue('hoverThrottleUs'),
        takeoffRampRateUsPerSec: inputValue('takeoffRampRateUsPerSec'),
        takeoffThrottleUs: inputValue('takeoffThrottleUs')
      });
    }

    function applyPidTuning() {
      fetch('/pid', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: pidPayload().toString()
      })
        .then(response => response.json())
        .then(data => {
          populatePidForm(data);
          setPidStatus('PID values applied in RAM. Press SAVE to keep them after reboot.');
        })
        .catch(() => setPidStatus('Failed to apply PID values'));
    }

    function savePidTuning() {
      fetch('/pid', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: pidPayload().toString()
      })
        .then(response => response.json())
        .then(data => {
          populatePidForm(data);
          return fetch('/pid/save', { method: 'POST' });
        })
        .then(response => response.text())
        .then(text => setPidStatus(text))
        .catch(() => setPidStatus('Failed to save PID values'));
    }

    function loadPidTuning(showMessage) {
      fetch('/pid')
        .then(response => response.json())
        .then(data => {
          populatePidForm(data);
          if (showMessage) {
            setPidStatus('Loaded stored PID values');
          }
        })
        .catch(() => setPidStatus('Failed to load PID values'));
    }

    function sendCommand(command) {
      fetch('/command?action=' + command)
        .then(response => response.text().then(text => ({ ok: response.ok, text: text })))
        .then(result => {
          if (command === 'arm' && result.ok && result.text === 'ARMED') {
            setStatus('ARMED');
            return;
          }

          if (command === 'disarm' && result.ok && result.text === 'DISARMED') {
            resetJoysticks();
            setStatus('DISARMED');
            return;
          }

          if (command === 'kill' && result.ok && result.text === 'KILLED') {
            resetJoysticks();
            setStatus('KILL SWITCH ACTIVE');
            return;
          }

          setStatus(result.text);
        })
        .catch(() => setStatus('Connection error'));
    }

    function attachJoystick(config) {
      const pad = document.getElementById(config.padId);
      let activePointerId = null;

      function updateFromPointer(clientX, clientY) {
        const rect = pad.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;
        const radius = rect.width / 2;
        let dx = (clientX - centerX) / radius;
        let dy = (clientY - centerY) / radius;
        const magnitude = Math.sqrt((dx * dx) + (dy * dy));

        if (magnitude > 1) {
          dx /= magnitude;
          dy /= magnitude;
        }

        config.state.x = normalize(dx);
        config.state.y = normalize(dy);
        renderAll();
      }

      pad.addEventListener('pointerdown', event => {
        if (activePointerId !== null) {
          return;
        }

        activePointerId = event.pointerId;
        pad.setPointerCapture(event.pointerId);
        updateFromPointer(event.clientX, event.clientY);
      });

      pad.addEventListener('pointermove', event => {
        if (event.pointerId !== activePointerId) {
          return;
        }

        updateFromPointer(event.clientX, event.clientY);
      });

      function releasePointer(event) {
        if (event.pointerId != activePointerId) {
          return;
        }

        activePointerId = null;
        config.onRelease();
        renderAll();
      }

      pad.addEventListener('pointerup', releasePointer);
      pad.addEventListener('pointercancel', releasePointer);
    }

    attachJoystick({
      padId: 'leftPad',
      state: leftState,
      onRelease: () => {
        leftState.x = 0;
      }
    });

    attachJoystick({
      padId: 'rightPad',
      state: rightState,
      onRelease: () => {
        rightState.x = 0;
        rightState.y = 0;
      }
    });

    window.addEventListener('resize', renderAll);
    window.setInterval(scheduleControlSend, 100);
    loadPidTuning(false);
    resetJoysticks();
  </script>
</body>
</html>
)HTML";
  }

  void handleRoot()
  {
    server.send(200, "text/html", webpage());
  }

  void handlePidGet()
  {
    server.send(200, "application/json", pidSettingsJson());
  }

  void handlePidUpdate()
  {
    pidSettings.rollPitch.kp = clampFloat(argToFloat("rollPitchKp", pidSettings.rollPitch.kp), 0.0f, 50.0f);
    pidSettings.rollPitch.ki = clampFloat(argToFloat("rollPitchKi", pidSettings.rollPitch.ki), 0.0f, 10.0f);
    pidSettings.rollPitch.kd = clampFloat(argToFloat("rollPitchKd", pidSettings.rollPitch.kd), 0.0f, 10.0f);
    pidSettings.yaw.kp = clampFloat(argToFloat("yawKp", pidSettings.yaw.kp), 0.0f, 20.0f);
    pidSettings.yaw.ki = clampFloat(argToFloat("yawKi", pidSettings.yaw.ki), 0.0f, 10.0f);
    pidSettings.yaw.kd = clampFloat(argToFloat("yawKd", pidSettings.yaw.kd), 0.0f, 10.0f);
    pidSettings.velocity.kp = clampFloat(argToFloat("velocityKp", pidSettings.velocity.kp), 0.0f, 10.0f);
    pidSettings.velocity.ki = clampFloat(argToFloat("velocityKi", pidSettings.velocity.ki), 0.0f, 10.0f);
    pidSettings.velocity.kd = clampFloat(argToFloat("velocityKd", pidSettings.velocity.kd), 0.0f, 10.0f);
    pidSettings.idleThrottleUs = clampFloat(argToFloat("idleThrottleUs", pidSettings.idleThrottleUs), 1000.0f, 1400.0f);
    pidSettings.hoverThrottleUs = clampFloat(argToFloat("hoverThrottleUs", pidSettings.hoverThrottleUs), pidSettings.idleThrottleUs, 2000.0f);
    pidSettings.takeoffRampRateUsPerSec = clampFloat(argToFloat("takeoffRampRateUsPerSec", pidSettings.takeoffRampRateUsPerSec), 1.0f, 1000.0f);
    pidSettings.takeoffThrottleUs = clampFloat(argToFloat("takeoffThrottleUs", pidSettings.takeoffThrottleUs), pidSettings.idleThrottleUs, 2000.0f);

    applyPidSettings();
    server.send(200, "application/json", pidSettingsJson());
  }

  void handlePidSave()
  {
    if (!savePidSettings())
    {
      server.send(500, "text/plain", "Failed to save PID values");
      return;
    }

    server.send(200, "text/plain", "PID values saved for next boot");
  }

  void handleControl()
  {
    if (server.hasArg("throttle"))
    {
      throttle = constrain(server.arg("throttle").toInt(), 1000, 2000);
      pitch = constrain(server.arg("pitch").toInt(), -300, 300);
      roll = constrain(server.arg("roll").toInt(), -300, 300);
      yaw = constrain(server.arg("yaw").toInt(), -300, 300);
    }
    else
    {
      String type = server.arg("type");
      int value = server.arg("value").toInt();

      if (type == "throttle") throttle = constrain(value, 1000, 2000);
      if (type == "pitch") pitch = constrain(value, -300, 300);
      if (type == "roll") roll = constrain(value, -300, 300);
      if (type == "yaw") yaw = constrain(value, -300, 300);
    }

    lastControlMs = millis();
    failsafeTriggered = false;
    server.send(200, "text/plain", "OK");
  }

  void handleCommand()
  {
    String action = server.arg("action");

    if (action == "arm")
    {
      if (armMotors())
      {
        server.send(200, "text/plain", "ARMED");
      }
      else
      {
        server.send(400, "text/plain", "Set throttle to 1000 before arming");
      }
      return;
    }

    if (action == "disarm")
    {
      killSwitchActive = false;
      failsafeTriggered = false;
      lastControlMs = 0;
      disarmMotors();
      server.send(200, "text/plain", "DISARMED");
      return;
    }

    if (action == "kill")
    {
      failsafeTriggered = false;
      killMotors();
      server.send(200, "text/plain", "KILLED");
      return;
    }

    server.send(400, "text/plain", "Unknown command");
  }
}

namespace DroneController
{
  void begin()
  {
    Serial.begin(115200);

    for (int i = 0; i < 4; i++)
    {
      ledcSetup(pwmChannel[i], pwmFreq, pwmResolution);
      ledcAttachPin(escPins[i], pwmChannel[i]);
      ledcWrite(pwmChannel[i], usToDuty(1000));
    }

    loadPidSettings();

    delay(3000);

    WiFi.softAP(ssid, password);

    Serial.print("Connect to: ");
    Serial.println(ssid);

    Serial.print("Open browser: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.on("/control", handleControl);
    server.on("/command", handleCommand);
    server.on("/pid", HTTP_GET, handlePidGet);
    server.on("/pid", HTTP_POST, handlePidUpdate);
    server.on("/pid/save", HTTP_POST, handlePidSave);

    server.begin();
  }

  void loop()
  {
    server.handleClient();
    runStabilizedControlLoop();

    if (isArmed && !killSwitchActive && lastControlMs != 0)
    {
      if ((millis() - lastControlMs) > controlTimeoutMs)
      {
        Serial.println("Control link timeout. Failsafe disarm.");
        failsafeTriggered = true;
        lastControlMs = 0;
        disarmMotors();
      }
    }
  }
}
