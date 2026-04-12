#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

#include "acclgyr.h"
#include "drone_controller.h"
#include "filter.h"
#include "optical_flow_test.h"
#include "pid_controller.h"
#include "controller_interface.h"
#include "bmp585_test.h"

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

  void handleTelemetry()
  {
    String json = "{";
    
    // Get altitude from barometer if available
    float altitude = 0.0f;
    if (Bmp585Test::isReady())
    {
      Bmp585Test::Sample sample = Bmp585Test::latestSample();
      if (sample.valid)
      {
        altitude = sample.altitudeM;
      }
    }
    
    json += "\"altitude\":" + String(altitude, 2) + ",";
    json += "\"throttle\":" + String(throttle) + ",";
    json += "\"pitch\":" + String(pitch) + ",";
    json += "\"roll\":" + String(roll) + ",";
    json += "\"yaw\":" + String(yaw) + ",";
    json += "\"armed\":" + String(isArmed ? "true" : "false") + ",";
    json += "\"failsafe\":" + String(failsafeTriggered ? "true" : "false");
    json += "}";
    
    server.send(200, "application/json", json);
  }

  void handleControlPost()
  {
    if (server.hasHeader("Content-Type") && server.header("Content-Type") == "application/json")
    {
      String body = server.arg("plain");
      
      // Parse JSON: {"pitch": 0, "roll": 0, "throttle": 1000, "yaw": 0, "armed": 0}
      int throttleVal = 1000, pitchVal = 0, rollVal = 0, yawVal = 0;
      int armedVal = 0;
      
      if (body.indexOf("throttle") >= 0)
      {
        int start = body.indexOf("throttle") + 10;
        int end = body.indexOf(",", start);
        if (end < 0) end = body.indexOf("}", start);
        throttleVal = constrain(body.substring(start, end).toInt() * 2 + 1000, 1000, 2000);
      }
      
      if (body.indexOf("pitch") >= 0)
      {
        int start = body.indexOf("pitch") + 7;
        int end = body.indexOf(",", start);
        if (end < 0) end = body.indexOf("}", start);
        pitchVal = constrain(body.substring(start, end).toInt(), -300, 300);
      }
      
      if (body.indexOf("roll") >= 0)
      {
        int start = body.indexOf("roll") + 6;
        int end = body.indexOf(",", start);
        if (end < 0) end = body.indexOf("}", start);
        rollVal = constrain(body.substring(start, end).toInt(), -300, 300);
      }
      
      if (body.indexOf("yaw") >= 0)
      {
        int start = body.indexOf("yaw") + 5;
        int end = body.indexOf(",", start);
        if (end < 0) end = body.indexOf("}", start);
        yawVal = constrain(body.substring(start, end).toInt(), -300, 300);
      }
      
      if (body.indexOf("armed") >= 0)
      {
        int start = body.indexOf("armed") + 7;
        int end = body.indexOf(",", start);
        if (end < 0) end = body.indexOf("}", start);
        armedVal = body.substring(start, end).toInt();
      }
      
      throttle = throttleVal;
      pitch = pitchVal;
      roll = rollVal;
      yaw = yawVal;
      
      if (armedVal && !isArmed)
      {
        armMotors();
      }
      else if (!armedVal && isArmed)
      {
        disarmMotors();
      }
      
      lastControlMs = millis();
      failsafeTriggered = false;
      
      server.send(200, "application/json", "{\"status\":\"ok\"}");
    }
    else
    {
      handleControl();
    }
  }

  String webpage()
  {
    return String(HTML_INTERFACE);
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
    server.on("/control", HTTP_GET, handleControl);
    server.on("/control", HTTP_POST, handleControlPost);
    server.on("/telemetry", handleTelemetry);
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
