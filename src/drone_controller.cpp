#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#include "drone_controller.h"

namespace
{
  const char* ssid = "ESP32_DRONE";
  const char* password = "12345678";

  WebServer server(80);

  int escPins[4] = {14, 27, 26, 25};
  int pwmChannel[4] = {0, 1, 2, 3};

  const int pwmFreq = 50;
  const int pwmResolution = 16;

  int throttle = 1000;
  int pitch = 0;
  int roll = 0;
  int yaw = 0;
  bool isArmed = false;
  bool killSwitchActive = false;

  int motorSpeed[4];

  uint32_t usToDuty(int us)
  {
    return (us * 65535) / 20000;
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
    writeAllMotors(1000);
  }

  void killMotors()
  {
    killSwitchActive = true;
    disarmMotors();
  }

  bool armMotors()
  {
    if (throttle != 1000)
    {
      return false;
    }

    killSwitchActive = false;
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

    motorSpeed[0] = throttle - pitch - roll - yaw;
    motorSpeed[1] = throttle - pitch + roll + yaw;
    motorSpeed[2] = throttle + pitch - roll + yaw;
    motorSpeed[3] = throttle + pitch + roll - yaw;

    for (int i = 0; i < 4; i++)
    {
      motorSpeed[i] = constrain(motorSpeed[i], 1000, 2000);
      ledcWrite(pwmChannel[i], usToDuty(motorSpeed[i]));
    }
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
      touch-action: none;
      overflow: hidden;
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
    .joystick-card {
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

    updateMotors();
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
      disarmMotors();
      server.send(200, "text/plain", "DISARMED");
      return;
    }

    if (action == "kill")
    {
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

    delay(3000);

    WiFi.softAP(ssid, password);

    Serial.print("Connect to: ");
    Serial.println(ssid);

    Serial.print("Open browser: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.on("/control", handleControl);
    server.on("/command", handleCommand);

    server.begin();
  }

  void loop()
  {
    server.handleClient();
  }
}
