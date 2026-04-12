# Sensor Fusion Suite - Quick Reference

## Quick Start (5 minutes)

```cpp
#include "sensor_integration.h"

SensorIntegration::SensorManager sensorManager;

void setup() {
  Serial.begin(115200);
  sensorManager.begin();
}

void loop() {
  // This does EVERYTHING:
  // - Reads all sensors
  // - Fuses them
  // - Computes state
  sensorManager.update();
  
  // Get the result
  auto fused = sensorManager.getFusionOutput();
  
  // Use it!
  float roll = fused.state.rollDeg;
  float alt = fused.state.altitudeM;
  float vx = fused.state.velocityXMps;
}
```

---

## Data You Get

After `sensorManager.update()`, access:

```cpp
auto state = sensorManager.getFusionOutput().state;

// ATTITUDE (degrees, degrees/sec)
float roll   = state.rollDeg;
float pitch  = state.pitchDeg;
float yaw    = state.yawDeg;

// VERTICAL (meters, m/s)
float alt    = state.altitudeM;
float vz     = state.verticalVelocityMps;

// HORIZONTAL (meters, m/s)
float x      = state.positionXM;
float y      = state.positionYM;
float vx     = state.velocityXMps;
float vy     = state.velocityYMps;

// CONFIDENCE
float confidence = sensorManager.getFusionOutput().confidence;  // 0-100%
```

---

## Tuning Parameters (Most Important)

```cpp
SensorFusion::FusionConfig cfg;

// Trust gyro more/less for attitude (0-1, def 0.98)
cfg.alphaAttitude = 0.98f;

// Barometer quality (lower = more trusted, def 1.0)
cfg.RBaroAltitude = 0.5f;  // Trust more
cfg.RBaroAltitude = 2.0f;  // Trust less

// ToF quality (lower = more trusted, def 0.5)
cfg.RTofDistance = 0.2f;   // Trust more

// Optical flow quality (lower = more trusted, def 0.1)
cfg.ROpticalFlowVelocity = 0.05f;  // Trust more

// When to use optical flow
cfg.minAltitudeForFlowM = 0.1f;   // Don't use below 10cm
cfg.maxAltitudeForFlowM = 5.0f;   // Don't use above 5m

// Apply tuning
sensorManager.setFusionConfig(cfg);
```

---

## Sensor Health Indicators

```cpp
auto status = sensorManager.getSystemStatus();

if (status.imuReady)    Serial.println("IMU OK");
if (status.baroReady)   Serial.println("Baro OK");
if (status.tofReady)    Serial.println("ToF OK");
if (status.flowReady)   Serial.println("Flow OK");

// Overall
Serial.printf("Confidence: %.0f%%\n", status.fusionConfidence);  // 0-100
```

---

## Pre-Flight Checklist

- ✓ All sensors initialized and reading
- ✓ Confidence > 60%
- ✓ Attitude stable when stationary
- ✓ Altitude stable (no drift > 10cm/10sec)
- ✓ ToF working for landing
- ✓ Optical flow healthy (if needed)

---

## Common Issues & Fixes

| Problem | Likely Cause | Fix |
|---------|--------------|-----|
| Altitude drifts | Baro poor | Increase `RBaroAltitude` |
| Erratic XY motion | Flow noisy | Increase `ROpticalFlowVelocity` |
| Attitude wobbles | Accel noisy | Increase `alphaAttitude` |
| Loses alt fast | Q too high | Lower `QVerticalNoise` |
| Confidence low | Multiple sensors bad | Check sensor connections |

---

## Thread Safety & Timing

- ✓ Safe to call `update()` from any timing loop
- ✓ Safe to call `getOutput()` anytime
- ✓ Recommended: 100-200 Hz update rate
- ✓ Minimum: 50 Hz, Maximum: 500 Hz

---

## Integration with Flight Controller

```cpp
struct DroneState {
  float roll, pitch, yaw;
  float x, y, z;
  float vx, vy, vz;
};

DroneState getLatestState() {
  auto fused = sensorManager.getFusionOutput();
  auto s = fused.state;
  
  return {
    s.rollDeg, s.pitchDeg, s.yawDeg,
    s.positionXM, s.positionYM, s.altitudeM,
    s.velocityXMps, s.velocityYMps, s.verticalVelocityMps
  };
}
```

---

## Calibration Shortcut

```cpp
void quickCal() {
  // Assumes drone is LEVEL and STATIONARY
  
  auto bias_gyro_x = /* average 100 gyro samples */;
  auto bias_gyro_y = /* average 100 gyro samples */;
  auto bias_gyro_z = /* average 100 gyro samples */;
  
  auto bias_accel_x = /* average with gravity removed */;
  auto bias_accel_y = /* average with gravity removed */;
  auto bias_accel_z = /* average with gravity removed */;
  
  // Will be implemented in SensorManager::calibrateSensors()
}
```

---

## Debug Monitoring

```cpp
void printDebugInfo() {
  auto fused = sensorManager.getFusionOutput();
  auto state = fused.state;
  auto status = sensorManager.getSystemStatus();
  
  Serial.printf("=== ATTITUDE ===\n");
  Serial.printf("Roll: %.2f° Pitch: %.2f° Yaw: %.2f°\n", 
    state.rollDeg, state.pitchDeg, state.yawDeg);
  
  Serial.printf("=== VERTICAL ===\n");
  Serial.printf("Alt: %.3f m  Vz: %.3f m/s\n", 
    state.altitudeM, state.verticalVelocityMps);
  
  Serial.printf("=== HORIZONTAL ===\n");
  Serial.printf("X: %.3f m Y: %.3f m\n", 
    state.positionXM, state.positionYM);
  Serial.printf("Vx: %.3f m/s Vy: %.3f m/s\n", 
    state.velocityXMps, state.velocityYMps);
  
  Serial.printf("=== HEALTH ===\n");
  Serial.printf("IMU: %d Baro: %d ToF: %d Flow: %d\n",
    status.imuReady, status.baroReady, status.tofReady, status.flowReady);
  Serial.printf("Confidence: %.0f%%\n\n", status.fusionConfidence);
}
```

---

## Architecture Layers

```
┌─────────────────────────────────────┐
│ Application (Flight Control)        │  ← Your code
├─────────────────────────────────────┤
│ SensorIntegration::SensorManager    │  ← Easy wrapper
├─────────────────────────────────────┤
│ SensorFusion::SensorFusionEngine    │  ← Core fusion
├─────────────────────────────────────┤
│ Sensor Drivers (IMU, Baro, ToF...)  │  ← Hardware
├─────────────────────────────────────┤
│ Communication (I2C, SPI)            │  ← Protocols
├─────────────────────────────────────┤
│ ESP32 Hardware                      │  ← Microcontroller
└─────────────────────────────────────┘
```

---

## Files & What They Do

| File | Purpose |
|------|---------|
| `sensor_fusion.h/cpp` | Core Kalman filters + attitude |
| `sensor_integration.h/cpp` | Wrapper around fusion + drivers |
| `acclgyr.h/cpp` | IMU (MPU6050) communication |
| `bmp585_test.h/cpp` | Barometer communication |
| `tof_distance.h/cpp` | ToF distance sensor communication |
| `optical_flow_test.h/cpp` | Optical flow camera communication |
| `SENSOR_FUSION_GUIDE.md` | Detailed documentation |
| `SENSOR_FUSION_QUICK_REF.md` | This file! |

---

## Next: Flight Control

Once you have stable fusion output, implement:

1. **Attitude Control** → stabilize roll/pitch/yaw
2. **Altitude Hold** → control vz via throttle
3. **Position Hold** → Stabilize x/y via roll/pitch commands
4. **Trajectory Tracking** → Follow desired paths

See `pid_controller.h` for existing PID implementations!

