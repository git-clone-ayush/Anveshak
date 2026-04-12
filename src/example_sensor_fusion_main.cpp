/**
 * EXAMPLE: Complete Sensor Fusion Flight System
 * 
 * This example demonstrates how to:
 * 1. Initialize all sensors
 * 2. Run the sensor fusion engine
 * 3. Monitor system health
 * 4. Use the fused state for control
 * 
 * Compile with platformio.ini configured for ESP32
 */

#include <Arduino.h>
#include "sensor_integration.h"

// Global sensor manager
SensorIntegration::SensorManager sensorManager;

// ==================== TIMING CONTROL ====================

namespace Timing
{
  const unsigned long FUSION_UPDATE_INTERVAL_MS = 5;     // 200 Hz
  const unsigned long DEBUG_PRINT_INTERVAL_MS = 500;     // 2 Hz
  const unsigned long STATUS_CHECK_INTERVAL_MS = 1000;   // 1 Hz

  unsigned long lastFusionMs = 0;
  unsigned long lastDebugMs = 0;
  unsigned long lastStatusMs = 0;
}

// ==================== DEBUG UTILITIES ====================

void printFusedState(const SensorFusion::FusionOutput& output)
{
  const auto& state = output.state;

  Serial.println("\n╔════════════════ FUSED STATE ════════════════╗");

  // ATTITUDE
  Serial.print("║ ATTITUDE");
  Serial.printf(" │ Roll: %6.2f° │ Pitch: %6.2f° │ Yaw: %6.2f° ║\n",
                state.rollDeg, state.pitchDeg, state.yawDeg);

  // VERTICAL
  Serial.print("║ VERTICAL");
  Serial.printf(" │ Alt: %6.3f m │ Vz: %6.3f m/s │ ║\n",
                state.altitudeM, state.verticalVelocityMps);

  // HORIZONTAL POSITION
  Serial.print("║ POSITION");
  Serial.printf(" │ X: %6.3f m │ Y: %6.3f m    │ ║\n",
                state.positionXM, state.positionYM);

  // HORIZONTAL VELOCITY
  Serial.print("║ VELOCITY");
  Serial.printf(" │ Vx: %6.3f m/s │ Vy: %6.3f m/s │ ║\n",
                state.velocityXMps, state.velocityYMps);

  // CONFIDENCE
  Serial.printf("║ CONFIDENCE: %.0f %% │ ", output.confidence);
  if (output.baroHealthy)
    Serial.print("B");
  if (output.tofHealthy)
    Serial.print("T");
  if (output.flowHealthy)
    Serial.print("F");
  if (output.imuHealthy)
    Serial.print("I");
  Serial.println(" ║");

  Serial.println("╚═════════════════════════════════════════════════╝");
}

void printSystemStatus(const SensorIntegration::SystemStatus& status)
{
  Serial.println("\n┌─── SYSTEM STATUS ───┐");
  Serial.printf("│ IMU:   %s         │\n", status.imuReady ? "✓ OK" : "✗ NO");
  Serial.printf("│ Baro:  %s        │\n", status.baroReady ? "✓ OK" : "✗ NO");
  Serial.printf("│ ToF:   %s        │\n", status.tofReady ? "✓ OK" : "✗ NO");
  Serial.printf("│ Flow:  %s       │\n", status.flowReady ? "✓ OK" : "✗ NO");
  Serial.printf("│ Confidence: %.0f %%│\n", status.fusionConfidence);
  Serial.printf("│ Last update: %lu ms    │\n", status.lastFusionUpdateMs);
  Serial.println("└──────────────────────┘");
}

void printCalibrationTip()
{
  Serial.println("\n" \
                 "╔══════════════════════════════════════════╗\n" \
                 "║   CALIBRATION REQUIRED (First Time)      ║\n" \
                 "║                                          ║\n" \
                 "║   1. Place drone LEVEL on ground         ║\n" \
                 "║   2. Keep STATIONARY for 10 seconds    ║\n" \
                 "║   3. System will auto-calibrate gyro     ║\n" \
                 "║                                          ║\n" \
                 "║   (Implement calibrateSensors() next)   ║\n" \
                 "╚══════════════════════════════════════════╝\n");
}

// ==================== SETUP ====================

void setup()
{
  Serial.begin(115200);
  delay(1000); // Allow serial monitor to connect

  Serial.println("\n" \
                 "╔════════════════════════════════════════╗\n" \
                 "║  SENSOR FUSION FLIGHT SYSTEM v1.0     ║\n" \
                 "║  Drone Controller with IMU/Baro/ToF/OF║\n" \
                 "╚════════════════════════════════════════╝\n");

  Serial.println("→ Initializing sensor systems...");

  // Initialize sensor manager (does everything)
  sensorManager.begin();

  Serial.println("✓ Sensors initialized");
  Serial.println("✓ Sensor fusion engine ready");
  Serial.println("\nWaiting for sensors to stabilize...");
  Serial.println("(This may take 5-10 seconds)\n");

  printCalibrationTip();
}

// ==================== MAIN LOOP ====================

void loop()
{
  unsigned long nowMs = millis();

  // ========== FUSION UPDATE (High Frequency) ==========
  if (nowMs - Timing::lastFusionMs >= Timing::FUSION_UPDATE_INTERVAL_MS)
  {
    Timing::lastFusionMs = nowMs;

    // This does all the heavy lifting:
    // - Reads all sensors
    // - Fuses them together
    // - Produces state estimate
    sensorManager.update();
  }

  // ========== DEBUG PRINTING (Lower Frequency) ==========
  if (nowMs - Timing::lastDebugMs >= Timing::DEBUG_PRINT_INTERVAL_MS)
  {
    Timing::lastDebugMs = nowMs;

    auto output = sensorManager.getFusionOutput();
    printFusedState(output);
  }

  // ========== STATUS CHECK (Lower Frequency) ==========
  if (nowMs - Timing::lastStatusMs >= Timing::STATUS_CHECK_INTERVAL_MS)
  {
    Timing::lastStatusMs = nowMs;

    auto status = sensorManager.getSystemStatus();
    printSystemStatus(status);

    // Check for issues
    float confidence = status.fusionConfidence;
    if (confidence < 50.0f)
    {
      Serial.println("⚠ WARNING: Low confidence in fusion estimate!");
      Serial.println("  Check sensor connections and health");
    }
  }

  // Allow other processing if needed
  delay(1);
}

// ==================== EXAMPLE: USING FUSED STATE FOR CONTROL ====================

/**
 * Example function showing how to integrate fusion output with flight control
 */
void exampleFlightControl()
{
  // Get the latest fused state
  auto output = sensorManager.getFusionOutput();
  auto state = output.state;

  // Extract useful variables
  float roll = state.rollDeg;
  float pitch = state.pitchDeg;
  float yaw = state.yawDeg;
  float altitude = state.altitudeM;
  float vz = state.verticalVelocityMps;
  float vx = state.velocityXMps;
  float vy = state.velocityYMps;

  // Check if estimate is healthy
  if (output.confidence < 70.0f)
  {
    Serial.println("Not ready to fly - confidence too low");
    return;
  }

  // Example attitude stabilization
  float targetRoll = 0.0f;
  float targetPitch = 0.0f;
  float targetYaw = 0.0f;

  float rollError = targetRoll - roll;
  float pitchError = targetPitch - pitch;
  float yawError = targetYaw - yaw;

  // These errors would be fed to your PID controllers
  // (See pid_controller.h for implementation)

  // Example altitude hold
  float targetAltitude = 1.0f; // 1 meter
  float altitudeError = targetAltitude - altitude;

  // Velocity-based control
  float currentVx = vx;
  float targetVx = 0.0f; // Hover in place
  float vxError = targetVx - currentVx;

  // Serial.printf("Attitude errors: roll=%.2f pitch=%.2f yaw=%.2f\n",
  //               rollError, pitchError, yawError);
  // Serial.printf("Altitude error: %.3f m\n", altitudeError);
  // Serial.printf("Velocity error: X=%.3f m/s\n", vxError);
}

// ==================== EXAMPLE: ADVANCED CONFIGURATION ====================

/**
 * Example showing how to tune the fusion engine for different conditions
 */
void exampleAdvancedTuning()
{
  // Get current configuration
  SensorFusion::FusionConfig config;

  // === ATTITUDE TUNING ===
  // 0.98 = trust gyro heavily (good for fast dynamics)
  // 0.90 = balance between gyro and accel
  // 0.80 = trust accel more (good for high vibration)
  config.alphaAttitude = 0.95f;

  // === MEASUREMENT NOISE (Lower = more trust) ===

  // Barometer: varies by sensor quality
  config.RBaroAltitude = 1.0f;    // Default
  // config.RBaroAltitude = 0.5f;  // If baro is very good
  // config.RBaroAltitude = 2.0f;  // If baro is noisy

  // ToF distance sensor (usually very good)
  config.RTofDistance = 0.5f;    // Default (lower than baro = more trusted)
  // config.RTofDistance = 0.1f;  // If you want precise landing

  // Optical flow (very sensitive to tuning)
  config.ROpticalFlowVelocity = 0.1f;  // Default
  // config.ROpticalFlowVelocity = 0.05f; // If flow is very good
  // config.ROpticalFlowVelocity = 0.5f;  // If flow is noisy

  // === PROCESS NOISE (Higher = more forgiving) ===
  config.QVerticalNoise = 0.01f;   // Default (conservative)
  // config.QVerticalNoise = 0.001f; // Very smooth (minimal drift allowed)
  // config.QVerticalNoise = 0.1f;   // Forgiving (allows more drift)

  config.QHorizontalNoise = 0.02f;  // Default

  // === OPTICAL FLOW CONSTRAINTS ===
  config.minAltitudeForFlowM = 0.1f;   // Don't use flow below 10cm
  config.maxAltitudeForFlowM = 5.0f;   // Don't use flow above 5m

  // === LOW-PASS FILTERING ===
  config.lpfAccelAlpha = 0.1f;  // Higher = less filtering
  config.lpfGyroAlpha = 0.05f;

  // === VELOCITY CLAMPING ===
  config.maxVerticalVelocityMps = 3.0f;
  config.maxHorizontalVelocityMps = 5.0f;

  // Apply configuration
  sensorManager.setFusionConfig(config);

  Serial.println("✓ Advanced tuning applied");
}

// ==================== UTILITIES ====================

/**
 * Check if system is ready for flight
 */
bool isReadyForFlight()
{
  auto status = sensorManager.getSystemStatus();

  // All sensors must be healthy
  if (!status.imuReady || !status.baroReady || !status.tofReady)
  {
    return false;
  }

  // Confidence must be adequate
  if (status.fusionConfidence < 70.0f)
  {
    return false;
  }

  // Estimate must be stable
  auto output = sensorManager.getFusionOutput();
  auto state = output.state;

  // Attitude should be close to level
  if (fabsf(state.rollDeg) > 15.0f || fabsf(state.pitchDeg) > 15.0f)
  {
    return false;
  }

  // Should be near ground
  if (state.altitudeM > 0.5f)
  {
    return false; // Likely in air or sensor error
  }

  return true;
}

/**
 * Get only the most critical data (for minimal bandwidth/compute)
 */
struct MinimalState
{
  float roll, pitch, yaw;
  float altitude, verticalVelocity;
  float positionX, positionY;
  float velocityX, velocityY;
  float confidence;
};

MinimalState getMinimalState()
{
  auto output = sensorManager.getFusionOutput();
  auto s = output.state;

  return {
    s.rollDeg, s.pitchDeg, s.yawDeg,
    s.altitudeM, s.verticalVelocityMps,
    s.positionXM, s.positionYM,
    s.velocityXMps, s.velocityYMps,
    output.confidence
  };
}
