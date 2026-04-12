#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Arduino.h>
#include "filter.h"

namespace SensorFusion
{
  // ==================== STATE STRUCTURES ====================

  struct State
  {
    // Attitude (degrees, degrees/sec)
    float rollDeg;
    float pitchDeg;
    float yawDeg;

    // Vertical position and velocity
    float altitudeM;           // meters, where 0 is ground
    float verticalVelocityMps; // m/s, positive = up

    // Horizontal position and velocity
    float positionXM;          // meters
    float positionYM;          // meters
    float velocityXMps;        // m/s
    float velocityYMps;        // m/s

    // Biases for calibration
    float accelBiasX;
    float accelBiasY;
    float accelBiasZ;
    float gyroBiasX;
    float gyroBiasY;
    float gyroBiasZ;

    // Covariance matrices (simplified diagonal elements for vertical and horizontal)
    float covVertical[2][2];   // [z, vz] covariance
    float covHorizontal[4][4]; // [x, y, vx, vy] covariance
  };

  // ==================== SENSOR INPUT STRUCTURES ====================

  struct ImuSensorReading
  {
    float accelXGravities;
    float accelYGravities;
    float accelZGravities;
    float gyroXDegPerSec;
    float gyroYDegPerSec;
    float gyroZDegPerSec;
    unsigned long timestampMs;
    bool valid;
  };

  struct MagnetometerReading
  {
    float magXMicroTesla; // raw magnetometer X
    float magYMicroTesla; // raw magnetometer Y
    float magZMicroTesla;
    unsigned long timestampMs;
    bool valid;
  };

  struct BarometerReading
  {
    float altitudeM;
    float temperatureC;
    unsigned long timestampMs;
    bool valid;
  };

  struct ToFReading
  {
    float distanceM;
    unsigned long timestampMs;
    bool valid;
  };

  struct OpticalFlowReading
  {
    float velocityXMps;     // converted to m/s using altitude
    float velocityYMps;
    float qualityPercent;   // 0-100
    unsigned long timestampMs;
    bool valid;
  };

  // ==================== OUTPUT STRUCTURES ====================

  struct FusionOutput
  {
    State state;                // Full state estimate
    float confidence;           // 0-100, overall confidence in estimate
    bool baroHealthy;
    bool tofHealthy;
    bool flowHealthy;
    bool imuHealthy;
    unsigned long lastUpdateMs;
  };

  // ==================== TUNING PARAMETERS ====================

  struct FusionConfig
  {
    // Complementary filter weight for attitude
    float alphaAttitude = 0.98f;

    // Measurement noise (R matrix diagonal)
    float RBaroAltitude = 1.0f;        // variance for barometer altitude
    float RTofDistance = 0.5f;         // variance for ToF distance
    float ROpticalFlowVelocity = 0.1f; // variance for optical flow velocity

    // Process noise (Q matrix)
    float QVerticalNoise = 0.01f;  // process noise for vertical states
    float QHorizontalNoise = 0.02f; // process noise for horizontal states

    // Minimum altitude for optical flow to be trusted
    float minAltitudeForFlowM = 0.1f;
    float maxAltitudeForFlowM = 5.0f;

    // Low-pass filter alphas for preprocessing
    float lpfAccelAlpha = 0.1f;
    float lpfGyroAlpha = 0.05f;

    // Gravity constant
    float gravityMps2 = 9.81f;

    // Min/max velocity estimates to prevent divergence
    float maxVerticalVelocityMps = 3.0f;
    float maxHorizontalVelocityMps = 5.0f;
  };

  // ==================== SENSOR FUSION CLASS ====================

  class SensorFusionEngine
  {
  public:
    SensorFusionEngine();

    // Initialize fusion engine
    void initialize();

    // Configuration
    void setConfig(const FusionConfig& config);
    FusionConfig getConfig() const;

    // Main update
    void update(
        const ImuSensorReading& imuReading,
        const MagnetometerReading& magReading,
        const BarometerReading& baroReading,
        const ToFReading& tofReading,
        const OpticalFlowReading& flowReading,
        float dtSeconds
    );

    // Get current state
    FusionOutput getOutput() const;
    State getState() const;

    // Reset
    void reset();

    // Calibration
    void calibrateGyro(float gyroBiasX, float gyroBiasY, float gyroBiasZ);
    void calibrateAccel(float accelBiasX, float accelBiasY, float accelBiasZ);

  private:
    // ==================== INTERNAL STATE ====================
    State state_;
    FusionConfig config_;
    FusionOutput output_;

    // Low-pass filters for preprocessing
    Filter::LowPassFilter lpfAccelX_;
    Filter::LowPassFilter lpfAccelY_;
    Filter::LowPassFilter lpfAccelZ_;
    Filter::LowPassFilter lpfGyroX_;
    Filter::LowPassFilter lpfGyroY_;
    Filter::LowPassFilter lpfGyroZ_;

    // Sensor health tracking
    bool imuHealthy_;
    bool magHealthy_;
    bool baroHealthy_;
    bool tofHealthy_;
    bool flowHealthy_;

    unsigned long lastUpdateMs_;
    bool isInitialized_;

    // ==================== PRIVATE METHODS ====================

    // 1. ATTITUDE ESTIMATION
    void updateAttitude(
        const ImuSensorReading& imuReading,
        const MagnetometerReading& magReading,
        float dtSeconds
    );

    // 2. PREPROCESSING
    void preprocessImu(
        const ImuSensorReading& imuReading,
        float& accelX,
        float& accelY,
        float& accelZ,
        float& gyroX,
        float& gyroY,
        float& gyroZ
    );

    // 3. COORDINATE TRANSFORMATION (body to world)
    void rotateBodyToWorld(
        float accelX, float accelY, float accelZ,
        float roll, float pitch, float yaw,
        float& accelXWorld, float& accelYWorld, float& accelZWorld
    );

    float degToRad(float degrees) const;
    float radToDeg(float radians) const;

    // 4. VERTICAL KALMAN FILTER
    void updateVerticalKalman(
        float accelZWorld,
        const BarometerReading& baroReading,
        const ToFReading& tofReading,
        float dtSeconds
    );

    // 5. HORIZONTAL KALMAN FILTER
    void updateHorizontalKalman(
        float accelXWorld,
        float accelYWorld,
        const OpticalFlowReading& flowReading,
        float dtSeconds
    );

    // 6. OPTICAL FLOW PROCESSING
    void processOpticalFlow(
        const OpticalFlowReading& flowReading,
        float& velocityXOut,
        float& velocityYOut
    );

    // Kalman gain computation (simplified)
    float computeKalmanGain(float P, float Q, float R) const;

    // Sensor health checks
    void updateSensorHealth(
        const ImuSensorReading& imuReading,
        const MagnetometerReading& magReading,
        const BarometerReading& baroReading,
        const ToFReading& tofReading,
        const OpticalFlowReading& flowReading
    );

    // Utility for computing rotation matrix elements
    float computeRotationElement(
        int row, int col,
        float roll, float pitch, float yaw
    ) const;
  };
}

#endif
