#include "sensor_fusion.h"
#include <math.h>
#include <string.h>

namespace
{
  constexpr float PI = 3.14159265359f;
  constexpr float DEG_TO_RAD = PI / 180.0f;
  constexpr float RAD_TO_DEG = 180.0f / PI;

  // Gravity vector constant
  constexpr float GRAVITY_MPS2 = 9.81f;

  // Sensor health thresholds
  constexpr float ACCEL_SATURATION_LIMIT = 4.0f; // g
  constexpr float GYRO_SATURATION_LIMIT = 250.0f; // deg/s
}

namespace SensorFusion
{
  // ==================== CONSTRUCTOR & INITIALIZATION ====================

  SensorFusionEngine::SensorFusionEngine()
      : lpfAccelX_(0.1f), lpfAccelY_(0.1f), lpfAccelZ_(0.1f),
        lpfGyroX_(0.05f), lpfGyroY_(0.05f), lpfGyroZ_(0.05f),
        imuHealthy_(false), magHealthy_(false), baroHealthy_(false),
        tofHealthy_(false), flowHealthy_(false),
        lastUpdateMs_(0), isInitialized_(false)
  {
    memset(&state_, 0, sizeof(state_));
    memset(&output_, 0, sizeof(output_));
    memset(&config_, 0, sizeof(config_));
  }

  void SensorFusionEngine::initialize()
  {
    // Initialize state
    state_.rollDeg = 0.0f;
    state_.pitchDeg = 0.0f;
    state_.yawDeg = 0.0f;
    state_.altitudeM = 0.0f;
    state_.verticalVelocityMps = 0.0f;
    state_.positionXM = 0.0f;
    state_.positionYM = 0.0f;
    state_.velocityXMps = 0.0f;
    state_.velocityYMps = 0.0f;

    // Initialize biases
    state_.accelBiasX = 0.0f;
    state_.accelBiasY = 0.0f;
    state_.accelBiasZ = 0.0f;
    state_.gyroBiasX = 0.0f;
    state_.gyroBiasY = 0.0f;
    state_.gyroBiasZ = 0.0f;

    // Initialize covariance matrices (diagonal elements)
    state_.covVertical[0][0] = 1.0f;
    state_.covVertical[0][1] = 0.0f;
    state_.covVertical[1][0] = 0.0f;
    state_.covVertical[1][1] = 1.0f;

    state_.covHorizontal[0][0] = 1.0f;
    state_.covHorizontal[1][1] = 1.0f;
    state_.covHorizontal[2][2] = 1.0f;
    state_.covHorizontal[3][3] = 1.0f;

    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        if (i != j)
        {
          state_.covHorizontal[i][j] = 0.0f;
        }
      }
    }

    // Initialize filters
    lpfAccelX_.reset(0.0f);
    lpfAccelY_.reset(0.0f);
    lpfAccelZ_.reset(0.0f);
    lpfGyroX_.reset(0.0f);
    lpfGyroY_.reset(0.0f);
    lpfGyroZ_.reset(0.0f);

    isInitialized_ = true;
  }

  void SensorFusionEngine::setConfig(const FusionConfig& config)
  {
    config_ = config;
    lpfAccelX_.setAlpha(config_.lpfAccelAlpha);
    lpfAccelY_.setAlpha(config_.lpfAccelAlpha);
    lpfAccelZ_.setAlpha(config_.lpfAccelAlpha);
    lpfGyroX_.setAlpha(config_.lpfGyroAlpha);
    lpfGyroY_.setAlpha(config_.lpfGyroAlpha);
    lpfGyroZ_.setAlpha(config_.lpfGyroAlpha);
  }

  FusionConfig SensorFusionEngine::getConfig() const
  {
    return config_;
  }

  void SensorFusionEngine::reset()
  {
    initialize();
  }

  void SensorFusionEngine::calibrateGyro(float gyroBiasX, float gyroBiasY, float gyroBiasZ)
  {
    state_.gyroBiasX = gyroBiasX;
    state_.gyroBiasY = gyroBiasY;
    state_.gyroBiasZ = gyroBiasZ;
  }

  void SensorFusionEngine::calibrateAccel(float accelBiasX, float accelBiasY, float accelBiasZ)
  {
    state_.accelBiasX = accelBiasX;
    state_.accelBiasY = accelBiasY;
    state_.accelBiasZ = accelBiasZ;
  }

  // ==================== MAIN UPDATE FUNCTION ====================

  void SensorFusionEngine::update(
      const ImuSensorReading& imuReading,
      const MagnetometerReading& magReading,
      const BarometerReading& baroReading,
      const ToFReading& tofReading,
      const OpticalFlowReading& flowReading,
      float dtSeconds)
  {
    if (!isInitialized_)
    {
      initialize();
    }

    // Clamp dt to prevent numerical issues
    if (dtSeconds < 0.0001f || dtSeconds > 0.1f)
    {
      return;
    }

    // 1. UPDATE SENSOR HEALTH
    updateSensorHealth(imuReading, magReading, baroReading, tofReading, flowReading);

    // 2. ATTITUDE ESTIMATION (roll, pitch, yaw)
    updateAttitude(imuReading, magReading, dtSeconds);

    // 3. PREPROCESS IMU DATA
    float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
    preprocessImu(imuReading, accelX, accelY, accelZ, gyroX, gyroY, gyroZ);

    // 4. ROTATE ACCELERATION FROM BODY TO WORLD FRAME
    float accelXWorld, accelYWorld, accelZWorld;
    rotateBodyToWorld(accelX, accelY, accelZ,
                      state_.rollDeg, state_.pitchDeg, state_.yawDeg,
                      accelXWorld, accelYWorld, accelZWorld);

    // 5. REMOVE GRAVITY FROM Z ACCELERATION
    accelZWorld = accelZWorld - config_.gravityMps2;

    // 6. VERTICAL KALMAN FILTER (z, vz)
    updateVerticalKalman(accelZWorld, baroReading, tofReading, dtSeconds);

    // 7. PROCESS OPTICAL FLOW
    float velXFlow, velYFlow;
    processOpticalFlow(flowReading, velXFlow, velYFlow);

    // 8. HORIZONTAL KALMAN FILTER (x, y, vx, vy)
    updateHorizontalKalman(accelXWorld, accelYWorld, flowReading, dtSeconds);

    // 9. UPDATE OUTPUT STRUCTURE
    output_.state = state_;
    output_.lastUpdateMs = millis();

    // Compute overall confidence
    float healthScore = 0.0f;
    if (imuHealthy_)
      healthScore += 25.0f;
    if (baroHealthy_)
      healthScore += 25.0f;
    if (tofHealthy_)
      healthScore += 25.0f;
    if (flowHealthy_)
      healthScore += 25.0f;
    output_.confidence = healthScore;

    output_.baroHealthy = baroHealthy_;
    output_.tofHealthy = tofHealthy_;
    output_.flowHealthy = flowHealthy_;
    output_.imuHealthy = imuHealthy_;

    lastUpdateMs_ = millis();
  }

  FusionOutput SensorFusionEngine::getOutput() const
  {
    return output_;
  }

  State SensorFusionEngine::getState() const
  {
    return state_;
  }

  // ==================== ATTITUDE ESTIMATION ====================

  void SensorFusionEngine::updateAttitude(
      const ImuSensorReading& imuReading,
      const MagnetometerReading& magReading,
      float dtSeconds)
  {
    if (!imuReading.valid)
    {
      return;
    }

    // Convert gyro readings from degrees/sec to radians/sec
    float gyroXRad = (imuReading.gyroXDegPerSec - state_.gyroBiasX) * DEG_TO_RAD;
    float gyroYRad = (imuReading.gyroYDegPerSec - state_.gyroBiasY) * DEG_TO_RAD;
    float gyroZRad = (imuReading.gyroZDegPerSec - state_.gyroBiasZ) * DEG_TO_RAD;

    // GYRO INTEGRATION
    float rollGyro = state_.rollDeg + (imuReading.gyroXDegPerSec - state_.gyroBiasX) * dtSeconds;
    float pitchGyro = state_.pitchDeg + (imuReading.gyroYDegPerSec - state_.gyroBiasY) * dtSeconds;

    // ACCELEROMETER ANGLES
    float rollAcc = atan2f(imuReading.accelYGravities, imuReading.accelZGravities) * RAD_TO_DEG;
    float pitchAcc = atan2f(-imuReading.accelXGravities,
                            sqrtf((imuReading.accelYGravities * imuReading.accelYGravities) +
                                  (imuReading.accelZGravities * imuReading.accelZGravities))) *
                     RAD_TO_DEG;

    // COMPLEMENTARY FILTER
    state_.rollDeg = config_.alphaAttitude * rollGyro + (1.0f - config_.alphaAttitude) * rollAcc;
    state_.pitchDeg = config_.alphaAttitude * pitchGyro + (1.0f - config_.alphaAttitude) * pitchAcc;

    // YAW FROM MAGNETOMETER
    if (magReading.valid)
    {
      float yawMag = atan2f(magReading.magYMicroTesla, magReading.magXMicroTesla) * RAD_TO_DEG;
      float yawGyro = state_.yawDeg + (imuReading.gyroZDegPerSec - state_.gyroBiasZ) * dtSeconds;
      state_.yawDeg = config_.alphaAttitude * yawGyro + (1.0f - config_.alphaAttitude) * yawMag;
    }
    else
    {
      // No magnetometer, just use gyro integration
      state_.yawDeg += (imuReading.gyroZDegPerSec - state_.gyroBiasZ) * dtSeconds;
    }

    // Wrap angles to [-180, 180]
    while (state_.yawDeg > 180.0f)
      state_.yawDeg -= 360.0f;
    while (state_.yawDeg < -180.0f)
      state_.yawDeg += 360.0f;

    imuHealthy_ = true;
  }

  // ==================== IMU PREPROCESSING ====================

  void SensorFusionEngine::preprocessImu(
      const ImuSensorReading& imuReading,
      float& accelX,
      float& accelY,
      float& accelZ,
      float& gyroX,
      float& gyroY,
      float& gyroZ)
  {
    // Remove biases
    accelX = imuReading.accelXGravities - state_.accelBiasX;
    accelY = imuReading.accelYGravities - state_.accelBiasY;
    accelZ = imuReading.accelZGravities - state_.accelBiasZ;
    gyroX = imuReading.gyroXDegPerSec - state_.gyroBiasX;
    gyroY = imuReading.gyroYDegPerSec - state_.gyroBiasY;
    gyroZ = imuReading.gyroZDegPerSec - state_.gyroBiasZ;

    // Apply low-pass filters
    accelX = lpfAccelX_.update(accelX);
    accelY = lpfAccelY_.update(accelY);
    accelZ = lpfAccelZ_.update(accelZ);
    gyroX = lpfGyroX_.update(gyroX);
    gyroY = lpfGyroY_.update(gyroY);
    gyroZ = lpfGyroZ_.update(gyroZ);
  }

  // ==================== COORDINATE TRANSFORMATION ====================

  float SensorFusionEngine::degToRad(float degrees) const
  {
    return degrees * DEG_TO_RAD;
  }

  float SensorFusionEngine::radToDeg(float radians) const
  {
    return radians * RAD_TO_DEG;
  }

  float SensorFusionEngine::computeRotationElement(
      int row, int col,
      float roll, float pitch, float yaw) const
  {
    float rollRad = degToRad(roll);
    float pitchRad = degToRad(pitch);
    float yawRad = degToRad(yaw);

    float cr = cosf(rollRad);
    float sr = sinf(rollRad);
    float cp = cosf(pitchRad);
    float sp = sinf(pitchRad);
    float cy = cosf(yawRad);
    float sy = sinf(yawRad);

    // ZYX Euler angle rotation matrix
    if (row == 0)
    {
      if (col == 0)
        return cy * cp;
      if (col == 1)
        return cy * sp * sr - sy * cr;
      if (col == 2)
        return cy * sp * cr + sy * sr;
    }
    if (row == 1)
    {
      if (col == 0)
        return sy * cp;
      if (col == 1)
        return sy * sp * sr + cy * cr;
      if (col == 2)
        return sy * sp * cr - cy * sr;
    }
    if (row == 2)
    {
      if (col == 0)
        return -sp;
      if (col == 1)
        return cp * sr;
      if (col == 2)
        return cp * cr;
    }

    return 0.0f;
  }

  void SensorFusionEngine::rotateBodyToWorld(
      float accelX, float accelY, float accelZ,
      float roll, float pitch, float yaw,
      float& accelXWorld, float& accelYWorld, float& accelZWorld)
  {
    // Apply rotation matrix (body frame to world frame)
    // Using ZYX Euler angle convention
    accelXWorld = computeRotationElement(0, 0, roll, pitch, yaw) * accelX +
                  computeRotationElement(0, 1, roll, pitch, yaw) * accelY +
                  computeRotationElement(0, 2, roll, pitch, yaw) * accelZ;

    accelYWorld = computeRotationElement(1, 0, roll, pitch, yaw) * accelX +
                  computeRotationElement(1, 1, roll, pitch, yaw) * accelY +
                  computeRotationElement(1, 2, roll, pitch, yaw) * accelZ;

    accelZWorld = computeRotationElement(2, 0, roll, pitch, yaw) * accelX +
                  computeRotationElement(2, 1, roll, pitch, yaw) * accelY +
                  computeRotationElement(2, 2, roll, pitch, yaw) * accelZ;
  }

  // ==================== VERTICAL KALMAN FILTER ====================

  float SensorFusionEngine::computeKalmanGain(float P, float Q, float R) const
  {
    float S = P + Q + R;
    if (S < 0.001f)
      return 0.0f;
    return (P + Q) / S;
  }

  void SensorFusionEngine::updateVerticalKalman(
      float accelZWorld,
      const BarometerReading& baroReading,
      const ToFReading& tofReading,
      float dtSeconds)
  {
    // PREDICTION STEP
    state_.altitudeM = state_.altitudeM + state_.verticalVelocityMps * dtSeconds +
                       0.5f * accelZWorld * dtSeconds * dtSeconds;
    state_.verticalVelocityMps = state_.verticalVelocityMps + accelZWorld * dtSeconds;

    // Update covariance prediction (simplified)
    state_.covVertical[0][0] += config_.QVerticalNoise;
    state_.covVertical[1][1] += config_.QVerticalNoise;

    // BAROMETER UPDATE
    if (baroReading.valid)
    {
      float K = computeKalmanGain(state_.covVertical[0][0], config_.QVerticalNoise,
                                  config_.RBaroAltitude);
      float zMeas = baroReading.altitudeM;
      float innovation = zMeas - state_.altitudeM;
      state_.altitudeM += K * innovation;

      // Small velocity correction from altitude measurement
      state_.verticalVelocityMps += K * 0.1f * innovation;

      // Update covariance
      state_.covVertical[0][0] *= (1.0f - K);
      baroHealthy_ = true;
    }

    // TOF UPDATE
    if (tofReading.valid)
    {
      float K = computeKalmanGain(state_.covVertical[0][0], config_.QVerticalNoise,
                                  config_.RTofDistance);
      float zMeas = tofReading.distanceM;
      float innovation = zMeas - state_.altitudeM;
      state_.altitudeM += K * innovation;

      // Small velocity correction from distance measurement
      state_.verticalVelocityMps += K * 0.15f * innovation;

      // Update covariance
      state_.covVertical[0][0] *= (1.0f - K);
      tofHealthy_ = true;
    }

    // Clamp vertical velocity
    if (state_.verticalVelocityMps > config_.maxVerticalVelocityMps)
    {
      state_.verticalVelocityMps = config_.maxVerticalVelocityMps;
    }
    else if (state_.verticalVelocityMps < -config_.maxVerticalVelocityMps)
    {
      state_.verticalVelocityMps = -config_.maxVerticalVelocityMps;
    }

    // Prevent negative altitude
    if (state_.altitudeM < 0.0f)
    {
      state_.altitudeM = 0.0f;
      if (state_.verticalVelocityMps < 0.0f)
      {
        state_.verticalVelocityMps = 0.0f;
      }
    }
  }

  // ==================== OPTICAL FLOW PROCESSING ====================

  void SensorFusionEngine::processOpticalFlow(
      const OpticalFlowReading& flowReading,
      float& velocityXOut,
      float& velocityYOut)
  {
    velocityXOut = 0.0f;
    velocityYOut = 0.0f;

    if (!flowReading.valid)
    {
      return;
    }

    // Check altitude is in valid range
    if (state_.altitudeM < config_.minAltitudeForFlowM ||
        state_.altitudeM > config_.maxAltitudeForFlowM)
    {
      return;
    }

    // Flow is already in m/s from the flow sensor preprocessor
    velocityXOut = flowReading.velocityXMps;
    velocityYOut = flowReading.velocityYMps;

    flowHealthy_ = true;
  }

  // ==================== HORIZONTAL KALMAN FILTER ====================

  void SensorFusionEngine::updateHorizontalKalman(
      float accelXWorld,
      float accelYWorld,
      const OpticalFlowReading& flowReading,
      float dtSeconds)
  {
    // PREDICTION STEP
    state_.positionXM = state_.positionXM + state_.velocityXMps * dtSeconds +
                        0.5f * accelXWorld * dtSeconds * dtSeconds;
    state_.positionYM = state_.positionYM + state_.velocityYMps * dtSeconds +
                        0.5f * accelYWorld * dtSeconds * dtSeconds;

    state_.velocityXMps = state_.velocityXMps + accelXWorld * dtSeconds;
    state_.velocityYMps = state_.velocityYMps + accelYWorld * dtSeconds;

    // Update covariance prediction (simplified diagonal)
    state_.covHorizontal[0][0] += config_.QHorizontalNoise;
    state_.covHorizontal[1][1] += config_.QHorizontalNoise;
    state_.covHorizontal[2][2] += config_.QHorizontalNoise;
    state_.covHorizontal[3][3] += config_.QHorizontalNoise;

    // OPTICAL FLOW UPDATE (velocity measurement)
    if (flowReading.valid && 
        state_.altitudeM >= config_.minAltitudeForFlowM &&
        state_.altitudeM <= config_.maxAltitudeForFlowM)
    {
      // X velocity update
      float Kx = computeKalmanGain(state_.covHorizontal[2][2], config_.QHorizontalNoise,
                                   config_.ROpticalFlowVelocity);
      float vxMeas = flowReading.velocityXMps;
      float innovationX = vxMeas - state_.velocityXMps;
      state_.velocityXMps += Kx * innovationX;

      // Also slightly correct position based on velocity
      state_.positionXM += Kx * 0.1f * innovationX * dtSeconds;

      // Y velocity update
      float Ky = computeKalmanGain(state_.covHorizontal[3][3], config_.QHorizontalNoise,
                                   config_.ROpticalFlowVelocity);
      float vyMeas = flowReading.velocityYMps;
      float innovationY = vyMeas - state_.velocityYMps;
      state_.velocityYMps += Ky * innovationY;

      // Also slightly correct position based on velocity
      state_.positionYM += Ky * 0.1f * innovationY * dtSeconds;

      // Update covariance
      state_.covHorizontal[2][2] *= (1.0f - Kx);
      state_.covHorizontal[3][3] *= (1.0f - Ky);
    }

    // Clamp horizontal velocities
    if (state_.velocityXMps > config_.maxHorizontalVelocityMps)
    {
      state_.velocityXMps = config_.maxHorizontalVelocityMps;
    }
    else if (state_.velocityXMps < -config_.maxHorizontalVelocityMps)
    {
      state_.velocityXMps = -config_.maxHorizontalVelocityMps;
    }

    if (state_.velocityYMps > config_.maxHorizontalVelocityMps)
    {
      state_.velocityYMps = config_.maxHorizontalVelocityMps;
    }
    else if (state_.velocityYMps < -config_.maxHorizontalVelocityMps)
    {
      state_.velocityYMps = -config_.maxHorizontalVelocityMps;
    }
  }

  // ==================== SENSOR HEALTH CHECKS ====================

  void SensorFusionEngine::updateSensorHealth(
      const ImuSensorReading& imuReading,
      const MagnetometerReading& magReading,
      const BarometerReading& baroReading,
      const ToFReading& tofReading,
      const OpticalFlowReading& flowReading)
  {
    imuHealthy_ = imuReading.valid &&
                  fabsf(imuReading.accelXGravities) < ACCEL_SATURATION_LIMIT &&
                  fabsf(imuReading.accelYGravities) < ACCEL_SATURATION_LIMIT &&
                  fabsf(imuReading.accelZGravities) < ACCEL_SATURATION_LIMIT &&
                  fabsf(imuReading.gyroXDegPerSec) < GYRO_SATURATION_LIMIT &&
                  fabsf(imuReading.gyroYDegPerSec) < GYRO_SATURATION_LIMIT &&
                  fabsf(imuReading.gyroZDegPerSec) < GYRO_SATURATION_LIMIT;

    magHealthy_ = magReading.valid;
    baroHealthy_ = baroReading.valid;
    tofHealthy_ = tofReading.valid;
    flowHealthy_ = flowReading.valid;
  }
}
