#include "sensor_integration.h"

namespace SensorIntegration
{
  void SensorManager::begin()
  {
    // Initialize all sensor drivers
    AcclGyr::begin();
    Bmp585Test::begin();
    TofDistance::begin();
    OpticalFlowTest::begin();

    // Initialize fusion engine
    fusionEngine_.initialize();

    // Configure with default parameters
    SensorFusion::FusionConfig defaultConfig;
    fusionEngine_.setConfig(defaultConfig);

    lastFusionMs_ = millis();

    status_.imuReady = false;
    status_.baroReady = false;
    status_.tofReady = false;
    status_.flowReady = false;
    status_.fusionReady = false;
    status_.lastFusionUpdateMs = 0;
    status_.fusionConfidence = 0.0f;
  }

  void SensorManager::update()
  {
    unsigned long nowMs = millis();
    unsigned long deltaMsUnsigned = nowMs - lastFusionMs_;
    lastFusionMs_ = nowMs;

    // Cap delta time to reasonable value (5-50ms typically)
    float dtSeconds = constrain(deltaMsUnsigned / 1000.0f, 0.001f, 0.1f);

    // 1. READ ALL SENSORS
    AcclGyr::loop();
    Bmp585Test::loop();
    TofDistance::loop();
    OpticalFlowTest::loop();

    // 2. CHECK SENSOR READINESS
    status_.imuReady = AcclGyr::isReady();
    status_.baroReady = Bmp585Test::isReady();
    status_.tofReady = TofDistance::isReady();
    status_.flowReady = OpticalFlowTest::isHealthy();

    // 3. COLLECT SENSOR READINGS
    SensorFusion::ImuSensorReading imuReading;
    imuReading.valid = false;
    if (status_.imuReady)
    {
      imuReading = convertImuReading(AcclGyr::latestSample());
    }

    SensorFusion::BarometerReading baroReading;
    baroReading.valid = false;
    if (status_.baroReady)
    {
      baroReading = convertBaroReading(Bmp585Test::latestSample());
    }

    SensorFusion::ToFReading tofReading;
    tofReading.valid = false;
    if (status_.tofReady)
    {
      tofReading = convertTofReading(TofDistance::latestSample());
    }

    SensorFusion::OpticalFlowReading flowReading;
    flowReading.valid = false;
    if (status_.flowReady)
    {
      flowReading = convertFlowReading(OpticalFlowTest::latestSample());
    }

    // 4. CREATE DUMMY MAGNETOMETER READING (if available)
    // For now, we'll pass an invalid reading - integrate magnetometer if available
    SensorFusion::MagnetometerReading magReading;
    magReading.valid = false;
    magReading.magXMicroTesla = 0.0f;
    magReading.magYMicroTesla = 0.0f;
    magReading.magZMicroTesla = 0.0f;
    magReading.timestampMs = nowMs;

    // 5. UPDATE FUSION ENGINE
    if (status_.imuReady)
    {
      fusionEngine_.update(imuReading, magReading, baroReading, tofReading, flowReading, dtSeconds);
      status_.fusionReady = true;
      status_.lastFusionUpdateMs = nowMs;

      auto output = fusionEngine_.getOutput();
      status_.fusionConfidence = output.confidence;
    }
  }

  SensorFusion::FusionOutput SensorManager::getFusionOutput() const
  {
    return fusionEngine_.getOutput();
  }

  SystemStatus SensorManager::getSystemStatus() const
  {
    return status_;
  }

  void SensorManager::setFusionConfig(const SensorFusion::FusionConfig& config)
  {
    fusionEngine_.setConfig(config);
  }

  void SensorManager::calibrateSensors()
  {
    // TODO: Implement calibration routine
    // This would involve:
    // 1. Reading gyro bias over multiple samples
    // 2. Reading accel bias (with gravity removed)
    // 3. Setting these biases in the fusion engine
  }

  // ==================== CONVERSION HELPERS ====================

  SensorFusion::ImuSensorReading SensorManager::convertImuReading(const AcclGyr::Sample& sample)
  {
    SensorFusion::ImuSensorReading reading;
    reading.accelXGravities = sample.imu.accelX;
    reading.accelYGravities = sample.imu.accelY;
    reading.accelZGravities = sample.imu.accelZ;
    reading.gyroXDegPerSec = sample.imu.gyroXDegPerSec;
    reading.gyroYDegPerSec = sample.imu.gyroYDegPerSec;
    reading.gyroZDegPerSec = sample.imu.gyroZDegPerSec;
    reading.timestampMs = sample.timestampMs;
    reading.valid = sample.valid;
    return reading;
  }

  SensorFusion::BarometerReading SensorManager::convertBaroReading(const Bmp585Test::Sample& sample)
  {
    SensorFusion::BarometerReading reading;
    reading.altitudeM = sample.altitudeM;
    reading.temperatureC = sample.temperatureC;
    reading.timestampMs = sample.timestampMs;
    reading.valid = sample.valid;
    return reading;
  }

  SensorFusion::ToFReading SensorManager::convertTofReading(const TofDistance::Sample& sample)
  {
    SensorFusion::ToFReading reading;
    reading.distanceM = sample.distanceCm / 100.0f; // Convert cm to meters
    reading.timestampMs = sample.timestampMs;
    reading.valid = sample.valid && !sample.timeout;
    return reading;
  }

  SensorFusion::OpticalFlowReading SensorManager::convertFlowReading(const OpticalFlowTest::Sample& sample)
  {
    SensorFusion::OpticalFlowReading reading;
    reading.velocityXMps = sample.flow.velocityX;
    reading.velocityYMps = sample.flow.velocityY;
    reading.qualityPercent = sample.flow.quality * 100.0f;
    reading.timestampMs = sample.timestampMs;
    reading.valid = sample.flow.valid && sample.healthy;
    return reading;
  }
}
