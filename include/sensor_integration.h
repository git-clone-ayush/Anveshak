#ifndef SENSOR_INTEGRATION_H
#define SENSOR_INTEGRATION_H

#include <Arduino.h>
#include "sensor_fusion.h"
#include "acclgyr.h"
#include "bmp585_test.h"
#include "tof_distance.h"
#include "optical_flow_test.h"

/**
 * SensorIntegration: High-level interface that collects data from all sensor
 * modules and feeds them into the sensor fusion engine.
 *
 * This module handles:
 * - Reading from all sensor driver modules
 * - Converting sensor readings to fusion-compatible formats
 * - Managing fusion engine state
 * - Providing fused output
 */

namespace SensorIntegration
{
  struct SystemStatus
  {
    bool imuReady;
    bool baroReady;
    bool tofReady;
    bool flowReady;
    bool fusionReady;
    
    unsigned long lastFusionUpdateMs;
    float fusionConfidence;
  };

  class SensorManager
  {
  public:
    /**
     * Initialize all sensor systems
     */
    void begin();

    /**
     * Main update loop - should be called frequently (e.g., every 5-10ms)
     * Reads all sensors and updates fusion engine
     */
    void update();

    /**
     * Get the fused state estimate
     */
    SensorFusion::FusionOutput getFusionOutput() const;

    /**
     * Get system status
     */
    SystemStatus getSystemStatus() const;

    /**
     * Set fusion engine configuration
     */
    void setFusionConfig(const SensorFusion::FusionConfig& config);

    /**
     * Calibrate sensors
     * Should be called on startup with drone level and stationary
     */
    void calibrateSensors();

  private:
    SensorFusion::SensorFusionEngine fusionEngine_;
    SystemStatus status_;
    unsigned long lastFusionMs_;
    
    // Conversion helpers from sensor drivers to fusion structures
    SensorFusion::ImuSensorReading convertImuReading(const AcclGyr::Sample& sample);
    SensorFusion::BarometerReading convertBaroReading(const Bmp585Test::Sample& sample);
    SensorFusion::ToFReading convertTofReading(const TofDistance::Sample& sample);
    SensorFusion::OpticalFlowReading convertFlowReading(const OpticalFlowTest::Sample& sample);
  };
}

#endif
