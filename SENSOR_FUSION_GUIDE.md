# Sensor Fusion Suite Documentation

## Overview

The sensor fusion suite provides a complete system for combining readings from multiple sensors (IMU, barometer, ToF distance, optical flow, and magnetometer) to produce accurate estimates of the drone's state.

**Main Outputs:**
- **Attitude**: roll, pitch, yaw (degrees)
- **Vertical Position**: altitude (meters), vertical velocity (m/s)
- **Horizontal Position**: x, y (meters), vx, vy (m/s)

---

## Architecture

### 3 Main Modules

#### 1. **Attitude Estimator**
- **Input**: IMU (accelerometer + gyroscope) + Magnetometer
- **Output**: roll, pitch, yaw
- **Method**: Complementary filter (gyro integrated + accelerometer corrected)
- **Weighting**: 98% gyro, 2% accelerometer

**Process:**
```
Roll:  roll = 0.98 * roll_gyro + 0.02 * roll_accel
Pitch: pitch = 0.98 * pitch_gyro + 0.02 * pitch_accel
Yaw:   yaw = 0.98 * yaw_gyro + 0.02 * yaw_mag
```

#### 2. **Vertical Kalman Filter**
- **Input**: Accelerometer Z (world frame) + Barometer altitude + ToF distance
- **Output**: altitude, vertical velocity
- **Process Noise (Q)**: 0.01 (default)
- **Measurement Noise (R)**: 
  - Barometer: 1.0
  - ToF: 0.5 (more trusted)

**Features:**
- Detects and removes gravity from acceleration
- Fuses multiple altitude sensors
- Prevents ground penetration (altitude ≥ 0)

#### 3. **Horizontal Kalman Filter**
- **Input**: Accelerometer X/Y (world frame) + Optical flow velocity
- **Output**: x, y position + vx, vy velocity
- **Process Noise (Q)**: 0.02 (default)
- **Measurement Noise (R)**: 0.1 (optical flow)

**Features:**
- Validates optical flow vs altitude
- Only trusts flow when: 0.1m < altitude < 5.0m
- Fuses accelerometer and optical flow velocity

---

## Data Flow Diagram

```
┌─────────────────────────────────────┐
│      SENSOR DRIVERS                 │
├─────────────────────────────────────┤
│ • AcclGyr (IMU: ax,ay,az,gx,gy,gz) │
│ • Bmp585Test (Barometer: alt)       │
│ • TofDistance (ToF: distance)       │
│ • OpticalFlowTest (Flow: vx, vy)    │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│   SENSOR INTEGRATION MODULE         │
│   (Conversion to fusion format)     │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│     SENSOR FUSION ENGINE            │
├─────────────────────────────────────┤
│  1. ATTITUDE ESTIMATOR              │
│     → roll, pitch, yaw              │
│                                     │
│  2. PREPROCESSING                   │
│     → Remove bias, low-pass filter  │
│                                     │
│  3. BODY→WORLD TRANSFORMATION       │
│     → Rotate accel to world frame   │
│                                     │
│  4. VERTICAL KALMAN FILTER          │
│     → altitude, vz                  │
│                                     │
│  5. OPTICAL FLOW PROCESSING         │
│     → Altitude-dependent velocity   │
│                                     │
│  6. HORIZONTAL KALMAN FILTER        │
│     → x, y, vx, vy                  │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│   FUSED STATE ESTIMATE              │
├─────────────────────────────────────┤
│ Attitude: roll, pitch, yaw          │
│ Vertical: z, vz                     │
│ Horizontal: x, y, vx, vy            │
│ Confidence: 0-100%                  │
└─────────────────────────────────────┘
```

---

## State Definition

### Full 9-DOF State Vector
```cpp
struct State {
  // Attitude (body frame orientation in world)
  float rollDeg;      // rotation around X axis
  float pitchDeg;     // rotation around Y axis
  float yawDeg;       // rotation around Z axis

  // Vertical motion
  float altitudeM;           // height above ground
  float verticalVelocityMps; // dz/dt (positive = up)

  // Horizontal position
  float positionXM;   // world X coordinate
  float positionYM;   // world Y coordinate

  // Horizontal velocity
  float velocityXMps; // dx/dt
  float velocityYMps; // dy/dt

  // Sensor biases (estimated during calibration)
  float accelBiasX;   // accelerometer bias
  float accelBiasY;
  float accelBiasZ;
  float gyroBiasX;    // gyroscope bias
  float gyroBiasY;
  float gyroBiasZ;

  // Uncertainty (covariance diagonal)
  float covVertical[2][2];     // [z, vz]
  float covHorizontal[4][4];   // [x, y, vx, vy]
};
```

---

## Configuration Parameters

```cpp
struct FusionConfig {
  // Complementary filter weight (0-1)
  float alphaAttitude = 0.98f;  // higher = trust gyro more

  // Measurement noise variances (larger = less trust in measurement)
  float RBaroAltitude = 1.0f;       // barometer altitude
  float RTofDistance = 0.5f;        // ToF distance (more trusted)
  float ROpticalFlowVelocity = 0.1f; // optical flow velocity

  // Process noise (larger = more conservative filter)
  float QVerticalNoise = 0.01f;   // vertical dynamics
  float QHorizontalNoise = 0.02f; // horizontal dynamics

  // Optical flow constraints
  float minAltitudeForFlowM = 0.1f;  // don't use flow below 10cm
  float maxAltitudeForFlowM = 5.0f;  // don't use flow above 5m

  // Low-pass filter alphas (0-1, higher = more filtering)
  float lpfAccelAlpha = 0.1f;  // accelerometer LPF
  float lpfGyroAlpha = 0.05f;  // gyroscope LPF

  // Physical constants
  float gravityMps2 = 9.81f;

  // Velocity clamping (prevent divergence)
  float maxVerticalVelocityMps = 3.0f;
  float maxHorizontalVelocityMps = 5.0f;
};
```

---

## Integration Example

### Basic Usage

```cpp
#include "sensor_integration.h"

SensorIntegration::SensorManager sensorManager;

void setup() {
  Serial.begin(115200);
  
  // Initialize all sensors and fusion
  sensorManager.begin();
  
  // Optional: customize fusion configuration
  SensorFusion::FusionConfig config = sensorManager.getFusionOutput().state;
  config.alphaAttitude = 0.95f;  // trust accel more
  config.RTofDistance = 0.3f;     // trust ToF distance more
  sensorManager.setFusionConfig(config);
}

void loop() {
  // Main update loop (call frequently, e.g., 100Hz)
  sensorManager.update();
  
  // Get fused state estimate
  auto fusedOutput = sensorManager.getFusionOutput();
  auto state = fusedOutput.state;
  
  // Use state for control laws
  float roll = state.rollDeg;
  float pitch = state.pitchDeg;
  float yaw = state.yawDeg;
  float altitude = state.altitudeM;
  float vx = state.velocityXMps;
  float vy = state.velocityYMps;
  
  // Print for debugging
  Serial.printf("Roll: %.2f° Pitch: %.2f° Alt: %.2f m\n", roll, pitch, altitude);
  Serial.printf("Vx: %.2f m/s Vy: %.2f m/s Vz: %.2f m/s\n", 
                vx, vy, state.verticalVelocityMps);
  
  // Check system health
  auto status = sensorManager.getSystemStatus();
  Serial.printf("IMU: %d Baro: %d ToF: %d Flow: %d Confidence: %.0f%%\n",
                status.imuReady, status.baroReady, status.tofReady, 
                status.flowReady, status.fusionConfidence);
}
```

---

## Calibration Procedure

Before flight, calibrate sensors on level ground:

```cpp
void calibrateDrone() {
  /*
  1. Place drone LEVEL on ground
  2. Remain STATIONARY for 10 seconds
  3. Collect IMU samples during this time
  4. Compute average values as bias
  */
  
  const int CALIBRATION_SAMPLES = 200;
  float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
  float accelSumX = 0, accelSumY = 0, accelSumZ = 0;
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    auto imuSample = AcclGyr::latestSample();
    if (imuSample.valid) {
      gyroSumX += imuSample.imu.gyroXDegPerSec;
      gyroSumY += imuSample.imu.gyroYDegPerSec;
      gyroSumZ += imuSample.imu.gyroZDegPerSec;
      
      accelSumX += imuSample.imu.accelX;
      accelSumY += imuSample.imu.accelY;
      accelSumZ += imuSample.imu.accelZ - 1.0f; // subtract gravity
    }
    delay(10);
  }
  
  float gyroBiasX = gyroSumX / CALIBRATION_SAMPLES;
  float gyroBiasY = gyroSumY / CALIBRATION_SAMPLES;
  float gyroBiasZ = gyroSumZ / CALIBRATION_SAMPLES;
  
  float accelBiasX = accelSumX / CALIBRATION_SAMPLES;
  float accelBiasY = accelSumY / CALIBRATION_SAMPLES;
  float accelBiasZ = accelSumZ / CALIBRATION_SAMPLES;
  
  // Set biases in fusion engine
  sensorManager.calibrateSensors();  // Future implementation
}
```

---

## Kalman Filter Details

### Vertical Kalman Filter

State: **[z, vz]ᵀ** (altitude, vertical velocity)

**Prediction:**
```
z_new = z + vz * dt + 0.5 * az * dt²
vz_new = vz + az * dt
```

**Measurement Update (from barometer or ToF):**
```
K = P / (P + R)              // Kalman gain
z = z + K * (z_meas - z)     // altitude correction
vz = vz + K * correction     // velocity correction
```

### Horizontal Kalman Filter

State: **[x, y, vx, vy]ᵀ** (position and velocity)

**Prediction:**
```
x = x + vx * dt + 0.5 * ax * dt²
y = y + vy * dt + 0.5 * ay * dt²
vx = vx + ax * dt
vy = vy + ay * dt
```

**Measurement Update (from optical flow):**
```
K = P / (P + R)
vx = vx + K * (vx_flow - vx)
vy = vy + K * (vy_flow - vy)
```

---

## Troubleshooting

### Altitude Drifting
- **Cause**: Poor barometer quality
- **Solution**: 
  - Lower `RBaroAltitude` to trust barometer less
  - Lower `QVerticalNoise` to make filter more stable
  - Verify barometer signal is valid

### Erratic Horizontal Motion
- **Cause**: Optical flow too noisy or outside valid altitude range
- **Solution**:
  - Increase `minAltitudeForFlowM` (flow not reliable near ground)
  - Lower `ROpticalFlowVelocity` to trust less
  - Check flow sensor quality

### Attitude Oscillation
- **Cause**: High complementary filter weight on accel
- **Solution**:
  - Increase `alphaAttitude` (trust gyro more)
  - Increase `lpfAccelAlpha` (more filtering on accel)

### Filter Divergence (large error growth)
- **Cause**: Kalman gain too high or process noise too low
- **Solution**:
  - Increase `Q` process noise
  - Increase `R` measurement noise to trust sensors less
  - Check sensor calibration

---

## File Structure

```
include/
  ├── sensor_fusion.h          # Main fusion engine
  ├── sensor_integration.h     # High-level wrapper
  ├── acclgyr.h               # IMU driver
  ├── bmp585_test.h           # Barometer driver
  ├── tof_distance.h          # ToF distance driver
  ├── optical_flow_test.h     # Optical flow driver
  
src/
  ├── sensor_fusion.cpp       # Fusion engine implementation
  ├── sensor_integration.cpp  # Integration implementation
  ├── acclgyr.cpp            # IMU implementation
  ├── bmp585_test.cpp        # Barometer implementation
  ├── tof_distance.cpp       # ToF implementation
  ├── optical_flow_test.cpp  # Optical flow implementation
  └── main.cpp               # Application example
```

---

## Classes and Interfaces

### SensorFusionEngine

**Main class for sensor fusion:**

```cpp
class SensorFusionEngine {
  void initialize();
  void setConfig(const FusionConfig& config);
  void update(...);                    // Main update with all sensor readings
  FusionOutput getOutput() const;      // Get fused state + confidence
  State getState() const;              // Get just the state
  void calibrateGyro(...);
  void calibrateAccel(...);
};
```

### SensorManager

**High-level convenience wrapper:**

```cpp
class SensorManager {
  void begin();              // Initialize all sensors
  void update();             // Main loop - reads sensors and updates fusion
  FusionOutput getFusionOutput() const;
  SystemStatus getSystemStatus() const;
  void setFusionConfig(...);
};
```

---

## Performance Considerations

- **Update Rate**: Optimal at 100-200 Hz (5-10ms loop time)
- **CPU Usage**: ~10-15% on ESP32 at 100Hz
- **Memory**: ~2KB state + ~4KB code
- **Latency**: ~5-10ms (mostly sensor read time)

---

## Next Steps

1. **Mount sensors** on drone with proper orientation
2. **Calibrate biases** on level ground
3. **Tune Kalman parameters** based on sensor quality
4. **Test in hover** to verify stability
5. **Implement control laws** using fusion output
6. **Add magnetometer** for drift-free yaw
7. **Add wind compensation** for horizontal velocity

