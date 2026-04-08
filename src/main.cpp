#include <Arduino.h>
#include <Wire.h>
#include "7Semi_ISM330DHCX.h"
#include "7Semi_MMC5983MA.h"

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define ISM330_I2C_ADDR 0x6B
#define MMC5983MA_I2C_ADDR MMC5983MA_7Semi::DEFAULT_ADDRESS

ISM330DHCX_7Semi imu;
MMC5983MA_7Semi mag;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  Serial.println("SmartElex 9DoF IMU test starting...");
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  bool imu_ok = imu.begin(Wire, ISM330_I2C_ADDR);
  if (!imu_ok) {
    Serial.println("ISM330DHCX not found at 0x6B, trying 0x6A...");
    imu_ok = imu.begin(Wire, 0x6A);
  }
  if (!imu_ok) {
    Serial.println("ERROR: ISM330DHCX not detected on I2C.");
    while (1) {
      delay(1000);
    }
  }

  bool mag_ok = mag.beginI2C(Wire, MMC5983MA_I2C_ADDR, 400000);
  if (!mag_ok) {
    Serial.println("ERROR: MMC5983MA not detected on I2C.");
    while (1) {
      delay(10000);
    }
  }

  mag.enableContinuousMode(true, MMC5983MA_7Semi::CM_10Hz);

  Serial.println("ISM330DHCX and MMC5983MA detected!");
  Serial.print("I2C SDA pin: ");
  Serial.println(I2C_SDA_PIN);
  Serial.print("I2C SCL pin: ");
  Serial.println(I2C_SCL_PIN);
  Serial.println();
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float imu_temp;
  float mx, my, mz;
  float mag_temp;

  if (!imu.readAccel(ax, ay, az)) {
    Serial.println("Failed to read accel");
  }
  if (!imu.readGyro(gx, gy, gz)) {
    Serial.println("Failed to read gyro");
  }
  if (!imu.readTemp(imu_temp)) {
    Serial.println("Failed to read IMU temp");
  }

  if (!mag.readMagnetometer(mx, my, mz)) {
    Serial.println("Failed to read magnetometer");
  }
  if (!mag.readTemperature(mag_temp)) {
    Serial.println("Failed to read mag temp");
  }

  Serial.print("Accel (g): ");
  Serial.print(ax, 3); Serial.print(", ");
  Serial.print(ay, 3); Serial.print(", ");
  Serial.println(az, 3);

  Serial.print("Gyro (dps): ");
  Serial.print(gx, 3); Serial.print(", ");
  Serial.print(gy, 3); Serial.print(", ");
  Serial.println(gz, 3);

  Serial.print("IMU Temp (°C): ");
  Serial.println(imu_temp, 2);

  Serial.print("Mag (mG): ");
  Serial.print(mx, 2); Serial.print(", ");
  Serial.print(my, 2); Serial.print(", ");
  Serial.println(mz, 2);

  Serial.print("Mag Temp (°C): ");
  Serial.println(mag_temp, 2);

  Serial.println();
  delay(2500);
}
