// imu_manager.hpp
// High-level wrapper around FastIMU sensor selection, initialization,
// calibration and simple read/update helpers.
// This header exposes a small, self-contained API so higher-level code
// (e.g., `main.cpp`) can remain concise and focused on application flow.

#pragma once

#include <stdint.h>
#include "FastIMU.h"

class ImuManager {
public:
  explicit ImuManager(TwoWire& wire = Wire);

  // Select only this address; never substitute another device on a shared bus.
  bool selectAtAddress(uint8_t address);

  // Return device WHO_AM_I (0xFF if read failed before selection).
  uint8_t getWhoAmI();

  // Return selected I2C address or 0 if none.
  uint8_t getImuAddress();

  // Initialize the selected IMU using the provided calibration structure.
  // Returns 0 on success or an error code from the underlying driver.
  int initImu(calData& calib);

  // Perform magnetometer and accel/gyro calibration helpers.
  // These wrap the FastIMU calls and will block until calibration completes.
  void calibrateMag(calData* calib);
  void calibrateAccelGyro(calData* calib);
  void calibrateGyroOnly(calData* calib, uint16_t sampleCount = 400, uint16_t sampleDelayMs = 5);

  // Update sensor internal state and copy latest sensor samples to outputs.
  void update();
  void getAccel(AccelData* out);
  void getGyro(GyroData* out);
  void getMag(MagData* out);

  // Query whether the current IMU has a magnetometer.
  bool hasMagnetometer();

private:
  TwoWire& wire;
  MPU6050 imuMpu6050;
  MPU6500 imuMpu6500;
  MPU9250 imuMpu9250;
  MPU9255 imuMpu9255;
  MPU6515 imuMpu6515;
  MPU6886 imuMpu6886;
  ICM20689 imuIcm20689;
  ICM20690 imuIcm20690;
  IMUBase* imu = nullptr;
  uint8_t imuAddress = 0;
  uint8_t whoAmIVal = 0xFF;
  bool hasMag = false;
  bool probeI2CAddress(uint8_t address);
  uint8_t readWhoAmILocal(uint8_t address);
};
