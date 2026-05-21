// imu_manager.hpp
// High-level wrapper around FastIMU sensor selection, initialization,
// calibration and simple read/update helpers.
// This header exposes a small, self-contained API so higher-level code
// (e.g., `main.cpp`) can remain concise and focused on application flow.

#pragma once

#include <stdint.h>
#include "FastIMU.h"

namespace ImuManager {

// Probe I2C addresses and select an available IMU. Returns true on success.
bool probeAndSelect(uint8_t primary = 0x68, uint8_t secondary = 0x69);

// Return device WHO_AM_I (0xFF if read failed before selection).
uint8_t getWhoAmI();

// Return selected I2C address (primary/secondary) or 0 if none.
uint8_t getImuAddress();

// Initialize the selected IMU using the provided calibration structure.
// Returns 0 on success or an error code from the underlying driver.
int initImu(calData& calib);

// Perform magnetometer and accel/gyro calibration helpers.
// These wrap the FastIMU calls and will block until calibration completes.
void calibrateMag(calData* calib);
void calibrateAccelGyro(calData* calib);

// Update sensor internal state and copy latest sensor samples to outputs.
void update();
void getAccel(AccelData* out);
void getGyro(GyroData* out);
void getMag(MagData* out);

// Query whether the current IMU has a magnetometer.
bool hasMagnetometer();

} // namespace ImuManager
