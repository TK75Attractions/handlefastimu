// imu_manager.cpp
// Implementation of the simple IMU manager that encapsulates detection,
// selection, initialization and basic read/update operations for the
// FastIMU-based drivers included in the project.

#include "imu_manager.hpp"
#include <Wire.h>
#include <Arduino.h>

// Include FastIMU device implementations
#include "FastIMU.h"

namespace ImuManager {

// Concrete device instances for all supported drivers. Keeping them as
// file-scoped globals mirrors how the original sketch declared them,
// but the rest of the project uses only the functions below.
static MPU6050 imuMpu6050;
static MPU6500 imuMpu6500;
static MPU9250 imuMpu9250;
static MPU9255 imuMpu9255;
static MPU6515 imuMpu6515;
static MPU6886 imuMpu6886;
static ICM20689 imuIcm20689;
static ICM20690 imuIcm20690;

static IMUBase* imu = nullptr;
static uint8_t imuAddress = 0;
static uint8_t whoAmIVal = 0xFF;
static bool hasMag = false;

// Probe a single I2C address. Returns true if a device ACKs.
static bool probeI2CAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission(true) == 0;
}

// Read WHO_AM_I register from a device at the given address.
static uint8_t readWhoAmILocal(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write((uint8_t)0x75);
  if (Wire.endTransmission(true) != 0) {
    return 0xFF;
  }
  if (Wire.requestFrom((int)address, 1) != 1) {
    return 0xFF;
  }
  return Wire.read();
}

bool probeAndSelect(uint8_t primary, uint8_t secondary) {
  // Choose an I2C address that replies.
  if (probeI2CAddress(primary)) {
    imuAddress = primary;
  } else if (probeI2CAddress(secondary)) {
    imuAddress = secondary;
  } else {
    imu = nullptr;
    imuAddress = 0;
    whoAmIVal = 0xFF;
    return false;
  }

  whoAmIVal = readWhoAmILocal(imuAddress);

  // Map WHO_AM_I to the right class instance.
  switch (whoAmIVal) {
    case 0x68: imu = &imuMpu6050; break;
    case 0x70: imu = &imuMpu6500; break;
    case 0x71: imu = &imuMpu9250; break;
    case 0x73: imu = &imuMpu9255; break;
    case 0x74: imu = &imuMpu6515; break;
    case 0x19: imu = &imuMpu6886; break;
    case 0x98: imu = &imuIcm20689; break;
    case 0x20: imu = &imuIcm20690; break;
    default:
      imu = nullptr;
      return false;
  }

  // Query magnetometer capability now that imu is chosen.
  hasMag = imu->hasMagnetometer();

  return true;
}

uint8_t getWhoAmI() { return whoAmIVal; }
uint8_t getImuAddress() { return imuAddress; }

int initImu(calData& calib) {
  if (!imu) return -1;
  return imu->init(calib, imuAddress);
}

void calibrateMag(calData* calib) {
  if (!imu || !hasMag || !calib) return;
  imu->calibrateMag(calib);
}

void calibrateAccelGyro(calData* calib) {
  if (!imu || !calib) return;
  imu->calibrateAccelGyro(calib);
}

void update() {
  if (!imu) return;
  imu->update();
}

void getAccel(AccelData* out) {
  if (!imu || !out) return;
  imu->getAccel(out);
}

void getGyro(GyroData* out) {
  if (!imu || !out) return;
  imu->getGyro(out);
}

void getMag(MagData* out) {
  if (!imu || !out) return;
  imu->getMag(out);
}

bool hasMagnetometer() { return hasMag; }

} // namespace ImuManager
