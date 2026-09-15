// imu_manager.cpp
// Implementation of the simple IMU manager that encapsulates detection,
// selection, initialization and basic read/update operations for the
// FastIMU-based drivers included in the project.

#include "imu_manager.hpp"
#include <Wire.h>
#include <Arduino.h>

// Include FastIMU device implementations
#include "FastIMU.h"

ImuManager::ImuManager(TwoWire& wireIn)
  : wire(wireIn), imuMpu6050(wireIn), imuMpu6500(wireIn),
    imuMpu9250(wireIn), imuMpu9255(wireIn), imuMpu6515(wireIn),
    imuMpu6886(wireIn), imuIcm20689(wireIn), imuIcm20690(wireIn) {}

// Probe a single I2C address. Returns true if a device ACKs.
bool ImuManager::probeI2CAddress(uint8_t address) {
  wire.beginTransmission(address);
  return wire.endTransmission(true) == 0;
}

// Read WHO_AM_I register from a device at the given address.
uint8_t ImuManager::readWhoAmILocal(uint8_t address) {
  wire.beginTransmission(address);
  wire.write((uint8_t)0x75);
  if (wire.endTransmission(true) != 0) {
    return 0xFF;
  }
  if (wire.requestFrom((int)address, 1) != 1) {
    return 0xFF;
  }
  return wire.read();
}

bool ImuManager::selectAtAddress(uint8_t address) {
  imu = nullptr;
  imuAddress = 0;
  whoAmIVal = 0xFF;
  hasMag = false;
  if (!probeI2CAddress(address)) return false;
  imuAddress = address;

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

uint8_t ImuManager::getWhoAmI() { return whoAmIVal; }
uint8_t ImuManager::getImuAddress() { return imuAddress; }

int ImuManager::initImu(calData& calib) {
  if (!imu) return -1;
  return imu->init(calib, imuAddress);
}

void ImuManager::calibrateMag(calData* calib) {
  if (!imu || !hasMag || !calib) return;
  imu->calibrateMag(calib);
}

void ImuManager::calibrateAccelGyro(calData* calib) {
  if (!imu || !calib) return;
  imu->calibrateAccelGyro(calib);
}

void ImuManager::calibrateGyroOnly(calData* calib, uint16_t sampleCount, uint16_t sampleDelayMs) {
  if (!imu || !calib || sampleCount == 0) return;

  GyroData gyro = {0};
  float gyroBiasX = 0.0f;
  float gyroBiasY = 0.0f;
  float gyroBiasZ = 0.0f;

  for (uint16_t i = 0; i < sampleCount; ++i) {
    imu->update();
    imu->getGyro(&gyro);
    gyroBiasX += gyro.gyroX;
    gyroBiasY += gyro.gyroY;
    gyroBiasZ += gyro.gyroZ;
    delay(sampleDelayMs);
  }

  calib->accelBias[0] = 0.0f;
  calib->accelBias[1] = 0.0f;
  calib->accelBias[2] = 0.0f;
  calib->gyroBias[0] = gyroBiasX / sampleCount;
  calib->gyroBias[1] = gyroBiasY / sampleCount;
  calib->gyroBias[2] = gyroBiasZ / sampleCount;
  calib->valid = true;
}

void ImuManager::update() {
  if (!imu) return;
  imu->update();
}

void ImuManager::getAccel(AccelData* out) {
  if (!imu || !out) return;
  imu->getAccel(out);
}

void ImuManager::getGyro(GyroData* out) {
  if (!imu || !out) return;
  imu->getGyro(out);
}

void ImuManager::getMag(MagData* out) {
  if (!imu || !out) return;
  imu->getMag(out);
}

bool ImuManager::hasMagnetometer() { return hasMag; }
