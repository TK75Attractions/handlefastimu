// imu_manager.cpp
// Implementation of the simple IMU manager that encapsulates detection,
// selection, initialization and basic read/update operations for the
// FastIMU-based drivers included in the project.

#include "imu_manager.hpp"
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

// Include FastIMU device implementations
#include "FastIMU.h"

ImuManager::ImuManager(TwoWire& wireIn)
  : wire(wireIn), imuMpu6050(wireIn), imuMpu6500(wireIn),
    imuMpu9250(wireIn), imuMpu9255(wireIn), imuMpu6515(wireIn),
    imuMpu6886(wireIn), imuIcm20689(wireIn), imuIcm20690(wireIn), imuGeneric(wireIn) {}

// Probe a single I2C address. Returns true if a device ACKs.
bool ImuManager::probeI2CAddress(uint8_t address) {
  wire.beginTransmission(address);
  return wire.endTransmission(true) == 0;
}

// Read WHO_AM_I register from a device at the given address.
uint8_t ImuManager::readWhoAmILocal(uint8_t address) {
  wire.beginTransmission(address);
  wire.write((uint8_t)0x75);
  // Keep the register pointer and read in one transaction, as FastIMU does.
  // Devices requiring a repeated START can ACK scans but fail identification.
  if (wire.endTransmission(false) != 0) {
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
    // FastIMU's IMUIdentifier maps this MPU-compatible identity to IMU_Generic.
    // Keep it explicit: unknown identities must not use a permissive fallback.
    case 0x75: imu = &imuGeneric; break;
    case 0x19: imu = &imuMpu6886; break;
    case 0x98: imu = &imuIcm20689; break;
    case 0x20: imu = &imuIcm20690; break;
    default:
      imu = nullptr;
      return false;
  }

  // Query magnetometer capability now that imu is chosen.
  // The 0x75-compatible six-axis device has no AK8963 at 0x0C. FastIMU's
  // Generic driver probes/reads that absent address even during update().
  hasMag = whoAmIVal != 0x75 && imu->hasMagnetometer();

  return true;
}

uint8_t ImuManager::getWhoAmI() { return whoAmIVal; }
uint8_t ImuManager::getImuAddress() { return imuAddress; }

int ImuManager::initImu(calData& calib) {
  if (!imu) return -1;
  if (whoAmIVal == 0x75) {
    // Same six-axis ranges/configuration as FastIMU Generic, with checked I/O
    // and no magnetometer transactions. Keep this fix in tracked sources.
    if (readWhoAmILocal(imuAddress) != 0x75 || !writeRegister(0x6B, 0x80)) return -1;
    delay(100);
    if (!writeRegister(0x6B, 0x00)) return -1;
    delay(100);
    if (!writeRegister(0x6B, 0x01)) return -1;
    delay(200);
    const uint8_t config[][2] = {{0x1A, 0x03}, {0x19, 0x02}, {0x1B, 0x18},
      {0x1C, 0x18}, {0x1D, 0x03}, {0x37, 0x00}, {0x38, 0x01}};
    for (const auto& setting : config)
      if (!writeRegister(setting[0], setting[1])) return -1;
    delay(100);
    genericCalibration = calib.valid ? calib : calData{};
    genericAccel = {NAN, NAN, NAN};
    genericGyro = {NAN, NAN, NAN};
    return 0;
  }
  return imu->init(calib, imuAddress);
}

bool ImuManager::writeRegister(uint8_t reg, uint8_t value) {
  wire.beginTransmission(imuAddress);
  wire.write(reg);
  wire.write(value);
  return wire.endTransmission(true) == 0;
}
bool ImuManager::readRegisters(uint8_t reg, uint8_t* data, uint8_t count) {
  wire.beginTransmission(imuAddress);
  wire.write(reg);
  if (wire.endTransmission(false) != 0 || wire.requestFrom(int(imuAddress), int(count)) != count)
    return false;
  for (uint8_t i = 0; i < count; ++i) data[i] = uint8_t(wire.read());
  return true;
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

  GyroData gyro = {};
  float gyroBiasX = 0.0f;
  float gyroBiasY = 0.0f;
  float gyroBiasZ = 0.0f;

  for (uint16_t i = 0; i < sampleCount; ++i) {
    update();
    getGyro(&gyro);
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
  if (whoAmIVal == 0x75) {
    uint8_t raw[14] = {};
    if (!readRegisters(0x3B, raw, sizeof(raw))) {
      // Never pass stale values or uninitialized stack bytes into calibration.
      genericAccel = {NAN, NAN, NAN};
      genericGyro = {NAN, NAN, NAN};
      return;
    }
    const auto sample = [&](uint8_t offset) {
      const uint16_t bits = uint16_t(raw[offset]) << 8 | raw[offset + 1];
      return bits < 0x8000 ? int32_t(bits) : int32_t(bits) - 65536;
    };
    constexpr float accelScale = 16.0f / 32768.0f;
    constexpr float gyroScale = 2000.0f / 32768.0f;
    genericAccel = {sample(0) * accelScale - genericCalibration.accelBias[0],
      sample(2) * accelScale - genericCalibration.accelBias[1],
      sample(4) * accelScale - genericCalibration.accelBias[2]};
    genericGyro = {sample(8) * gyroScale - genericCalibration.gyroBias[0],
      sample(10) * gyroScale - genericCalibration.gyroBias[1],
      sample(12) * gyroScale - genericCalibration.gyroBias[2]};
    return;
  }
  imu->update();
}

void ImuManager::getAccel(AccelData* out) {
  if (!imu || !out) return;
  if (whoAmIVal == 0x75) { *out = genericAccel; return; }
  imu->getAccel(out);
}

void ImuManager::getGyro(GyroData* out) {
  if (!imu || !out) return;
  if (whoAmIVal == 0x75) { *out = genericGyro; return; }
  imu->getGyro(out);
}

void ImuManager::getMag(MagData* out) {
  if (!imu || !out) return;
  imu->getMag(out);
}

bool ImuManager::hasMagnetometer() { return hasMag; }
