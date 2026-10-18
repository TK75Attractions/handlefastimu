#pragma once
#include "Wire.h"
struct calData { bool valid; float accelBias[3]; float gyroBias[3]; };
struct AccelData { float accelX, accelY, accelZ; };
struct GyroData { float gyroX, gyroY, gyroZ; };
struct MagData { float magX, magY, magZ; };
struct IMUBase {
  virtual ~IMUBase() = default;
  virtual int init(calData&, uint8_t) = 0;
  virtual bool hasMagnetometer() { return false; }
  virtual void update() {}
  virtual void getAccel(AccelData*) = 0;
  virtual void getGyro(GyroData*) = 0;
  virtual void getMag(MagData*) {}
  virtual void calibrateMag(calData*) {}
  virtual void calibrateAccelGyro(calData*) {}
};
template<uint8_t Identity> struct MockImu : IMUBase {
  TwoWire& bus;
  uint8_t address = 0;
  explicit MockImu(TwoWire& wire) : bus(wire) {}
  int init(calData&, uint8_t value) override {
    address = value;
    // Match FastIMU's independent identity check during every initialization.
    bus.beginTransmission(address); bus.write(0x75);
    if (bus.endTransmission(false) != 0 || bus.requestFrom(address, 1) != 1) return -1;
    return bus.read() == Identity ? 0 : -1;
  }
  void getAccel(AccelData* out) override { *out = {-1, 0, 0}; }
  void getGyro(GyroData* out) override { *out = {.1f, .2f, address == 0x69 ? .6f : .3f}; }
};
using MPU6050 = MockImu<0x68>;
using MPU6500 = MockImu<0x70>;
using MPU9250 = MockImu<0x71>;
using MPU9255 = MockImu<0x73>;
using MPU6515 = MockImu<0x74>;
using MPU6886 = MockImu<0x19>;
using ICM20689 = MockImu<0x98>;
using ICM20690 = MockImu<0x20>;
using IMU_Generic = MockImu<0x75>;
