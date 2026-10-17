#pragma once
#include "Wire.h"
struct calData { bool valid; float accelBias[3]; float gyroBias[3]; };
struct AccelData { float accelX, accelY, accelZ; };
struct GyroData { float gyroX, gyroY, gyroZ; };
struct MagData { float magX, magY, magZ; };
struct IMUBase {};
struct MockImu : IMUBase { explicit MockImu(TwoWire&) {} };
using MPU6050 = MockImu;
using MPU6500 = MockImu;
using MPU9250 = MockImu;
using MPU9255 = MockImu;
using MPU6515 = MockImu;
using MPU6886 = MockImu;
using ICM20689 = MockImu;
using ICM20690 = MockImu;
