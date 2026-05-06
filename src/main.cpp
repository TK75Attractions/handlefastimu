#include <Arduino.h>
#include "FastIMU.h"
#include <Wire.h>

#include "Madgwick.h"

// 必要に応じてIMU種別とI2Cアドレスを変更してください。
#define IMU_ADDRESS_PRIMARY 0x68
#define IMU_ADDRESS_SECONDARY 0x69

MPU6050 imuMpu6050;
MPU6500 imuMpu6500;
MPU9250 imuMpu9250;
MPU9255 imuMpu9255;
MPU6515 imuMpu6515;
MPU6886 imuMpu6886;
ICM20689 imuIcm20689;
ICM20690 imuIcm20690;

IMUBase* imu = nullptr;

calData calib = {0};
AccelData imuAccel;
GyroData imuGyro;
MagData imuMag;
Madgwick filter;
bool hasMagnetometer = false;
uint8_t imuAddress = IMU_ADDRESS_PRIMARY;

uint8_t whoAmI = 0xFF;

void quaternionToEulerDeg(float w, float x, float y, float z, float& rollDeg, float& pitchDeg, float& yawDeg) {
  const float sinrCosp = 2.0f * (w * x + y * z);
  const float cosrCosp = 1.0f - 2.0f * (x * x + y * y);
  const float roll = atan2f(sinrCosp, cosrCosp);

  const float sinp = 2.0f * (w * y - z * x);
  const float pitch = asinf(constrain(sinp, -1.0f, 1.0f));

  const float sinyCosp = 2.0f * (w * z + x * y);
  const float cosyCosp = 1.0f - 2.0f * (y * y + z * z);
  const float yaw = atan2f(sinyCosp, cosyCosp);

  constexpr float RAD_TO_DEG_F = 57.2957795f;
  rollDeg = roll * RAD_TO_DEG_F;
  pitchDeg = pitch * RAD_TO_DEG_F;
  yawDeg = yaw * RAD_TO_DEG_F;
}

bool probeI2CAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission(true) == 0;
}

uint8_t readWhoAmI(uint8_t address) {
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

bool selectImuByWhoAmI(uint8_t id) {
  switch (id) {
    case 0x68:
      imu = &imuMpu6050;
      return true;
    case 0x70:
      imu = &imuMpu6500;
      return true;
    case 0x71:
      imu = &imuMpu9250;
      return true;
    case 0x73:
      imu = &imuMpu9255;
      return true;
    case 0x74:
      imu = &imuMpu6515;
      return true;
    case 0x19:
      imu = &imuMpu6886;
      return true;
    case 0x98:
      imu = &imuIcm20689;
      return true;
    case 0x20:
      imu = &imuIcm20690;
      return true;
    default:
      imu = nullptr;
      return false;
  }
}

void setup() {
  Wire.begin();
  Wire.setClock(100000);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000);
#endif

  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  if (probeI2CAddress(IMU_ADDRESS_PRIMARY)) {
    imuAddress = IMU_ADDRESS_PRIMARY;
  } else if (probeI2CAddress(IMU_ADDRESS_SECONDARY)) {
    imuAddress = IMU_ADDRESS_SECONDARY;
  } else {
    Serial.println("IMU not found on 0x68/0x69. Check wiring/power.");
    while (true) {
      delay(1000);
    }
  }

  Serial.print("IMU I2C address: 0x");
  Serial.println(imuAddress, HEX);

  whoAmI = readWhoAmI(imuAddress);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);

  if (!selectImuByWhoAmI(whoAmI)) {
    Serial.println("Unsupported or unknown IMU ID for this sketch.");
    while (true) {
      delay(1000);
    }
  }

  int err = imu->init(calib, imuAddress);
  if (err != 0) {
    Serial.print("IMU init error: ");
    Serial.println(err);
    while (true) {
      delay(1000);
    }
  }

  // 磁気センサ有無は起動時に1回だけ判定する。
  hasMagnetometer = imu->hasMagnetometer();

  Serial.println("Start calibration...");

  if (hasMagnetometer) {
    Serial.println("Mag calibration: move the IMU in a figure-8 pattern.");
    delay(3000);
    imu->calibrateMag(&calib);
    Serial.println("Mag calibration done.");
  } else {
    Serial.println("No magnetometer detected. Yaw drift cannot be fully removed.");
  }

  Serial.println("Accel/Gyro calibration: keep the IMU still and level.");
  delay(3000);
  imu->calibrateAccelGyro(&calib);
  Serial.println("Accel/Gyro calibration done.");

  err = imu->init(calib, imuAddress);
  if (err != 0) {
    Serial.print("IMU re-init error: ");
    Serial.println(err);
    while (true) {
      delay(1000);
    }
  }

  // Betaは大きいほどドリフト補正を強める。環境ノイズに応じて調整可。
  filter.begin(hasMagnetometer ? 0.12f : 0.05f);
  Serial.println("Calibration and filter setup complete.");
}

void loop() {
  imu->update();
  imu->getAccel(&imuAccel);
  imu->getGyro(&imuGyro);

  if (hasMagnetometer) {
    imu->getMag(&imuMag);
    filter.update(
      imuGyro.gyroX, imuGyro.gyroY, imuGyro.gyroZ,
      imuAccel.accelX, imuAccel.accelY, imuAccel.accelZ,
      imuMag.magX, imuMag.magY, imuMag.magZ
    );
  } else {
    filter.updateIMU(
      imuGyro.gyroX, imuGyro.gyroY, imuGyro.gyroZ,
      imuAccel.accelX, imuAccel.accelY, imuAccel.accelZ
    );
  }
  /*
  Serial.print("q: ");
  Serial.print(filter.getQuatW(), 6);
  Serial.print(", ");
  Serial.print(filter.getQuatX(), 6);
  Serial.print(", ");
  Serial.print(filter.getQuatY(), 6);
  Serial.print(", ");
  Serial.println(filter.getQuatZ(), 6);
  */

  float rollDeg, pitchDeg, yawDeg;
  quaternionToEulerDeg(
    filter.getQuatW(), filter.getQuatX(), filter.getQuatY(), filter.getQuatZ(),
    rollDeg, pitchDeg, yawDeg
  );

  Serial.print("euler: ");
  Serial.print(rollDeg, 2);
  Serial.print(", ");
  Serial.print(pitchDeg, 2);
  Serial.print(", ");
  Serial.println(yawDeg, 2);

  delay(10);
}