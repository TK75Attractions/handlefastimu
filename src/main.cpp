#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_ADS1X15.h>

// Split responsibilities: IMU hardware handling is in imu_manager, while
// filter/quaternion math is in orientation. This keeps this file focused
// on high-level application flow (setup/loop) and makes it easier to add
// new features.

#include "imu_manager.hpp"
#include "orientation.hpp"

// Necessary I2C addresses; change if your hardware uses other pins.
#define IMU_ADDRESS_PRIMARY 0x68
#define IMU_ADDRESS_SECONDARY 0x69
#define ADS_ADDRESS 0x48


// ADS
Adafruit_ADS1115 ads;

int16_t raw;
float voltage;
float pedalPercent = 0.0f;

// Tilt of the handle/IMU Z axis away from world vertical.
// 0 = vertical Z axis, 90 = horizontal Z axis tilted toward world +X.
constexpr float SHAFT_Z_TILT_DEG = 90.0f;
constexpr uint32_t FILTER_SETTLE_MS = 5000;
constexpr uint32_t RECONNECT_FILTER_SETTLE_MS = 1200;
constexpr uint32_t RECONNECT_RETRY_MS = 1000;
constexpr uint16_t FILTER_SETTLE_SAMPLE_DELAY_MS = 5;

// Application-level state and buffers. Keep these local to the sketch
// file so modules own their own internal state.
static calData calib = {0};
static AccelData imuAccel;
static GyroData imuGyro;
static MagData imuMag;

static bool refLocked = false;
static bool i2cLinkActive = false;
static uint32_t nextReconnectAttemptMs = 0;
static bool hasMag = false;
static Vec3 shaftAxisWorld;
static Vec3 shaftAxisSensor;
static Vec3 sensorVector;
static float initialBeta = 0.2f;

// Small helper constant for radians→degrees conversion used below.
constexpr float RAD_TO_DEG_F = 57.2957795f;

static bool probeI2CAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission(true) == 0;
}

static bool isI2CBusHealthy() {
#ifdef WIRE_HAS_TIMEOUT
  if (Wire.getWireTimeoutFlag()) {
    return false;
  }
#endif
  return probeI2CAddress(ImuManager::getImuAddress()) && probeI2CAddress(ADS_ADDRESS);
}

static bool consumeWireTimeout() {
#ifdef WIRE_HAS_TIMEOUT
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    return true;
  }
#endif
  return false;
}

static bool updateOrientationFromImu() {
  ImuManager::update();
  ImuManager::getAccel(&imuAccel);
  ImuManager::getGyro(&imuGyro);

  if (hasMag) {
    ImuManager::getMag(&imuMag);
    Orientation::updateWithMag(
      imuGyro.gyroX, imuGyro.gyroY, imuGyro.gyroZ,
      imuAccel.accelX, imuAccel.accelY, imuAccel.accelZ,
      imuMag.magX, imuMag.magY, imuMag.magZ
    );
  } else {
    Orientation::updateIMU(
      imuGyro.gyroX, imuGyro.gyroY, imuGyro.gyroZ,
      imuAccel.accelX, imuAccel.accelY, imuAccel.accelZ
    );
  }

  return !consumeWireTimeout();
}

static bool settleOrientationFilter(uint32_t settleMs, bool resetFilter) {
  if (resetFilter) {
    Orientation::resetFilterForResync();
  }

  const float restoreBeta = Orientation::getBeta();
  if (hasMag) {
    Orientation::changeBeta(1.0f);
  }

  if (!isI2CBusHealthy()) {
    Orientation::changeBeta(restoreBeta);
    return false;
  }

  const uint32_t startMs = millis();
  do {
    if (!updateOrientationFromImu()) {
      Orientation::changeBeta(restoreBeta);
      return false;
    }
    delay(FILTER_SETTLE_SAMPLE_DELAY_MS);
  } while ((uint32_t)(millis() - startMs) < settleMs);

  Orientation::changeBeta(restoreBeta);
  return isI2CBusHealthy();
}

static void markI2CLost() {
  if (!i2cLinkActive) {
    return;
  }

  i2cLinkActive = false;
  nextReconnectAttemptMs = millis() + RECONNECT_RETRY_MS;
  Serial.println("[HANDLE] I2C lost. Stopping output and retrying scan...");
}

static bool initializeHandle(bool performCalibration, bool resetOrientation = false) {
  const bool wasZeroLocked = refLocked;

  if (!ImuManager::probeAndSelect(IMU_ADDRESS_PRIMARY, IMU_ADDRESS_SECONDARY)) {
    return false;
  }

  Serial.print("[HANDLE] IMU I2C address: 0x");
  Serial.println(ImuManager::getImuAddress(), HEX);

  Serial.print("[HANDLE] WHO_AM_I: 0x");
  Serial.println(ImuManager::getWhoAmI(), HEX);

  int err = ImuManager::initImu(calib);
  if (err != 0) {
    Serial.print("[HANDLE] IMU init error: ");
    Serial.println(err);
    return false;
  }

  hasMag = ImuManager::hasMagnetometer();

  if (performCalibration) {
    Serial.println("[HANDLE] Start calibration...");
    if (hasMag) {
      Serial.println("[HANDLE] Mag calibration: move the IMU in a figure-8 pattern.");
      delay(3000);
      ImuManager::calibrateMag(&calib);
      Serial.println("[HANDLE] Mag calibration done.");
    } else {
      Serial.println("[HANDLE] No magnetometer detected.");
    }

    Serial.println("[HANDLE] Gyro calibration: keep the pedal and IMU completely still.");
    delay(3000);
    ImuManager::calibrateGyroOnly(&calib);
    Serial.println("[HANDLE] Gyro calibration done.");

    err = ImuManager::initImu(calib);
    if (err != 0) {
      Serial.print("[HANDLE] IMU re-init error: ");
      Serial.println(err);
      return false;
    }
  }

  if (performCalibration) {
    initialBeta = hasMag ? 0.4f : 0.2f;
    Orientation::begin(hasMag, shaftAxisWorld, shaftAxisSensor, sensorVector, initialBeta);
    if (!settleOrientationFilter(FILTER_SETTLE_MS, false)) {
      return false;
    }
    Orientation::lockZeroHere();
    refLocked = true;
  } else if (resetOrientation) {
    initialBeta = hasMag ? 0.4f : 0.2f;
    Orientation::begin(hasMag, shaftAxisWorld, shaftAxisSensor, sensorVector, initialBeta);
    refLocked = wasZeroLocked;
  } else {
    refLocked = wasZeroLocked;
  }
  i2cLinkActive = true;
  nextReconnectAttemptMs = 0;

  Serial.println("[HANDLE] Calibration and filter setup complete.");
  Serial.println("[HANDLE] Send 'z' to zero the current angle. Send 'b'/'B' to tune beta.");
  return true;
}

static bool tryReconnect() {
  if (millis() < nextReconnectAttemptMs) {
    return false;
  }

#ifdef WIRE_HAS_TIMEOUT
  Wire.clearWireTimeoutFlag();
#endif

  if (!probeI2CAddress(ADS_ADDRESS)) {
    nextReconnectAttemptMs = millis() + RECONNECT_RETRY_MS;
    return false;
  }

  if (!initializeHandle(false, false)) {
    nextReconnectAttemptMs = millis() + RECONNECT_RETRY_MS;
    return false;
  }

  if (!settleOrientationFilter(RECONNECT_FILTER_SETTLE_MS, true)) {
    i2cLinkActive = false;
    nextReconnectAttemptMs = millis() + RECONNECT_RETRY_MS;
    return false;
  }

  if (!hasMag && refLocked && !Orientation::hasAbsoluteShaftReference()) {
    refLocked = false;
    Serial.println("[HANDLE] I2C link restored, but this IMU has no magnetometer.");
    Serial.println("[HANDLE] Shaft Z axis is too close to gravity to recover rotation. Send 'z' to zero again.");
    return true;
  }

  Serial.println("[HANDLE] I2C link restored and orientation re-synced. Resuming output.");
  return true;
}

// -------------------- Setup --------------------

void setup() {
  // I2CとSerialの初期化
  Wire.begin();
  Wire.setClock(100000);
#ifdef WIRE_HAS_TIMEOUT
  Wire.setWireTimeout(3000);
#endif

  Serial.begin(115200);
  while (!Serial) { ; }

  if (!ads.begin()) {
    Serial.println("[PEDAL] Failed to initialize ADS1115. Check wiring.");
    while (true) { delay(1000); }
  }

  ads.setGain(GAIN_ONE); // 1x gain = +/-4.096V range (default)
  Serial.println("[PEDAL] ADS1115 initialized.");

  // ここからハンドル部の初期化
  // ハンドルの回転軸はIMUのZ軸。SHAFT_Z_TILT_DEGで世界Z軸からの傾きを指定する。
  const float shaftTilt = SHAFT_Z_TILT_DEG * DEG_TO_RAD;
  shaftAxisWorld = normalize(Vec3(sinf(shaftTilt), 0.0f, cosf(shaftTilt)));
  shaftAxisSensor = normalize(Vec3(0.0f, 0.0f, 1.0f));
  sensorVector = normalize(Vec3(1.0f, 0.0f, 0.0f));

  // IMUの検出と初期化。probeAndSelect()はI2Cアドレスを確認し、利用可能なIMUを選択
  if (!initializeHandle(true)) {
    Serial.println("[HANDLE] Initial I2C setup failed. Check wiring/power.");
    while (true) { delay(1000); }
  }
}

// -------------------- Loop --------------------

void loop() {
  // ゼロロックとbeta調整のためのシリアルコマンド処理(ToDo: 後にUnityに移行予定)
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'z' || c == 'Z') {
      Orientation::lockZeroHere();
      refLocked = true;
      Serial.println("[HANDLE] Zero locked.");
    } else if (c == 'b') {
      float b = Orientation::getBeta() + 0.05f;
      if (b > 1.0f) b = 1.0f;
      Orientation::changeBeta(b);
      Serial.printf("[HANDLE] Beta increased: %.3f\n", b);
    } else if (c == 'B') {
      float b = Orientation::getBeta() - 0.05f;
      if (b < 0.01f) b = 0.01f;
      Orientation::changeBeta(b);
      Serial.printf("[HANDLE] Beta decreased: %.3f\n", b);
    }
  }

  if (!i2cLinkActive) {
    if (tryReconnect()) {
      delay(10);
      return;
    }

    delay(50);
    return;
  }

  if (!isI2CBusHealthy()) {
    markI2CLost();
    delay(10);
    return;
  }

  // ADS1115からペダルのアナログ値を読み取り、電圧とペダルの踏み込み率を計算
  raw = ads.readADC_SingleEnded(0);
  voltage = ads.computeVolts(raw);
  pedalPercent = voltage / 3.3f; // Assuming GAIN_ONE

  if (consumeWireTimeout()) {
    markI2CLost();
    delay(10);
    return;
  }


  // IMUの更新とOrientationフィルタの更新。磁気センサーがある場合は磁気データも使用
  if (!updateOrientationFromImu()) {
    markI2CLost();
    delay(10);
    return;
  }
  if (!refLocked) {
    delay(10);
    return;
  }

  // ペダル踏み込み率とハンドル角度をシリアル出力。ハンドル角度はラジアンから度に変換
  const float angleDeg = Orientation::shaftAngleRad() * RAD_TO_DEG_F;
  Serial.printf("%.2f,%.2f", pedalPercent, -angleDeg);
  // デバッグ用: IMUの生データも出力
  Serial.printf(" || [Log] Accel X: %.2f, Y: %.2f, Z: %.2f | GYRO X: %.2f, Y: %.2f, Z: %.2f", imuAccel.accelX, imuAccel.accelY, imuAccel.accelZ, imuGyro.gyroX, imuGyro.gyroY, imuGyro.gyroZ);
  Serial.println("");

  delay(10);
}
