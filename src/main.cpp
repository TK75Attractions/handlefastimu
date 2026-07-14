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


// ADS
Adafruit_ADS1115 ads;

int16_t raw;
float voltage;
float pedalPercent = 0.0f;

constexpr uint32_t FILTER_SETTLE_MS = 5000;

// Application-level state and buffers. Keep these local to the sketch
// file so modules own their own internal state.
static calData calib = {0};
static AccelData imuAccel;
static GyroData imuGyro;
static MagData imuMag;

static bool refLocked = false;
static uint32_t startupStableUntil = 0;

// Small helper constant for radians→degrees conversion used below.
constexpr float RAD_TO_DEG_F = 57.2957795f;

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
  // ハンドルの回転軸はIMUのZ軸。
  const Vec3 shaftAxisWorld = normalize(Vec3(0.0f, 0.0f, 1.0f));
  const Vec3 shaftAxisSensor = normalize(Vec3(0.0f, 0.0f, 1.0f));
  const Vec3 sensorVector = normalize(Vec3(1.0f, 0.0f, 0.0f));

  // IMUの検出と初期化。probeAndSelect()はI2Cアドレスを確認し、利用可能なIMUを選択
  if (!ImuManager::probeAndSelect(IMU_ADDRESS_PRIMARY, IMU_ADDRESS_SECONDARY)) {
    Serial.println("[HANDLE] IMU not found on 0x68/0x69. Check wiring/power.");
    while (true) { delay(1000); }
  }

  Serial.print("[HANDLE] IMU I2C address: 0x");
  Serial.println(ImuManager::getImuAddress(), HEX);

  Serial.print("[HANDLE] WHO_AM_I: 0x");
  Serial.println(ImuManager::getWhoAmI(), HEX);

  // 利用するIMUを初期化し、キャリブレーションデータを取得
  int err = ImuManager::initImu(calib);
  if (err != 0) {
    Serial.print("[HANDLE] IMU init error: ");
    Serial.println(err);
    while (true) { delay(1000); }
  }

  // キャリブレーションの実行。磁気センサーがある場合は磁気キャリブレーションも行う
  const bool hasMag = ImuManager::hasMagnetometer();
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

  // 再度IMUを初期化してキャリブレーションデータを反映
  err = ImuManager::initImu(calib);
  if (err != 0) {
    Serial.print("[HANDLE] IMU re-init error: ");
    Serial.println(err);
    while (true) { delay(1000); }
  }

  // Orientationモジュールの初期化。磁気センサーがある場合はbetaを大きめに設定
  const float initialBeta = hasMag ? 0.4f : 0.2f;
  Orientation::begin(hasMag, shaftAxisWorld, shaftAxisSensor, sensorVector, initialBeta);

  // フィルタが収束してから自動ゼロロックする。
  startupStableUntil = millis() + FILTER_SETTLE_MS;
  Serial.println("[HANDLE] Calibration and filter setup complete.");
  Serial.println("[HANDLE] Send 'z' to zero the current angle. Send 'b'/'B' to tune beta.");
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

  // ADS1115からペダルのアナログ値を読み取り、電圧とペダルの踏み込み率を計算
  raw = ads.readADC_SingleEnded(0);
  voltage = ads.computeVolts(raw);
  pedalPercent = voltage / 3.3f; // Assuming GAIN_ONE


  // IMUの更新とOrientationフィルタの更新。磁気センサーがある場合は磁気データも使用
  ImuManager::update();
  ImuManager::getAccel(&imuAccel);
  ImuManager::getGyro(&imuGyro);

  if (ImuManager::hasMagnetometer()) {
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

  // 自動ゼロロック: フィルタ安定化後、現在の角度をゼロとしてロック
  if (!refLocked && millis() > startupStableUntil) {
    Orientation::lockZeroHere();
    refLocked = true;
  }

  if (!refLocked) {
    delay(10);
    return;
  }

  // ペダル踏み込み率とハンドル角度をシリアル出力。ハンドル角度はラジアンから度に変換
  const float angleDeg = Orientation::shaftAngleRad() * RAD_TO_DEG_F;
  Serial.printf("%.2f,%.2f\n", pedalPercent, -angleDeg);

  delay(10);
}
