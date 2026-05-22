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

// Shaft tilt relative to the body frame: 0 = horizontal, 90 = vertical.
constexpr float SHAFT_TILT_DEG = 45.0f;

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
  // Start I2C and serial as usual.
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

  // Compute the shaft axis in body coordinates using simple tilt angle.
  const float theta = SHAFT_TILT_DEG * DEG_TO_RAD;
  const Vec3 shaftAxisBody = normalize(Vec3(0.0f, cosf(theta), sinf(theta)));

  // Detect and select IMU on the bus.
  if (!ImuManager::probeAndSelect(IMU_ADDRESS_PRIMARY, IMU_ADDRESS_SECONDARY)) {
    Serial.println("[HANDLE] IMU not found on 0x68/0x69. Check wiring/power.");
    while (true) { delay(1000); }
  }

  Serial.print("[HANDLE] IMU I2C address: 0x");
  Serial.println(ImuManager::getImuAddress(), HEX);

  Serial.print("[HANDLE] WHO_AM_I: 0x");
  Serial.println(ImuManager::getWhoAmI(), HEX);

  // Initialize the selected IMU; the driver returns 0 on success.
  int err = ImuManager::initImu(calib);
  if (err != 0) {
    Serial.print("[HANDLE] IMU init error: ");
    Serial.println(err);
    while (true) { delay(1000); }
  }

  // Calibrate if possible. Magnetometer calibration is optional.
  const bool hasMag = ImuManager::hasMagnetometer();
  Serial.println("[HANDLE] Start calibration...");
  if (hasMag) {
    Serial.println("[HANDLE] Mag calibration: move the IMU in a figure-8 pattern.");
    delay(3000);
    ImuManager::calibrateMag(&calib);
    Serial.println("[HANDLE] Mag calibration done.");
  } else {
    Serial.println("[HANDLE] No magnetometer detected. Yaw drift cannot be fully removed.");
  }

  Serial.println("[HANDLE] Accel/Gyro calibration: keep the IMU still and level.");
  delay(3000);
  ImuManager::calibrateAccelGyro(&calib);
  Serial.println("[HANDLE] Accel/Gyro calibration done.");

  // Re-initialize with calibration values.
  err = ImuManager::initImu(calib);
  if (err != 0) {
    Serial.print("[HANDLE] IMU re-init error: ");
    Serial.println(err);
    while (true) { delay(1000); }
  }

  // Start orientation filter module with an appropriate beta value.
  const float initialBeta = hasMag ? 0.4f : 0.2f;
  Orientation::begin(hasMag, shaftAxisBody, initialBeta);

  startupStableUntil = millis() + 1000; // delay ~1s before auto zero-lock
  Serial.println("[HANDLE] Calibration and filter setup complete.");
  Serial.println("[HANDLE] Send 'z' to zero the current angle. Send 'b'/'B' to tune beta.");
}

// -------------------- Loop --------------------

void loop() {
  // Handle simple serial commands for runtime tuning and zeroing.
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

  // Read sensors.
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

  // Auto-lock zero after a short stabilization period if not locked yet.
  if (!refLocked && millis() > startupStableUntil) {
    Orientation::lockZeroHere();
    refLocked = true;
  }

  // Compute shaft angle and print a compact CSV-like line for downstream
  // parsing. Keep this formatting unchanged from the original sketch.
  const float angleDeg = Orientation::shaftAngleRad() * RAD_TO_DEG_F;
  Serial.printf("0,%.2f\n", angleDeg);

  // Optional: compute Euler angles for debugging (commented by default).
  // float rollDeg, pitchDeg, yawDeg;
  // Orientation::quaternionToEulerDeg(Orientation::getQuatW(), Orientation::getQuatX(), Orientation::getQuatY(), Orientation::getQuatZ(), rollDeg, pitchDeg, yawDeg);

  delay(10);
}
