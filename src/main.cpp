#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "imu_manager.hpp"
#include "orientation.hpp"

// All inputs share one bus; AD0 selects each MPU6050 address.
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint8_t HANDLE1_ADDRESS = 0x68;
constexpr uint8_t HANDLE2_ADDRESS = 0x69;
constexpr uint8_t ADS_ADDRESS = 0x48;
constexpr uint32_t FILTER_SETTLE_MS = 5000;
constexpr uint32_t RECONNECT_SETTLE_MS = 1200;
constexpr uint32_t RECONNECT_RETRY_MS = 1000;
constexpr float RAD_TO_DEG_F = 57.2957795f;

Adafruit_ADS1115 ads;

struct HandleInput {
  explicit HandleInput(uint8_t addressIn) : address(addressIn), bus(Wire), imu(Wire) {}
  const uint8_t address;
  TwoWire& bus;
  ImuManager imu;
  Orientation orientation;
  calData calib = {};
  AccelData accel = {};
  GyroData gyro = {};
  MagData mag = {};
  bool configured = false;
  bool online = false;
  bool settling = false;
  bool refLocked = false;
  bool hasMag = false;
  uint32_t retryAtMs = 0;
  uint32_t settleStartedMs = 0;
  uint32_t settleDurationMs = 0;
  float restoreBeta = 0.2f;
  float angleDeg = 0.0f;
};

HandleInput handles[] = {HandleInput(HANDLE1_ADDRESS), HandleInput(HANDLE2_ADDRESS)};
bool adsOnline = false;
uint32_t adsRetryAtMs = 0;
float pedals[2] = {};

static bool probe(TwoWire& bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission(true) == 0;
}

static bool consumeTimeout(TwoWire& bus) {
#ifdef WIRE_HAS_TIMEOUT
  if (bus.getWireTimeoutFlag()) {
    bus.clearWireTimeoutFlag();
    return true;
  }
#else
  (void)bus;
#endif
  return false;
}

static bool due(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

static void startSettling(HandleInput& h, uint32_t duration) {
  h.restoreBeta = h.orientation.getBeta();
  if (h.hasMag) h.orientation.changeBeta(1.0f);
  h.settleStartedMs = millis();
  h.settleDurationMs = duration;
  h.settling = true;
}

static bool initializeHandle(HandleInput& h, size_t index) {
  consumeTimeout(h.bus);
  // Never fall back to the other handle's address, including on reconnect.
  if (!h.imu.selectAtAddress(h.address)) return false;
  // This shared-bus configuration is for MPU6050 (WHO_AM_I is 0x68 at both addresses).
  if (h.imu.getWhoAmI() != 0x68) {
    Serial.printf("[HANDLE%u] Expected MPU6050 at 0x%02X.\n", unsigned(index + 1), h.address);
    return false;
  }
  if (h.imu.initImu(h.calib) != 0) return false;
  h.hasMag = h.imu.hasMagnetometer();
  Serial.printf("[HANDLE%u] IMU 0x%02X, WHO_AM_I 0x%02X\n",
                unsigned(index + 1), h.imu.getImuAddress(), h.imu.getWhoAmI());

  if (!h.configured) {
    if (h.hasMag) {
      Serial.printf("[HANDLE%u] Mag calibration: move this IMU in a figure-8.\n", unsigned(index + 1));
      delay(3000);
      h.imu.calibrateMag(&h.calib);
    }
    Serial.printf("[HANDLE%u] Gyro calibration: keep this IMU still.\n", unsigned(index + 1));
    delay(3000);
    h.imu.calibrateGyroOnly(&h.calib);
    if (consumeTimeout(h.bus) || !probe(h.bus, h.imu.getImuAddress()) ||
        h.imu.initImu(h.calib) != 0) return false;
    h.orientation.begin(h.hasMag, Vec3(0, 0, 1), Vec3(-1, 0, 0), h.hasMag ? 0.4f : 0.2f);
    h.configured = true;
    startSettling(h, FILTER_SETTLE_MS);
  } else {
    h.orientation.resetFilterForResync();
    startSettling(h, RECONNECT_SETTLE_MS);
  }
  h.online = true;
  return true;
}

static void updateHandle(HandleInput& h, size_t index) {
  if (!h.online) {
    if (due(millis(), h.retryAtMs) && !initializeHandle(h, index))
      h.retryAtMs = millis() + RECONNECT_RETRY_MS;
    return;
  }

  if (!probe(h.bus, h.imu.getImuAddress()) || consumeTimeout(h.bus)) {
    if (h.settling) h.orientation.changeBeta(h.restoreBeta);
    h.online = false;
    h.settling = false;
    h.retryAtMs = millis() + RECONNECT_RETRY_MS;
    Serial.printf("[HANDLE%u] I2C lost; retrying.\n", unsigned(index + 1));
    return;
  }

  h.imu.update();
  h.imu.getAccel(&h.accel);
  h.imu.getGyro(&h.gyro);
  if (h.hasMag) h.imu.getMag(&h.mag);
  if (consumeTimeout(h.bus)) {
    if (h.settling) h.orientation.changeBeta(h.restoreBeta);
    h.online = false;
    h.settling = false;
    h.retryAtMs = millis() + RECONNECT_RETRY_MS;
    return;
  }
  if (h.hasMag) {
    h.orientation.updateWithMag(h.gyro.gyroX, h.gyro.gyroY, h.gyro.gyroZ,
      h.accel.accelX, h.accel.accelY, h.accel.accelZ, h.mag.magX, h.mag.magY, h.mag.magZ);
  } else {
    h.orientation.updateIMU(h.gyro.gyroX, h.gyro.gyroY, h.gyro.gyroZ,
      h.accel.accelX, h.accel.accelY, h.accel.accelZ);
  }
  if (h.settling) {
    if (uint32_t(millis() - h.settleStartedMs) < h.settleDurationMs) return;
    h.orientation.changeBeta(h.restoreBeta);
    if (!h.refLocked) {
      h.orientation.restartAngleTracking();
      h.refLocked = true;
    }
    h.settling = false;
    Serial.printf("[HANDLE%u] Ready.\n", unsigned(index + 1));
  }
  h.angleDeg = -h.orientation.shaftAngleRad() * RAD_TO_DEG_F;
}

static void updatePedals() {
  if (!adsOnline) {
    if (!due(millis(), adsRetryAtMs)) return;
    consumeTimeout(Wire);
    adsOnline = ads.begin(ADS_ADDRESS, &Wire);
    if (!adsOnline) {
      adsRetryAtMs = millis() + RECONNECT_RETRY_MS;
      return;
    }
    ads.setGain(GAIN_ONE);
    Serial.println("[PEDAL] ADS1115 ready: A0=pedal1, A1=pedal2.");
  }
  if (!probe(Wire, ADS_ADDRESS)) {
    adsOnline = false;
    adsRetryAtMs = millis() + RECONNECT_RETRY_MS;
    return;
  }
  for (uint8_t channel = 0; channel < 2; ++channel) {
    const int16_t raw = ads.readADC_SingleEnded(channel);
    if (consumeTimeout(Wire) || !probe(Wire, ADS_ADDRESS)) {
      adsOnline = false;
      adsRetryAtMs = millis() + RECONNECT_RETRY_MS;
      return;
    }
    pedals[channel] = constrain(ads.computeVolts(raw) / 3.3f, 0.0f, 1.0f);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  Serial.println("[INPUT] CSV: pedal1,handle1_deg,pedal2,handle2_deg");
  // Complete blocking startup calibrations before either live filter starts.
  for (size_t i = 0; i < 2; ++i) {
    if (!initializeHandle(handles[i], i)) {
      handles[i].retryAtMs = millis() + RECONNECT_RETRY_MS;
      Serial.printf("[HANDLE%u] Not found; retrying.\n", unsigned(i + 1));
    }
  }
  for (auto& h : handles) {
    if (h.online) h.settleStartedMs = millis();
  }
}

void loop() {
  while (Serial.available()) {
    const char c = char(Serial.read());
    for (auto& h : handles) {
      if (!h.configured) continue;
      if (c == 'z' || c == 'Z') {
        h.orientation.restartAngleTracking();
        h.refLocked = true;
      } else if (c == 'b' || c == 'B') {
        const float beta = constrain((h.settling ? h.restoreBeta : h.orientation.getBeta())
                                    + (c == 'b' ? 0.05f : -0.05f), 0.01f, 1.0f);
        if (h.settling) h.restoreBeta = beta;
        else h.orientation.changeBeta(beta);
      }
    }
  }
  updatePedals();
  for (size_t i = 0; i < 2; ++i) updateHandle(handles[i], i);
  // Missing inputs are explicit; never emit an old value as a live reading.
  const float missing = NAN;
  Serial.printf("%.2f,%.2f,%.2f,%.2f\n",
    adsOnline ? pedals[0] : missing,
    handles[0].online && !handles[0].settling ? handles[0].angleDeg : missing,
    adsOnline ? pedals[1] : missing,
    handles[1].online && !handles[1].settling ? handles[1].angleDeg : missing);
  delay(10);
}
