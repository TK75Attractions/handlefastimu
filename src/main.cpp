#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_ADS1X15.h>
#include "imu_manager.hpp"
#include "orientation.hpp"
#include "pedal_filter.hpp"

// All inputs share one bus; AD0 selects each MPU6050 address.
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint8_t HANDLE1_ADDRESS = 0x68;
constexpr uint8_t HANDLE2_ADDRESS = 0x69;
constexpr uint8_t ADS_ADDRESS = 0x48;
// GPIO27 reaches A0 and A1 through separate 100 kOhm resistors.
// A low-impedance pedal holds its voltage; an open input follows the probe.
constexpr uint8_t PEDAL_PROBE_PIN = 27;
constexpr uint32_t PEDAL_PROBE_INTERVAL_MS = 250;
constexpr uint16_t PEDAL_PROBE_SETTLE_MS = 20;
constexpr float PEDAL_OPEN_DELTA_VOLTS = 0.8f;
constexpr uint32_t FILTER_SETTLE_MS = 5000;
constexpr uint32_t RECONNECT_SETTLE_MS = 1200;
constexpr uint32_t RECONNECT_RETRY_MS = 1000;
constexpr uint32_t OUTPUT_PERIOD_US = 20000; // 50 Hz
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
PedalFilter pedalFilters[2];
bool pedalConnected[2] = {};
uint32_t nextPedalProbeMs = 0;
uint8_t nextPedalProbeChannel = 0;
int16_t pedalProbeHighRaw = 0;

enum class PedalAdcState : uint8_t {
  Normal0Waiting,
  Normal1Waiting,
  ProbeHighSettling,
  ProbeHighWaiting,
  ProbeLowSettling,
  ProbeLowWaiting
};

PedalAdcState pedalAdcState = PedalAdcState::Normal0Waiting;
uint32_t pedalProbeDeadlineMs = 0;
uint32_t nextOutputUs = 0;
bool diagnosticsEnabled = false;
uint32_t nextDiagnosticsMs = 0;
int16_t pedalRaw[2] = {};
uint32_t pedalSamples[2] = {};
float pedalProbeDelta[2] = {NAN, NAN};

// Unlike an address scan, this verifies that the requested register bytes arrived.
static bool readRegisters(uint8_t address, uint8_t reg, uint8_t* data, uint8_t count) {
  // Avoid issuing repeated-start reads to an absent device (ESP32 logs errors).
  Wire.beginTransmission(address);
  if (Wire.endTransmission(true) != 0) return false;
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

static void printDiagnostics() {
  Serial.printf("[DIAG] uptime_ms=%lu SDA=%u SCL=%u\n",
                (unsigned long)millis(), I2C_SDA, I2C_SCL);
  for (size_t i = 0; i < 2; ++i) {
    auto& h = handles[i];
    uint8_t who = 0, power = 0, data[14] = {};
    const bool whoOk = readRegisters(h.address, 0x75, &who, 1);
    const bool powerOk = readRegisters(h.address, 0x6B, &power, 1);
    const bool rawOk = readRegisters(h.address, 0x3B, data, sizeof(data));
    Serial.printf("[DIAG HANDLE%u] addr=0x%02X who_read=%u WHO_AM_I=0x%02X power_read=%u PWR_MGMT_1=0x%02X raw_read=%u online=%u settling=%u angle=%.2f\n",
      unsigned(i + 1), h.address, whoOk, who, powerOk, power, rawOk,
      h.online, h.settling, h.angleDeg);
    if (rawOk) {
      int16_t values[7];
      for (uint8_t j = 0; j < 7; ++j)
        values[j] = int16_t((uint16_t(data[2*j]) << 8) | data[2*j+1]);
      Serial.printf("[DIAG HANDLE%u RAW] accel=%d,%d,%d temp=%d gyro=%d,%d,%d\n",
        unsigned(i + 1), values[0], values[1], values[2], values[3], values[4], values[5], values[6]);
      Serial.printf("[DIAG HANDLE%u DRIVER] accel_g=%.4f,%.4f,%.4f gyro_dps=%.3f,%.3f,%.3f\n",
        unsigned(i + 1), h.accel.accelX, h.accel.accelY, h.accel.accelZ,
        h.gyro.gyroX, h.gyro.gyroY, h.gyro.gyroZ);
    }
  }
  uint8_t config[2] = {}, conversion[2] = {};
  const bool configOk = readRegisters(ADS_ADDRESS, 0x01, config, 2);
  const bool conversionOk = readRegisters(ADS_ADDRESS, 0x00, conversion, 2);
  Serial.printf("[DIAG ADS] addr=0x%02X config_read=%u config=0x%04X conversion_read=%u conversion_raw=%d online=%u state=%u\n",
    ADS_ADDRESS, configOk, unsigned((uint16_t(config[0]) << 8) | config[1]), conversionOk,
    int16_t((uint16_t(conversion[0]) << 8) | conversion[1]), adsOnline, unsigned(pedalAdcState));
  for (uint8_t i = 0; i < 2; ++i)
    Serial.printf("[DIAG A%u] samples=%lu raw=%d volts=%.4f probe_delta_V=%.4f connected=%u\n",
      i, (unsigned long)pedalSamples[i], pedalRaw[i], ads.computeVolts(pedalRaw[i]),
      pedalProbeDelta[i], pedalConnected[i]);
}

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

static uint16_t pedalMux(uint8_t channel) {
  return channel == 0 ? ADS1X15_REG_CONFIG_MUX_SINGLE_0
                      : ADS1X15_REG_CONFIG_MUX_SINGLE_1;
}

static void markAdsOffline() {
  pinMode(PEDAL_PROBE_PIN, INPUT);
  adsOnline = false;
  pedalConnected[0] = false;
  pedalConnected[1] = false;
  pedalFilters[0].reset();
  pedalFilters[1].reset();
  adsRetryAtMs = millis() + RECONNECT_RETRY_MS;
}

static bool startPedalConversion(uint8_t channel) {
  ads.startADCReading(pedalMux(channel), false);
  if (!consumeTimeout(Wire)) return true;
  markAdsOffline();
  return false;
}

static bool readCompletedPedalConversion(int16_t& raw) {
  const bool complete = ads.conversionComplete();
  if (consumeTimeout(Wire)) {
    markAdsOffline();
    return false;
  }
  if (!complete) return false;

  raw = ads.getLastConversionResults();
  if (!consumeTimeout(Wire)) return true;
  markAdsOffline();
  return false;
}

static void startSettling(HandleInput& h, uint32_t duration) {
  h.restoreBeta = h.orientation.getBeta();
  if (h.hasMag) h.orientation.changeBeta(1.0f);
  h.settleStartedMs = millis();
  h.settleDurationMs = duration;
  h.settling = true;
}

static bool calibrateHandle(HandleInput& h, size_t index) {
  if (!h.online) {
    Serial.printf("[HANDLE%u] Calibration skipped: IMU is offline.\n", unsigned(index + 1));
    return false;
  }

  if (h.hasMag) {
    Serial.printf("[HANDLE%u] Mag calibration: move this IMU in a figure-8.\n", unsigned(index + 1));
    delay(3000);
    h.imu.calibrateMag(&h.calib);
  }
  Serial.printf("[HANDLE%u] Gyro calibration: keep this IMU still.\n", unsigned(index + 1));
  delay(3000);
  h.imu.calibrateGyroOnly(&h.calib);
  if (consumeTimeout(h.bus) || !probe(h.bus, h.imu.getImuAddress()) ||
      h.imu.initImu(h.calib) != 0) {
    h.online = false;
    h.settling = false;
    h.retryAtMs = millis() + RECONNECT_RETRY_MS;
    Serial.printf("[HANDLE%u] Calibration failed; retrying connection.\n", unsigned(index + 1));
    return false;
  }

  h.orientation.begin(h.hasMag, Vec3(0, 0, 1), Vec3(-1, 0, 0), h.hasMag ? 0.4f : 0.2f);
  h.refLocked = false;
  startSettling(h, FILTER_SETTLE_MS);
  Serial.printf("[HANDLE%u] Calibration complete; filter settling.\n", unsigned(index + 1));
  return true;
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
    ads.setDataRate(RATE_ADS1115_860SPS);
    pedalConnected[0] = false;
    pedalConnected[1] = false;
    pedalFilters[0].reset();
    pedalFilters[1].reset();
    nextPedalProbeMs = millis() + PEDAL_PROBE_INTERVAL_MS;
    nextPedalProbeChannel = 0;
    pedalAdcState = PedalAdcState::Normal0Waiting;
    if (!startPedalConversion(0)) return;
    Serial.println("[PEDAL] ADS1115 ready: A0=pedal1, A1=pedal2.");
  }

  int16_t raw = 0;
  switch (pedalAdcState) {
    case PedalAdcState::Normal0Waiting:
      if (!readCompletedPedalConversion(raw)) return;
      pedalRaw[0] = raw;
      ++pedalSamples[0];
      pedals[0] = pedalFilters[0].update(
        constrain(ads.computeVolts(raw) / 3.3f, 0.0f, 1.0f), micros());
      if (!startPedalConversion(1)) return;
      pedalAdcState = PedalAdcState::Normal1Waiting;
      return;

    case PedalAdcState::Normal1Waiting:
      if (!readCompletedPedalConversion(raw)) return;
      pedalRaw[1] = raw;
      ++pedalSamples[1];
      pedals[1] = pedalFilters[1].update(
        constrain(ads.computeVolts(raw) / 3.3f, 0.0f, 1.0f), micros());
      if (due(millis(), nextPedalProbeMs)) {
        digitalWrite(PEDAL_PROBE_PIN, HIGH);
        pinMode(PEDAL_PROBE_PIN, OUTPUT);
        pedalProbeDeadlineMs = millis() + PEDAL_PROBE_SETTLE_MS;
        pedalAdcState = PedalAdcState::ProbeHighSettling;
      } else {
        if (!startPedalConversion(0)) return;
        pedalAdcState = PedalAdcState::Normal0Waiting;
      }
      return;

    case PedalAdcState::ProbeHighSettling:
      if (!due(millis(), pedalProbeDeadlineMs)) return;
      if (!startPedalConversion(nextPedalProbeChannel)) return;
      pedalAdcState = PedalAdcState::ProbeHighWaiting;
      return;

    case PedalAdcState::ProbeHighWaiting:
      if (!readCompletedPedalConversion(pedalProbeHighRaw)) return;
      digitalWrite(PEDAL_PROBE_PIN, LOW);
      pedalProbeDeadlineMs = millis() + PEDAL_PROBE_SETTLE_MS;
      pedalAdcState = PedalAdcState::ProbeLowSettling;
      return;

    case PedalAdcState::ProbeLowSettling:
      if (!due(millis(), pedalProbeDeadlineMs)) return;
      if (!startPedalConversion(nextPedalProbeChannel)) return;
      pedalAdcState = PedalAdcState::ProbeLowWaiting;
      return;

    case PedalAdcState::ProbeLowWaiting:
      if (!readCompletedPedalConversion(raw)) return;
      pinMode(PEDAL_PROBE_PIN, INPUT);
      {
        const bool wasConnected = pedalConnected[nextPedalProbeChannel];
        pedalProbeDelta[nextPedalProbeChannel] =
          fabsf(ads.computeVolts(pedalProbeHighRaw) - ads.computeVolts(raw));
        const bool isConnected =
          pedalProbeDelta[nextPedalProbeChannel] < PEDAL_OPEN_DELTA_VOLTS;
        pedalConnected[nextPedalProbeChannel] = isConnected;
        if (wasConnected != isConnected) {
          pedalFilters[nextPedalProbeChannel].reset();
        }
      }
      nextPedalProbeChannel ^= 1;
      nextPedalProbeMs = millis() + PEDAL_PROBE_INTERVAL_MS;
      if (!startPedalConversion(0)) return;
      pedalAdcState = PedalAdcState::Normal0Waiting;
      return;
  }
}

static void outputLatestInputs() {
  const uint32_t nowUs = micros();
  if (!due(nowUs, nextOutputUs)) return;

  nextOutputUs += OUTPUT_PERIOD_US;
  // Skip missed slots instead of emitting a burst after an exceptional delay.
  if (due(nowUs, nextOutputUs)) {
    nextOutputUs = nowUs + OUTPUT_PERIOD_US;
  }

  const float missing = NAN;
  Serial.printf("%.2f,%.2f,%.2f,%.2f\n",
    adsOnline && pedalConnected[0] && pedalFilters[0].hasValue() ? pedals[0] : missing,
    handles[0].online && !handles[0].settling ? handles[0].angleDeg : missing,
    adsOnline && pedalConnected[1] && pedalFilters[1].hasValue() ? pedals[1] : missing,
    handles[1].online && !handles[1].settling ? handles[1].angleDeg : missing);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  pinMode(PEDAL_PROBE_PIN, INPUT);
  Serial.println("[INPUT] CSV: pedal1,handle1_deg,pedal2,handle2_deg");
  Serial.println("[INPUT] Send 'c' to calibrate both handles.");
  Serial.println("[INPUT] Send 'd' to toggle register/raw diagnostics (1 Hz).");
  for (size_t i = 0; i < 2; ++i) {
    if (!initializeHandle(handles[i], i)) {
      handles[i].retryAtMs = millis() + RECONNECT_RETRY_MS;
      if (handles[i].imu.getImuAddress() != 0) {
        Serial.printf("[HANDLE%u] Not ready at 0x%02X: WHO_AM_I=0x%02X (MPU6050 requires 0x68); retrying.\n",
          unsigned(i + 1), handles[i].address, handles[i].imu.getWhoAmI());
      } else {
        Serial.printf("[HANDLE%u] No I2C ACK at 0x%02X; retrying.\n",
          unsigned(i + 1), handles[i].address);
      }
    }
  }
  for (auto& h : handles) {
    if (h.online) h.settleStartedMs = millis();
  }
  nextOutputUs = micros() + OUTPUT_PERIOD_US;
}

void loop() {
  while (Serial.available()) {
    const char c = char(Serial.read());
    if (c == 'd' || c == 'D') {
      diagnosticsEnabled = !diagnosticsEnabled;
      nextDiagnosticsMs = millis();
      Serial.printf("[DIAG] %s\n", diagnosticsEnabled ? "enabled" : "disabled");
      continue;
    }
    if (c == 'c' || c == 'C') {
      Serial.println("[INPUT] Starting handle calibration.");
      for (size_t i = 0; i < 2; ++i) calibrateHandle(handles[i], i);
      // Start the settling interval together after all blocking calibrations.
      for (auto& h : handles) {
        if (h.online && h.settling) h.settleStartedMs = millis();
      }
      continue;
    }
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
  outputLatestInputs();
  if (diagnosticsEnabled && due(millis(), nextDiagnosticsMs)) {
    printDiagnostics();
    nextDiagnosticsMs = millis() + 1000;
  }
  delay(1); // Yield without making the output period depend on loop duration.
}
