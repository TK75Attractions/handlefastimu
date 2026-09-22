#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_ADS1X15.h>
#include "imu_manager.hpp"
#include "orientation.hpp"

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
    ads.setDataRate(RATE_ADS1115_860SPS);
    pedalConnected[0] = false;
    pedalConnected[1] = false;
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
      pedals[0] = constrain(ads.computeVolts(raw) / 3.3f, 0.0f, 1.0f);
      if (!startPedalConversion(1)) return;
      pedalAdcState = PedalAdcState::Normal1Waiting;
      return;

    case PedalAdcState::Normal1Waiting:
      if (!readCompletedPedalConversion(raw)) return;
      pedals[1] = constrain(ads.computeVolts(raw) / 3.3f, 0.0f, 1.0f);
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
      pedalConnected[nextPedalProbeChannel] =
        fabsf(ads.computeVolts(pedalProbeHighRaw) - ads.computeVolts(raw))
          < PEDAL_OPEN_DELTA_VOLTS;
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
    adsOnline && pedalConnected[0] ? pedals[0] : missing,
    handles[0].online && !handles[0].settling ? handles[0].angleDeg : missing,
    adsOnline && pedalConnected[1] ? pedals[1] : missing,
    handles[1].online && !handles[1].settling ? handles[1].angleDeg : missing);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  pinMode(PEDAL_PROBE_PIN, INPUT);
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
  nextOutputUs = micros() + OUTPUT_PERIOD_US;
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
  outputLatestInputs();
  delay(1); // Yield without making the output period depend on loop duration.
}
