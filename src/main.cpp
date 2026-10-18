#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <Adafruit_ADS1X15.h>
#include "imu_manager.hpp"
#include "orientation.hpp"
#include "rgp_protocol.hpp"
#include "pedal_filter.hpp"

// All inputs share one bus; AD0 selects each player's IMU address.
constexpr uint8_t I2C_SDA = 21;
constexpr uint8_t I2C_SCL = 22;
constexpr uint8_t HANDLE1_ADDRESS = 0x68;
constexpr uint8_t HANDLE2_ADDRESS = 0x69;
constexpr uint8_t ADS_ADDRESS = 0x48;
// GPIO27 reaches A0 and A1 through two separate 100 kOhm resistors.
// A low-impedance pedal holds its voltage; an open input follows the probe.
constexpr uint8_t PEDAL_PROBE_PIN = 27;
constexpr uint32_t PEDAL_PROBE_INTERVAL_MS = 100;
constexpr uint16_t PEDAL_PROBE_SETTLE_MS = 20;
constexpr float PEDAL_OPEN_DELTA_VOLTS = 0.8f;
constexpr uint32_t FILTER_SETTLE_MS = 5000;
constexpr uint32_t RECONNECT_SETTLE_MS = 1200;
constexpr uint32_t RECONNECT_RETRY_MS = 1000;
constexpr float RAD_TO_DEG_F = 57.2957795f;
// Both IMUs are mounted 180 degrees around Z from the original orientation.
// Move the gravity zero from -X to +X without changing the rotation direction.
const Vec3 HANDLE_ZERO_GRAVITY(1, 0, 0);

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
static bool anyHandleOnline() {
  for (const auto& h : handles) if (h.online) return true;
  return false;
}
bool adsOnline = false;
uint32_t adsRetryAtMs = 0;
// One accelerator per player. A2/A3 are unused and never sampled or probed.
constexpr size_t PEDAL_COUNT = 2;
constexpr const char* PEDAL_NAMES[] = {"P1_ACCEL", "P2_ACCEL"};
struct PedalCalibration { float minimum; float maximum; };
// Each channel has its own calibration. Replace these nominal endpoints with measured values.
constexpr PedalCalibration PEDAL_CALIBRATION[] = {{0, 3.3f}, {0, 3.3f}};
float pedals[PEDAL_COUNT] = {};
PedalFilter pedalFilters[PEDAL_COUNT];
int16_t pedalRaw[PEDAL_COUNT] = {};
uint32_t pedalSamples[PEDAL_COUNT] = {};
float pedalProbeDelta[PEDAL_COUNT] = {NAN, NAN};
uint32_t nextDiagnosticsMs = 0;
bool pedalConnected[PEDAL_COUNT] = {};
bool pedalSampled[PEDAL_COUNT] = {};
bool pedalChecked[PEDAL_COUNT] = {};
bool pedalReportedLost[PEDAL_COUNT] = {};
uint8_t normalPedalChannel = 0;
uint32_t pedalConversionStartedMs = 0;
uint32_t nextPedalProbeMs = 0;
uint8_t nextPedalProbeChannel = 0;
int16_t pedalProbeHighRaw = 0;

enum class PedalAdcState : uint8_t {
  NormalWaiting,
  ProbeHighSettling,
  ProbeHighWaiting,
  ProbeLowSettling,
  ProbeLowWaiting
};

PedalAdcState pedalAdcState = PedalAdcState::NormalWaiting;
uint32_t pedalProbeDeadlineMs = 0;
uint32_t nextOutputUs = 0;

class FirmwareDevice : public rgp::Device {
public:
  void write(const std::string& wire) override { Serial.write(reinterpret_cast<const uint8_t*>(wire.data()), wire.size()); }
  rgp::Status status() const override;
  bool zeroAngles() override;
  void setBeta(float value) override;
  void calibrate(uint32_t origin, const std::string& target) override;
  void scan(uint32_t origin) override;
};
FirmwareDevice device;
rgp::Protocol protocol(device);

struct CalibrationJob {
  bool active = false;
  bool targets[2] = {};
  uint32_t origin = 0;
  std::string target;
  size_t handle = 0;
  uint16_t samples = 0;
  double sum[3] = {};
  uint32_t nextSampleMs = 0;
  bool prepared = false;
} calibration;
struct ScanJob {
  bool active = false;
  uint32_t origin = 0;
  uint8_t address = 0x08;
  uint8_t count = 0;
  bool adsFound = false;
  bool handleFound[2] = {};
} scanJob;
bool angleWasLocked = false;
bool connectionWasLost = false;
bool busFailureReported = false;
static void restartPedalSampling();
static void serviceSerial() {
  // Limit an RX pass while guaranteeing service between potentially slow IMU initializations.
  for (size_t count = 0; count < 256 && Serial.available(); ++count)
    protocol.receive(uint8_t(Serial.read()), millis());
  protocol.tick(millis());
}

static std::string integer(uint32_t n) { return std::to_string(n); }
static std::string addressText(uint8_t address) {
  char text[5];
  snprintf(text, sizeof(text), "0x%02X", address);
  return text;
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
  static const uint16_t mux[] = {ADS1X15_REG_CONFIG_MUX_SINGLE_0,
    ADS1X15_REG_CONFIG_MUX_SINGLE_1};
  return mux[channel];
}
static bool validPedalCalibration() {
  for (const auto& c : PEDAL_CALIBRATION) {
    float ignored;
    if (!rgp::normalizePedal(0, c.minimum, c.maximum, ignored)) return false;
  }
  return true;
}
static void markAdsOffline() {
  pinMode(PEDAL_PROBE_PIN, INPUT);
  adsOnline = false;
  for (size_t i = 0; i < PEDAL_COUNT; ++i) {
    pedalFilters[i].reset();
    pedalConnected[i] = false;
    pedalSampled[i] = false;
    pedalChecked[i] = false;
  }
  adsRetryAtMs = millis() + RECONNECT_RETRY_MS;
}
static void markHandleOffline(HandleInput& h) {
  if (h.online) protocol.event("IMU_LOST," + addressText(h.address), millis());
  if (h.settling) h.orientation.changeBeta(h.restoreBeta);
  h.online = false;
  h.settling = false;
  h.retryAtMs = millis() + RECONNECT_RETRY_MS;
}
static bool startPedalConversion(uint8_t channel) {
  if (!probe(Wire, ADS_ADDRESS)) { markAdsOffline(); return false; }
  ads.startADCReading(pedalMux(channel), false);
  pedalConversionStartedMs = millis();
  if (!consumeTimeout(Wire)) return true;
  markAdsOffline();
  return false;
}
static bool readCompletedPedalConversion(int16_t& raw) {
  if (!probe(Wire, ADS_ADDRESS)) { markAdsOffline(); return false; }
  const bool complete = ads.conversionComplete();
  if (consumeTimeout(Wire) || uint32_t(millis() - pedalConversionStartedMs) >= 100) {
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
  h.restoreBeta = protocol.beta();
  h.orientation.changeBeta(h.hasMag ? 1.0f : h.restoreBeta);
  h.settleStartedMs = millis();
  h.settleDurationMs = duration;
  h.settling = true;
}
static bool initializeHandle(HandleInput& h, size_t index) {
  consumeTimeout(h.bus);
  // ImuManager selects the driver by WHO_AM_I; P1/P2 may use different models.
  if (!h.imu.selectAtAddress(h.address) || h.imu.initImu(h.calib) != 0) return false;
  h.hasMag = h.imu.hasMagnetometer();
  protocol.send("SYS", "DEVICE,IMU," + addressText(h.address) + "," + addressText(h.imu.getWhoAmI()), millis());
  protocol.event("IMU_FOUND", millis());
  if (!h.configured) {
    h.orientation.begin(h.hasMag, Vec3(0, 0, 1), HANDLE_ZERO_GRAVITY, protocol.beta());
    h.configured = true;
    startSettling(h, FILTER_SETTLE_MS);
  } else {
    h.orientation.resetFilterForResync();
    startSettling(h, RECONNECT_SETTLE_MS);
  }
  h.online = true;
  (void)index;
  return true;
}
static void updateHandle(HandleInput& h, size_t index) {
  if (!h.online) {
    if (due(millis(), h.retryAtMs) && !initializeHandle(h, index))
      h.retryAtMs = millis() + RECONNECT_RETRY_MS;
    return;
  }
  if (!probe(h.bus, h.address) || consumeTimeout(h.bus)) { markHandleOffline(h); return; }
  if (calibration.active && calibration.handle == index) return;
  h.imu.update();
  h.imu.getAccel(&h.accel);
  h.imu.getGyro(&h.gyro);
  if (consumeTimeout(h.bus) || !isfinite(h.accel.accelX) || !isfinite(h.accel.accelY) ||
      !isfinite(h.accel.accelZ) || !isfinite(h.gyro.gyroX) || !isfinite(h.gyro.gyroY) ||
      !isfinite(h.gyro.gyroZ)) { markHandleOffline(h); return; }
  h.orientation.updateIMU(h.gyro.gyroX, h.gyro.gyroY, h.gyro.gyroZ,
    h.accel.accelX, h.accel.accelY, h.accel.accelZ);
  if (h.settling) {
    if (uint32_t(millis() - h.settleStartedMs) < h.settleDurationMs) return;
    h.orientation.changeBeta(h.restoreBeta);
    if (!h.orientation.canLockShaftReference()) return;
    if (!h.refLocked) {
      h.orientation.restartAngleTracking();
      h.refLocked = true;
    }
    h.settling = false;
  }
  h.angleDeg = -h.orientation.shaftAngleRad() * RAD_TO_DEG_F;
}
static void updatePedals() {
  if (!adsOnline) {
    if (!due(millis(), adsRetryAtMs)) return;
    consumeTimeout(Wire);
    adsOnline = ads.begin(ADS_ADDRESS, &Wire);
    if (!adsOnline) { adsRetryAtMs = millis() + RECONNECT_RETRY_MS; return; }
    ads.setGain(GAIN_ONE);
    ads.setDataRate(RATE_ADS1115_860SPS);
    for (size_t i = 0; i < PEDAL_COUNT; ++i) {
      pedalFilters[i].reset();
      pedalConnected[i] = false;
      pedalSampled[i] = false;
      pedalChecked[i] = false;
    }
    normalPedalChannel = 0;
    nextPedalProbeMs = millis() + PEDAL_PROBE_INTERVAL_MS;
    nextPedalProbeChannel = 0;
    pedalAdcState = PedalAdcState::NormalWaiting;
    if (!startPedalConversion(0)) return;
    protocol.send("SYS", "DEVICE,ADS1115," + addressText(ADS_ADDRESS), millis());
    protocol.event("ADS_READY", millis());
  }
  int16_t raw = 0;
  switch (pedalAdcState) {
    case PedalAdcState::NormalWaiting: {
      if (!readCompletedPedalConversion(raw)) return;
      const auto& c = PEDAL_CALIBRATION[normalPedalChannel];
      pedalRaw[normalPedalChannel] = raw;
      ++pedalSamples[normalPedalChannel];
      float normalized = 0;
      pedalSampled[normalPedalChannel] = rgp::normalizePedal(ads.computeVolts(raw),
        c.minimum, c.maximum, normalized);
      if (pedalSampled[normalPedalChannel])
        pedals[normalPedalChannel] = pedalFilters[normalPedalChannel].update(normalized, micros());
      else pedalFilters[normalPedalChannel].reset();
      if (!pedalSampled[normalPedalChannel] && !pedalReportedLost[normalPedalChannel]) {
        protocol.error(std::string("ERROR,ADS1115,READ_FAILED,") + PEDAL_NAMES[normalPedalChannel], millis());
        pedalReportedLost[normalPedalChannel] = true;
      }
      normalPedalChannel = (normalPedalChannel + 1) % PEDAL_COUNT;
      if (normalPedalChannel == 0 && due(millis(), nextPedalProbeMs)) {
        digitalWrite(PEDAL_PROBE_PIN, HIGH);
        pinMode(PEDAL_PROBE_PIN, OUTPUT);
        pedalProbeDeadlineMs = millis() + PEDAL_PROBE_SETTLE_MS;
        pedalAdcState = PedalAdcState::ProbeHighSettling;
      } else startPedalConversion(normalPedalChannel);
      return;
    }
    case PedalAdcState::ProbeHighSettling:
      if (!due(millis(), pedalProbeDeadlineMs)) return;
      if (startPedalConversion(nextPedalProbeChannel)) pedalAdcState = PedalAdcState::ProbeHighWaiting;
      return;
    case PedalAdcState::ProbeHighWaiting:
      if (!readCompletedPedalConversion(pedalProbeHighRaw)) return;
      digitalWrite(PEDAL_PROBE_PIN, LOW);
      pedalProbeDeadlineMs = millis() + PEDAL_PROBE_SETTLE_MS;
      pedalAdcState = PedalAdcState::ProbeLowSettling;
      return;
    case PedalAdcState::ProbeLowSettling:
      if (!due(millis(), pedalProbeDeadlineMs)) return;
      if (startPedalConversion(nextPedalProbeChannel)) pedalAdcState = PedalAdcState::ProbeLowWaiting;
      return;
    case PedalAdcState::ProbeLowWaiting: {
      if (!readCompletedPedalConversion(raw)) return;
      pinMode(PEDAL_PROBE_PIN, INPUT);
      const size_t i = nextPedalProbeChannel;
      pedalProbeDelta[i] = fabsf(ads.computeVolts(pedalProbeHighRaw) - ads.computeVolts(raw));
      const bool connected = pedalProbeDelta[i] < PEDAL_OPEN_DELTA_VOLTS;
      if (!connected || !pedalConnected[i]) { pedalSampled[i] = false; pedalFilters[i].reset(); }
      pedalConnected[i] = connected;
      pedalChecked[i] = true;
      if (!connected && !pedalReportedLost[i]) {
        protocol.error(std::string("ERROR,ADS1115,READ_FAILED,") + PEDAL_NAMES[i], millis());
        pedalReportedLost[i] = true;
      }
      if (connected) pedalReportedLost[i] = false;
      nextPedalProbeChannel = (nextPedalProbeChannel + 1) % PEDAL_COUNT;
      nextPedalProbeMs = millis() + PEDAL_PROBE_INTERVAL_MS;
      normalPedalChannel = 0;
      if (startPedalConversion(0)) pedalAdcState = PedalAdcState::NormalWaiting;
      return;
    }
  }
}

rgp::Status FirmwareDevice::status() const {
  rgp::Status s;
  s.ads = adsOnline;
  for (size_t i = 0; i < PEDAL_COUNT; ++i) s.ads = s.ads && pedalConnected[i] && pedalSampled[i];
  s.imu = anyHandleOnline();
  s.angle = s.imu;
  s.calibrated = s.imu && validPedalCalibration() && !calibration.active;
  for (const auto& h : handles) {
    if (!h.online) continue;
    s.angle = s.angle && !h.settling && h.refLocked && isfinite(h.angleDeg) && fabsf(h.angleDeg) <= 3600;
    s.calibrated = s.calibrated && h.calib.valid;
  }
  return s;
}
bool FirmwareDevice::zeroAngles() {
  if (!anyHandleOnline()) return false;
  for (auto& h : handles)
    if (h.online && (h.settling || !h.orientation.hasAbsoluteShaftReference())) return false;
  for (auto& h : handles) {
    if (!h.online) continue;
    h.orientation.restartAngleTracking();
    h.refLocked = true;
    h.angleDeg = -h.orientation.shaftAngleRad() * RAD_TO_DEG_F;
  }
  angleWasLocked = true; // Protocol emits the success event after its ACK.
  return true;
}
void FirmwareDevice::setBeta(float value) {
  for (auto& h : handles) {
    if (h.settling) h.restoreBeta = value;
    else h.orientation.changeBeta(value);
  }
}
void FirmwareDevice::calibrate(uint32_t origin, const std::string& target) {
  calibration = CalibrationJob();
  calibration.active = true;
  // Freeze the target set: an absent slot is optional, but losing a target
  // during calibration must fail rather than silently claiming success.
  for (size_t i = 0; i < 2; ++i) calibration.targets[i] = handles[i].online;
  while (calibration.handle < 2 && !calibration.targets[calibration.handle]) ++calibration.handle;
  calibration.origin = origin;
  calibration.target = target;
  calibration.nextSampleMs = millis() + 3000;
  pinMode(PEDAL_PROBE_PIN, INPUT);
  restartPedalSampling();
  protocol.setState(rgp::State::Calibrating, millis());
  protocol.event("CAL_START," + integer(origin) + "," + target, millis());
  if (target == "ALL") protocol.event("CAL_MAG_SKIPPED," + integer(origin), millis());
  protocol.event("CAL_GYRO_START," + integer(origin), millis());
}
static void failCalibration() {
  // The active IMU may be temporarily running with zero bias. Reconnect using its
  // last committed calibration before allowing normal inputs again.
  if (calibration.prepared && calibration.handle < 2 && handles[calibration.handle].online)
    markHandleOffline(handles[calibration.handle]);
  protocol.error("ERROR,CALIBRATION,READ_FAILED,GYRO,REQ," + integer(calibration.origin), millis());
  calibration.active = false;
  protocol.finishOperation();
  // The connection monitor chooses the state from the remaining online inputs.
}
static void updateCalibration() {
  if (!calibration.active) return;
  if (calibration.handle >= 2) { failCalibration(); return; }
  for (size_t i = 0; i < 2; ++i)
    if (calibration.targets[i] && !handles[i].online) { failCalibration(); return; }
  if (!due(millis(), calibration.nextSampleMs)) return;
  auto& h = handles[calibration.handle];
  if (!calibration.prepared) {
    // Measure raw gyro readings rather than applying the previous bias twice.
    calData uncorrected = {};
    if (h.imu.initImu(uncorrected) != 0) { markHandleOffline(h); failCalibration(); return; }
    calibration.prepared = true;
  }
  h.imu.update();
  GyroData g = {};
  h.imu.getGyro(&g);
  if (!probe(h.bus, h.address) || consumeTimeout(h.bus) ||
      !isfinite(g.gyroX) || !isfinite(g.gyroY) || !isfinite(g.gyroZ)) {
    markHandleOffline(h);
    failCalibration();
    return;
  }
  calibration.sum[0] += g.gyroX;
  calibration.sum[1] += g.gyroY;
  calibration.sum[2] += g.gyroZ;
  calibration.nextSampleMs = millis() + 5;
  if (++calibration.samples < 400) return;
  calData result = {};
  result.valid = true;
  for (size_t i = 0; i < 3; ++i) result.gyroBias[i] = calibration.sum[i] / calibration.samples;
  if (h.imu.initImu(result) != 0) { markHandleOffline(h); failCalibration(); return; }
  h.calib = result;
  h.orientation.begin(false, Vec3(0, 0, 1), HANDLE_ZERO_GRAVITY, protocol.beta());
  h.refLocked = false;
  startSettling(h, FILTER_SETTLE_MS);
  ++calibration.handle;
  while (calibration.handle < 2 && !calibration.targets[calibration.handle]) ++calibration.handle;
  if (calibration.handle < 2) {
    calibration.samples = 0;
    for (auto& sum : calibration.sum) sum = 0;
    calibration.prepared = false;
    calibration.nextSampleMs = millis() + 3000;
    return;
  }
  for (size_t i = 0; i < 2; ++i)
    if (calibration.targets[i]) handles[i].settleStartedMs = millis();
  protocol.event("CAL_GYRO_DONE," + integer(calibration.origin), millis());
  protocol.event("CAL_DONE," + integer(calibration.origin) + "," + calibration.target, millis());
  calibration.active = false;
  protocol.finishOperation();
  protocol.setState(rgp::State::WaitAngle, millis());
}
void FirmwareDevice::scan(uint32_t origin) {
  scanJob = ScanJob();
  scanJob.active = true;
  scanJob.origin = origin;
  pinMode(PEDAL_PROBE_PIN, INPUT);
  protocol.event("I2C_SCAN_START," + integer(origin), millis());
}
static void restartPedalSampling() {
  if (!adsOnline) return;
  // A scan may interrupt a probe conversion. Discard it and obtain a normal sample.
  normalPedalChannel = 0;
  pedalAdcState = PedalAdcState::NormalWaiting;
  startPedalConversion(0);
}
static void updateScan() {
  if (!scanJob.active) return;
  Wire.beginTransmission(scanJob.address);
  const uint8_t result = Wire.endTransmission(true);
  if (consumeTimeout(Wire) || (result != 0 && result != 2 && result != 3)) {
    protocol.error("ERROR,I2C,READ_FAILED,I2C_SCAN,REQ," + integer(scanJob.origin), millis());
    scanJob.active = false;
    protocol.finishOperation();
    restartPedalSampling();
    if (protocol.state() != rgp::State::Fatal) {
      markAdsOffline();
    }
    return;
  }
  if (result == 0) {
    ++scanJob.count;
    protocol.send("SYS", "I2C_DEVICE," + integer(scanJob.origin) + "," + addressText(scanJob.address), millis());
    if (scanJob.address == ADS_ADDRESS) scanJob.adsFound = true;
    for (size_t i = 0; i < 2; ++i) if (scanJob.address == handles[i].address) scanJob.handleFound[i] = true;
  }
  if (++scanJob.address <= 0x77) return;
  protocol.event("I2C_SCAN_END," + integer(scanJob.origin) + "," + integer(scanJob.count), millis());
  scanJob.active = false;
  protocol.finishOperation();
  if (protocol.state() != rgp::State::Fatal) {
    if (!scanJob.adsFound) markAdsOffline();
    for (size_t i = 0; i < 2; ++i) if (!scanJob.handleFound[i]) markHandleOffline(handles[i]);
  }
  restartPedalSampling();
}
static void updateState() {
  if (protocol.state() == rgp::State::Fatal || scanJob.active) return;
  const uint32_t now = millis();
  const bool busOnline = adsOnline && anyHandleOnline();
  if (!busOnline && !busFailureReported) {
    protocol.error("ERROR,I2C,CONNECTION_LOST", now);
    protocol.event("I2C_LOST", now);
    busFailureReported = true;
    connectionWasLost = true;
  } else if (busOnline && busFailureReported) {
    protocol.event("I2C_RESTORED", now);
    busFailureReported = false;
  }
  const auto s = device.status();
  if (s.angle && !angleWasLocked) protocol.event("ANGLE_LOCKED", now);
  if (!s.angle && angleWasLocked) {
    protocol.error("ERROR,HANDLE,ANGLE_REFERENCE_INVALID", now);
    protocol.event("ANGLE_UNLOCKED", now);
  }
  angleWasLocked = s.angle;
  if (calibration.active) return;
  bool knownDisconnected = false;
  for (size_t i = 0; i < PEDAL_COUNT; ++i)
    knownDisconnected = knownDisconnected || (pedalChecked[i] && (!pedalConnected[i] || !pedalSampled[i]));
  if (!busOnline || knownDisconnected) {
    connectionWasLost = true;
    protocol.setState(rgp::State::Recovering, now);
  } else if (s.ads && s.imu && s.angle && s.calibrated) {
    protocol.setState(rgp::State::Ready, now);
    connectionWasLost = false;
  } else if (!connectionWasLost || s.ads) protocol.setState(rgp::State::WaitAngle, now);
}
static void outputLatestInputs() {
  const uint32_t nowUs = micros();
  const uint32_t period = protocol.telemetryPeriodUs();
  static uint32_t scheduledPeriod = 10000;
  if (period != scheduledPeriod) {
    scheduledPeriod = period;
    nextOutputUs = nowUs + period;
  }
  if (!due(nowUs, nextOutputUs)) return;
  nextOutputUs += period;
  if (due(nowUs, nextOutputUs)) nextOutputUs = nowUs + period;
  // Preserve player/address mapping and the six RGP/1 fields. An absent handle
  // has a neutral value; never forward its last angle or mirror the other IMU.
  const rgp::Inputs snapshot = {{handles[0].online ? handles[0].angleDeg : 0.0f,
    handles[1].online ? handles[1].angleDeg : 0.0f},
    {pedals[0], pedals[1]}, {0.0f, 0.0f}};
  protocol.telemetry(snapshot, millis());
}
// Register diagnostics use RGP DBG frames; never interleave legacy CSV/raw logs.
static bool readRegisters(uint8_t address, uint8_t reg, uint8_t* data, uint8_t count) {
  Wire.beginTransmission(address);
  if (Wire.endTransmission(true) != 0) return false;
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}
static void diagnostic(const char* subsystem, const char* format, ...) {
  char text[256];
  va_list args; va_start(args, format);
  vsnprintf(text, sizeof(text), format, args); va_end(args);
  const std::string value(text);
  const char* hex = "0123456789ABCDEF";
  // 56 bytes expand to at most 168 encoded bytes, below the RGP frame limit.
  for (size_t offset = 0; offset < value.size(); offset += 56) {
    std::string encoded;
    for (size_t i = offset; i < value.size() && i < offset + 56; ++i) {
      const unsigned char c = value[i];
      if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
          (c >= '0' && c <= '9') || c == '-' || c == '.' || c == '_' || c == '~') encoded += c;
      else { encoded += '%'; encoded += hex[c >> 4]; encoded += hex[c & 15]; }
    }
    protocol.send("DBG", std::string(subsystem) + "," + encoded, millis());
  }
}
static void printDiagnostics() {
  for (size_t i = 0; i < 2; ++i) {
    auto& h = handles[i];
    uint8_t who = 0, power = 0, data[14] = {};
    const bool whoOk = readRegisters(h.address, 0x75, &who, 1);
    const bool powerOk = readRegisters(h.address, 0x6B, &power, 1);
    const bool rawOk = readRegisters(h.address, 0x3B, data, sizeof(data));
    diagnostic("IMU", "P%u addr=%s WHO_AM_I=%s read=%u power=%u/%u", unsigned(i + 1),
      addressText(h.address).c_str(), addressText(who).c_str(), whoOk, power, powerOk);
    diagnostic("IMU", "P%u online=%u settling=%u angle=%.2f raw_read=%u", unsigned(i + 1),
      h.online, h.settling, h.angleDeg, rawOk);
    if (rawOk) {
      int16_t v[7];
      for (uint8_t j = 0; j < 7; ++j) v[j] = int16_t((uint16_t(data[2*j]) << 8) | data[2*j+1]);
      diagnostic("IMU", "P%u raw=%d,%d,%d,%d,%d,%d,%d", unsigned(i + 1), v[0],v[1],v[2],v[3],v[4],v[5],v[6]);
      diagnostic("IMU", "P%u accel_g=%.4f,%.4f,%.4f", unsigned(i + 1), h.accel.accelX,h.accel.accelY,h.accel.accelZ);
      diagnostic("IMU", "P%u gyro_dps=%.3f,%.3f,%.3f", unsigned(i + 1), h.gyro.gyroX,h.gyro.gyroY,h.gyro.gyroZ);
    }
  }
  uint8_t config[2] = {}, conversion[2] = {};
  const bool configOk = readRegisters(ADS_ADDRESS, 0x01, config, 2);
  const bool conversionOk = readRegisters(ADS_ADDRESS, 0x00, conversion, 2);
  diagnostic("ADS1115", "config_read=%u config=%u raw_read=%u raw=%d", configOk,
    unsigned((uint16_t(config[0]) << 8) | config[1]), conversionOk,
    int16_t((uint16_t(conversion[0]) << 8) | conversion[1]));
  for (size_t i = 0; i < PEDAL_COUNT; ++i)
    diagnostic("ADS1115", "A%u samples=%lu raw=%d volts=%.4f delta=%.4f connected=%u", unsigned(i),
      (unsigned long)pedalSamples[i], pedalRaw[i], ads.computeVolts(pedalRaw[i]), pedalProbeDelta[i], pedalConnected[i]);
}

void setup() {
  Serial.begin(115200);
  protocol.begin();
  protocol.setState(rgp::State::Initializing, millis());
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  pinMode(PEDAL_PROBE_PIN, INPUT);
  for (size_t i = 0; i < 2; ++i) {
    // Gyro calibration remains explicit. Do not claim valid calibration before measurement.
    if (!initializeHandle(handles[i], i)) {
      handles[i].retryAtMs = millis() + RECONNECT_RETRY_MS;
      // A missing address is an optional player slot, not an init failure.
      if (probe(Wire, handles[i].address)) protocol.error("ERROR,IMU,INIT_FAILED", millis());
    }
    serviceSerial();
  }
  // Validate independent pedal parameters; gyro calibration is requested explicitly by CMD.
  protocol.setState(rgp::State::Calibrating, millis());
  if (!validPedalCalibration()) {
    protocol.error("FATAL,CALIBRATION,INIT_FAILED", millis());
    protocol.setState(rgp::State::Fatal, millis());
  } else protocol.setState(rgp::State::WaitAngle, millis());
  for (auto& h : handles) if (h.online) h.settleStartedMs = millis();
  nextOutputUs = micros() + protocol.telemetryPeriodUs();
}
void loop() {
  serviceSerial();
  if (scanJob.active) updateScan();
  else if (protocol.state() != rgp::State::Fatal) {
    updatePedals();
    for (size_t i = 0; i < 2; ++i) {
      updateHandle(handles[i], i);
      serviceSerial();
      if (scanJob.active) break;
    }
    if (!scanJob.active) updateCalibration();
  }
  updateState();
  outputLatestInputs();
  if (protocol.diagnosticsEnabled() && !scanJob.active && !calibration.active && due(millis(), nextDiagnosticsMs)) {
    printDiagnostics();
    nextDiagnosticsMs = millis() + 1000;
  }
  delay(1);
}
