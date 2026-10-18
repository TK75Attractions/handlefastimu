// Exercise the real setup()/loop() and protocol with deterministic hardware doubles.
// Filter math and physical sensor accuracy are outside this simulation.
#include "Arduino.h"
#include "Wire.h"
uint64_t mockTimeUs = 0;
int mockPinMode = INPUT, mockPinLevel = LOW, mockBusError = 0;
bool mockDevices[128] = {};
bool mockPedals[4] = {true, true, false, false};
bool mockConversionStuck = false;
float mockPedalVolts[4] = {2.31f, .825f, .99f, 2.64f};
MockSerial Serial;
TwoWire Wire;
#include "../src/main.cpp"
#include <iostream>
#include <stdexcept>

// Fake only the hardware wrappers and orientation estimates; the application is unchanged.
uint8_t mockImuIdentities[2] = {0x68, 0x68};
#ifndef FIRMWARE_REAL_IMU_MANAGER
ImuManager::ImuManager(TwoWire& bus) : wire(bus), imuMpu6050(bus), imuMpu6500(bus),
  imuMpu9250(bus), imuMpu9255(bus), imuMpu6515(bus), imuMpu6886(bus),
  imuIcm20689(bus), imuIcm20690(bus), imuGeneric(bus) {}
bool ImuManager::selectAtAddress(uint8_t address) { imuAddress = address; return mockDevices[address]; }
uint8_t ImuManager::getWhoAmI() { return mockImuIdentities[imuAddress == 0x69 ? 1 : 0]; }
uint8_t ImuManager::getImuAddress() { return imuAddress; }
int ImuManager::initImu(calData&) { return mockDevices[imuAddress] ? 0 : -1; }
bool ImuManager::hasMagnetometer() { return false; }
void ImuManager::update() {}
void ImuManager::getAccel(AccelData* out) { *out = {-1, 0, 0}; }
void ImuManager::getGyro(GyroData* out) { *out = {.1f, .2f, .3f}; }
#endif
void Orientation::begin(bool, const Vec3&, const Vec3&, float beta) { currentBeta = beta; refLocked = false; latestGravityProjectionUsable = false; }
void Orientation::changeBeta(float beta) { currentBeta = beta; }
float Orientation::getBeta() { return currentBeta; }
void Orientation::resetFilterForResync() { latestGravityProjectionUsable = false; }
void Orientation::updateIMU(float, float, float, float, float, float) { latestGravityProjectionUsable = true; }
void Orientation::restartAngleTracking() { refLocked = true; totalAngleRad = 0; }
bool Orientation::hasAbsoluteShaftReference() { return refLocked && latestGravityProjectionUsable; }
float Orientation::shaftAngleRad() { return totalAngleRad; }
Madgwick::Madgwick() = default;

static int assertions = 0;
static void require(bool condition, const std::string& reason) {
  ++assertions;
  if (!condition) throw std::runtime_error(reason);
}
static std::vector<rgp::Message> output(const std::string& type = "") {
  rgp::Decoder decoder; rgp::Message m;
  std::vector<rgp::Message> messages;
  size_t terminators = 0, decoded = 0;
  for (unsigned char c : Serial.output) {
    if (c == '\n') ++terminators;
    if (decoder.feed(c, millis(), m)) {
      ++decoded;
      if (type.empty() || m.fields[0] == type) messages.push_back(m);
    }
  }
  require(decoded == terminators, "every firmware output line is an RGP frame");
  return messages;
}
static void run(uint32_t milliseconds) { const uint32_t start = millis(); while (uint32_t(millis() - start) < milliseconds) loop(); }
static void command(uint32_t seq, const std::string& name) {
  for (unsigned char c : rgp::frame("CMD," + integer(seq) + ",0," + name)) Serial.input.push_back(c);
  loop();
}
static bool contains(const std::string& type, const std::string& data) {
  for (const auto& m : output(type)) {
    const auto prefix = m.fields[0] + "," + integer(m.seq) + "," + integer(m.time) + ",";
    if (m.body.substr(prefix.size()) == data) return true;
  }
  return false;
}
static void runWithPedalsReady(uint32_t milliseconds) {
  const uint32_t start = millis();
  while (uint32_t(millis() - start) < milliseconds) {
    loop();
    require(protocol.state() == rgp::State::Ready, "pedal movement must not interrupt READY");
  }
}
static void waitForPedalLowProbe(size_t channel) {
  const uint32_t start = millis();
  while (pedalAdcState != PedalAdcState::ProbeLowSettling || nextPedalProbeChannel != channel) {
    require(uint32_t(millis() - start) < 1000, "pedal probe is scheduled within one second");
    runWithPedalsReady(1);
  }
}
static void rapidPedalScenario(size_t channel) {
  const float original = mockPedalVolts[channel];
  mockPedalVolts[channel] = .3f;
  runWithPedalsReady(400);
  Serial.output.clear();
  // Move after the HIGH measurement, before the LOW measurement: the old
  // absolute-difference test reported a disconnected pedal in either direction.
  for (int repeat = 0; repeat < 3; ++repeat) {
    waitForPedalLowProbe(channel);
    mockPedalVolts[channel] = 3.0f;
    runWithPedalsReady(80);
    waitForPedalLowProbe(channel);
    mockPedalVolts[channel] = .3f;
    runWithPedalsReady(80);
  }
  require(output("ERR").empty() && !output("TEL").empty(),
          "rapid presses and releases preserve telemetry without pedal errors");
  mockPedalVolts[channel] = original;
  runWithPedalsReady(400);
}
static void isolatedPedalProbeScenario(size_t channel) {
  const float original = mockPedalVolts[channel];
  mockPedalVolts[channel] = 3.0f;
  runWithPedalsReady(400);
  Serial.output.clear();
  for (int repeat = 0; repeat < 2; ++repeat) {
    waitForPedalLowProbe(channel);
    mockPedalVolts[channel] = .3f;
    const uint32_t start = millis();
    while (pedalAdcState != PedalAdcState::ProbeConfirmHighSettling) {
      require(uint32_t(millis() - start) < 100, "suspect LOW sample starts HIGH confirmation");
      runWithPedalsReady(1);
    }
    // Even a quick release/press that mimics one entire open-circuit probe
    // must not stop operation; a healthy probe clears the suspicion.
    mockPedalVolts[channel] = 3.0f;
    runWithPedalsReady(80);
    require(pedalOpenChecks[channel] == 1, "one suspect probe is retained for confirmation");
    waitForPedalLowProbe(channel);
    runWithPedalsReady(80);
    require(pedalOpenChecks[channel] == 0, "healthy probe resets consecutive failure count");
  }
  require(output("ERR").empty() && !output("TEL").empty(),
          "isolated suspect probes do not produce errors or stop telemetry");
  mockPedalVolts[channel] = original;
  runWithPedalsReady(400);
}
static void singleHandleScenario(size_t player) {
  const size_t absent = 1 - player;
  mockDevices[0x48] = mockDevices[handles[player].address] = true;
  setup(); run(5100);
  require(protocol.state() == rgp::State::WaitAngle && device.status().imu &&
          device.status().angle && !device.status().calibrated, "one IMU reaches WAIT_ANGLE without inventing calibration");
  require(!contains("ERR", "ERROR,IMU,INIT_FAILED") && !contains("EVT", "I2C_LOST"),
          "absent optional IMU does not report an initialization or bus failure");
  command(0, "OPEN_SESSION,0123456789ABCDEF");
  command(1, "CALIBRATE,GYRO"); run(11000);
  require(contains("ACK", "1,CALIBRATE,ACCEPTED,GYRO") && contains("EVT", "CAL_DONE,1,GYRO"),
          "single online IMU calibration is accepted and completes");
  require(protocol.state() == rgp::State::Ready && handles[player].calib.valid &&
          !handles[absent].calib.valid, "single calibrated IMU reaches READY; absent IMU stays uncalibrated");
  handles[player].angleDeg = 37.5f;
  handles[absent].angleDeg = 123.0f;
  Serial.output.clear(); delay(10); outputLatestInputs();
  auto tel = output("TEL");
  require(tel.size() == 1 && tel[0].fields[3 + player * 3] == "37.50" &&
          tel[0].fields[3 + absent * 3] == "0.00", "TEL preserves player mapping and clears absent handle's stale angle");
  require(tel[0].fields[4] == "0.700" && tel[0].fields[7] == "0.250" &&
          tel[0].fields[5] == "0.000" && tel[0].fields[8] == "0.000", "single IMU retains two pedals and neutral brakes");
  command(2, "ANGLE_ZERO");
  require(contains("ACK", "2,ANGLE_ZERO,OK") && handles[player].angleDeg == 0 &&
          handles[absent].angleDeg == 123, "ANGLE_ZERO affects only online handles");
  Serial.output.clear(); command(3, "SCAN_I2C"); run(112);
  require(contains("EVT", "I2C_SCAN_END,3,2"), "scan supports ADS plus a single IMU");
  Serial.output.clear(); run(100);
  require(protocol.state() == rgp::State::Ready && !output("TEL").empty(), "single IMU telemetry resumes after scan");
  mockDevices[handles[player].address] = false; Serial.output.clear(); run(20);
  require(protocol.state() == rgp::State::Recovering && !device.status().imu &&
          !device.status().angle && !device.status().calibrated, "losing the only IMU invalidates all IMU status flags");
  require(contains("EVT", "IMU_LOST," + addressText(handles[player].address)) && contains("EVT", "I2C_LOST"),
          "last IMU disconnect names its address and reports loss");
  require(!device.zeroAngles(), "ANGLE_ZERO fails when no IMU exists");
  Serial.output.clear(); run(1200);
  require(output("TEL").empty(), "zero IMUs never emit TEL");
  mockDevices[handles[player].address] = true; run(2500);
  require(protocol.state() == rgp::State::Ready, "sole calibrated IMU reconnect resumes operation");
  mockDevices[handles[absent].address] = true; run(6500);
  require(protocol.state() == rgp::State::WaitAngle && !device.status().calibrated,
          "adding an uncalibrated second IMU requires calibration");
  command(4, "CALIBRATE,GYRO"); run(16000);
  require(protocol.state() == rgp::State::Ready && handles[0].calib.valid && handles[1].calib.valid,
          "both IMUs can be calibrated after hot plug");
  Serial.output.clear(); command(5, "CALIBRATE,GYRO"); run(3500);
  mockDevices[handles[absent].address] = false; run(10);
  require(!calibration.active && contains("ERR", "ERROR,CALIBRATION,READ_FAILED,GYRO,REQ,5") &&
          !contains("EVT", "CAL_DONE,5,GYRO"), "losing a calibration target fails even if another IMU remains");
}
int main(int argc, char** argv) {
  try {
    const std::string mode = argc > 1 ? argv[1] : "both";
    if (mode == "p1" || mode == "p2") {
      singleHandleScenario(mode == "p1" ? 0 : 1);
      std::cout << "PASS: " << mode << "-only firmware simulation, " << assertions << " assertions.\n";
      return 0;
    }
    if (mode == "none") {
      mockDevices[0x48] = true; setup(); run(6000);
      require(protocol.state() == rgp::State::Recovering && !device.status().imu &&
              !device.status().angle && !device.status().calibrated && output("TEL").empty(),
              "boot without any IMU cannot reach READY or send telemetry");
      command(0, "OPEN_SESSION,0123456789ABCDEF"); command(1, "CALIBRATE,GYRO");
      require(contains("ACK", "1,CALIBRATE,NOT_READY,RECOVERING") && !calibration.active,
              "calibration is rejected with zero IMUs");
      std::cout << "PASS: no-IMU firmware simulation, " << assertions << " assertions.\n";
      return 0;
    }
    if (mode == "no-ads") {
      mockDevices[0x68] = mockDevices[0x69] = true;
      setup(); run(6000);
      require(protocol.state() == rgp::State::WaitAngle && !device.status().ads &&
              !contains("EVT", "I2C_LOST"), "missing optional ADS does not block calibration");
      require(contains("EVT", "PEDAL_DISCONNECTED,P1_ACCEL") &&
              contains("EVT", "PEDAL_DISCONNECTED,P2_ACCEL"), "missing ADS reports both pedals unavailable");
      command(0, "OPEN_SESSION,0123456789ABCDEF"); command(1, "CALIBRATE,GYRO"); run(16000);
      require(protocol.state() == rgp::State::Ready && !device.status().ads,
              "both IMUs can calibrate and reach READY without ADS");
      Serial.output.clear(); run(1000);
      const auto tel = output("TEL");
      require(tel.size() == 100 && tel.back().fields[4] == "0.000" && tel.back().fields[7] == "0.000" &&
              output("EVT").empty() && output("ERR").empty(), "absent ADS sends 100 Hz with neutral pedals and quiet retries");
      command(2, "GET_STATUS");
      require(contains("ACK", "2,GET_STATUS,OK,READY,0,1,1,1"), "READY still reports ADS_OK=0");
      mockDevices[0x48] = true; run(2000);
      require(device.status().ads && contains("EVT", "PEDAL_CONNECTED,P1_ACCEL") &&
              contains("EVT", "PEDAL_CONNECTED,P2_ACCEL"), "ADS hot plug restores confirmed pedals");
      Serial.output.clear(); mockDevices[0x48] = false; run(1000);
      require(protocol.state() == rgp::State::Ready && !device.status().ads && !output("TEL").empty() &&
              contains("EVT", "PEDAL_DISCONNECTED,P1_ACCEL"), "ADS loss keeps IMU TEL running");
      require(output("TEL").back().fields[4] == "0.000" && output("TEL").back().fields[7] == "0.000",
              "ADS loss never forwards stale pedal values");
      std::cout << "PASS: no-ADS firmware simulation, " << assertions << " assertions.\n";
      return 0;
    }
    if (mode == "pedal-open") {
      mockDevices[0x48] = mockDevices[0x68] = mockDevices[0x69] = true;
      mockPedals[0] = false;
      setup();
      const uint32_t start = millis();
      while (pedalOpenChecks[0] == 0) {
        require(uint32_t(millis() - start) < 1000, "boot checks the open pedal");
        loop();
      }
      require(!pedalConnected[0] && !pedalChecked[0] && !device.status().ads && output("TEL").empty(),
              "first suspect boot probe cannot enable an unconfirmed pedal");
      run(800);
      require(protocol.state() == rgp::State::WaitAngle && pedalChecked[0] &&
              !pedalConnected[0] && contains("EVT", "PEDAL_DISCONNECTED,P1_ACCEL"),
              "persistent open pedal at boot is reported after confirmation");
      command(0, "OPEN_SESSION,0123456789ABCDEF"); command(1, "CALIBRATE,GYRO"); run(16000);
      Serial.output.clear(); run(100);
      const auto tel = output("TEL");
      require(protocol.state() == rgp::State::Ready && !device.status().ads && !tel.empty() &&
              tel.back().fields[4] == "0.000" && tel.back().fields[7] == "0.250",
              "confirmed open pedal is neutral while healthy pedal and IMUs keep running");
      mockPedals[0] = true;
      run(800);
      require(pedalConnected[0] && pedalSampled[0] && device.status().ads && pedalOpenChecks[0] == 0,
              "boot-disconnected pedal reconnects after healthy probe and normal sample");
      std::cout << "PASS: open-pedal boot simulation, " << assertions << " assertions.\n";
      return 0;
    }
    mockDevices[0x48] = mockDevices[0x68] = mockDevices[0x69] = true;
    setup();
    auto boot = output();
    require(boot[0].body == "SYS,0,0,HELLO,RGP,1.0,1.0.0", "HELLO boot sequence");
    require(boot[1].fields[3] == "BOOT" && boot[2].fields[4] == "BOOTING" &&
            boot[3].fields[4] == "INITIALIZING", "startup events");
    run(4800);
    require(output("TEL").empty(), "no TEL while handles settle");
    run(300);
    require(protocol.state() == rgp::State::WaitAngle && !device.status().calibrated && output("TEL").empty(),
            "unmeasured gyro calibration cannot claim READY or emit TEL");
    command(0, "OPEN_SESSION,0123456789ABCDEF");
    command(100, "CALIBRATE,GYRO");
    run(16000);
    require(protocol.state() == rgp::State::Ready, "two accelerators and both calibrated handles reach READY with A2/A3 disconnected");
    auto tel = output("TEL");
    require(!tel.empty() && tel.back().fields[4] == "0.700" && tel.back().fields[5] == "0.000" &&
            tel.back().fields[7] == "0.250" && tel.back().fields[8] == "0.000", "ADS A0/A1 accelerators and fixed-zero RGP brake fields");
    require(ads.conversionStarts[0] > 0 && ads.conversionStarts[1] > 0 &&
            ads.conversionStarts[2] == 0 && ads.conversionStarts[3] == 0, "only A0/A1 are sampled and probed");
    Serial.output.clear(); run(1000);
    require(output("TEL").size() == 100, "default 100 Hz without bursts");
    command(0, "OPEN_SESSION,0123456789ABCDEF");
    command(1, "SET_RATE,TEL,50");
    Serial.output.clear(); run(1000);
    require(output("TEL").size() == 50, "configured 50 Hz");
    command(101, "SET_RATE,TEL,1");
    run(50);
    command(102, "SET_RATE,TEL,100");
    Serial.output.clear(); run(100);
    require(output("TEL").size() == 10, "rate increase resets old one-second deadline");
    command(103, "SET_RATE,TEL,50");
    rapidPedalScenario(0);
    rapidPedalScenario(1);
    isolatedPedalProbeScenario(0);
    isolatedPedalProbeScenario(1);
    mockPedals[1] = false;
    run(800);
    require(protocol.state() == rgp::State::Ready && !device.status().ads, "missing P2 accelerator preserves IMU readiness");
    require(contains("EVT", "PEDAL_DISCONNECTED,P2_ACCEL"), "named P2 accelerator disconnect");
    Serial.output.clear(); run(100);
    require(!output("TEL").empty() && output("TEL").back().fields[7] == "0.000" &&
            output("TEL").back().fields[4] == "0.700", "missing P2 is neutral, P1 stays live");
    mockPedals[1] = true; run(800);
    require(protocol.state() == rgp::State::Ready, "pedal reconnect resumes READY");
    Serial.output.clear(); mockPedals[0] = false; run(800);
    require(protocol.state() == rgp::State::Ready &&
            contains("EVT", "PEDAL_DISCONNECTED,P1_ACCEL"), "missing P1 accelerator is also detected");
    Serial.output.clear(); run(100);
    require(!output("TEL").empty() && output("TEL").back().fields[4] == "0.000" &&
            output("TEL").back().fields[7] == "0.250", "missing P1 is neutral, P2 stays live");
    mockPedals[0] = true; run(800);
    require(protocol.state() == rgp::State::Ready, "P1 accelerator reconnect resumes READY");
    mockDevices[0x69] = false; Serial.output.clear(); run(20);
    require(protocol.state() == rgp::State::Ready, "remaining calibrated IMU continues operating after the other disconnects");
    require(contains("EVT", "IMU_LOST,0x69") && !contains("EVT", "I2C_LOST"), "optional IMU loss notification without whole-bus loss");
    Serial.output.clear(); run(1200);
    auto singleTel = output("TEL");
    require(!singleTel.empty() && singleTel.back().fields[6] == "0.00" && output("ERR").empty(),
            "retry is quiet and absent player's handle is neutral");
    mockDevices[0x69] = true; run(2500);
    require(protocol.state() == rgp::State::Ready && handles[1].online, "IMU recovers at its own address");
    Serial.output.clear();
    handles[1].angleDeg = 3600.01f; updateState(); outputLatestInputs();
    require(protocol.state() == rgp::State::WaitAngle && contains("EVT", "ANGLE_UNLOCKED"), "out-of-range angle invalidates reference");
    require(output("TEL").empty(), "out-of-range angle cannot emit TEL");
    command(2, "ANGLE_ZERO");
    require(protocol.state() == rgp::State::Ready && handles[0].angleDeg == 0 && handles[1].angleDeg == 0, "ANGLE_ZERO restores both angles atomically");
    size_t lockedEvents = 0;
    for (const auto& m : output("EVT")) if (m.fields[3] == "ANGLE_LOCKED") ++lockedEvents;
    require(lockedEvents == 1, "one ANGLE_LOCKED after zero");
    Serial.output.clear();
    command(3, "CALIBRATE,ALL");
    require(calibration.active && protocol.state() == rgp::State::Calibrating, "nonblocking calibration accepted");
    auto start = output();
    require(start[0].fields[0] == "ACK" && start[0].fields[5] == "ACCEPTED", "ACK precedes calibration events");
    command(4, "PING");
    require(contains("ACK", "4,PING,OK,PONG"), "serial serviced during calibration");
    command(5, "SCAN_I2C");
    require(contains("ACK", "5,SCAN_I2C,BUSY,CALIBRATE,3"), "scan/calibration conflict");
    run(11000);
    require(!calibration.active && contains("EVT", "CAL_MAG_SKIPPED,3") && contains("EVT", "CAL_GYRO_DONE,3") &&
            contains("EVT", "CAL_DONE,3,ALL"), "both gyro calibrations complete with request origin");
    run(5000);
    require(protocol.state() == rgp::State::Ready, "settles after calibration");
    Serial.output.clear(); command(6, "SCAN_I2C"); run(112);
    require(contains("EVT", "I2C_SCAN_START,6") && contains("EVT", "I2C_SCAN_END,6,3"), "scan count and origin");
    auto devices = output("SYS");
    require(devices.size() == 3 && devices[0].fields[5] == "0x48" && devices[1].fields[5] == "0x68" &&
            devices[2].fields[5] == "0x69", "scan ascending, unique results");
    require(output("TEL").empty(), "scan pauses TEL");
    Serial.output.clear(); run(100); require(!output("TEL").empty(), "TEL resumes after scan");
    Serial.output.clear(); command(7, "CALIBRATE,GYRO"); run(3500);
    mockDevices[0x69] = false; run(10);
    require(!calibration.active && contains("ERR", "ERROR,CALIBRATION,READ_FAILED,GYRO,REQ,7") &&
            !contains("EVT", "CAL_DONE,7,GYRO"), "calibration failure has REQ, no CAL_DONE");
    mockDevices[0x69] = true; run(3500);
    require(protocol.state() == rgp::State::Ready, "recovery after calibration failure");
    mockConversionStuck = true; Serial.output.clear(); run(200);
    require(protocol.state() == rgp::State::Ready && !adsOnline && !device.status().ads,
            "stuck ADC timeout detected without stopping IMU TEL");
    mockConversionStuck = false; run(2000);
    require(protocol.state() == rgp::State::Ready, "ADC timeout recovery");
    Serial.output.clear(); mockBusError = 4; command(8, "SCAN_I2C"); run(2);
    require(contains("ERR", "ERROR,I2C,READ_FAILED,I2C_SCAN,REQ,8") && !contains("EVT", "I2C_SCAN_END,8,0"), "failed scan has REQ, no end event");
    mockBusError = 0; run(3000);
    require(protocol.state() == rgp::State::Ready, "bus error recovery");
    require(ads.conversionStarts[2] == 0 && ads.conversionStarts[3] == 0,
            "unused A2/A3 stay unsampled through scans, calibration and recovery");
    protocol.setState(rgp::State::Fatal, millis()); Serial.output.clear(); command(9, "GET_STATUS");
    require(contains("ACK", "9,GET_STATUS,OK,FATAL,1,1,1,1") && output("TEL").empty(), "FATAL status remains accessible");
    std::cout << "PASS: firmware simulation, " << assertions << " assertions (hardware/filter doubles).\n";
  } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
  return 0;
}
