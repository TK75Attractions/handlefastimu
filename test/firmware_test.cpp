// Exercise the real setup()/loop() and protocol with deterministic hardware doubles.
// Filter math and physical sensor accuracy are outside this simulation.
#include "Arduino.h"
#include "Wire.h"
uint64_t mockTimeUs = 0;
int mockPinMode = INPUT, mockPinLevel = LOW, mockBusError = 0;
bool mockDevices[128] = {};
bool mockPedals[4] = {true, true, true, true};
bool mockConversionStuck = false;
float mockPedalVolts[4] = {2.31f, .825f, .99f, 2.64f};
MockSerial Serial;
TwoWire Wire;
#include "../src/main.cpp"
#include <iostream>
#include <stdexcept>

// Fake only the hardware wrappers and orientation estimates; the application is unchanged.
ImuManager::ImuManager(TwoWire& bus) : wire(bus), imuMpu6050(bus), imuMpu6500(bus),
  imuMpu9250(bus), imuMpu9255(bus), imuMpu6515(bus), imuMpu6886(bus),
  imuIcm20689(bus), imuIcm20690(bus) {}
bool ImuManager::selectAtAddress(uint8_t address) { imuAddress = address; return mockDevices[address]; }
uint8_t ImuManager::getWhoAmI() { return 0x68; }
uint8_t ImuManager::getImuAddress() { return imuAddress; }
int ImuManager::initImu(calData&) { return mockDevices[imuAddress] ? 0 : -1; }
bool ImuManager::hasMagnetometer() { return false; }
void ImuManager::update() {}
void ImuManager::getAccel(AccelData* out) { *out = {-1, 0, 0}; }
void ImuManager::getGyro(GyroData* out) { *out = {.1f, .2f, .3f}; }
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
int main() {
  try {
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
    require(protocol.state() == rgp::State::Ready, "all six calibrated application inputs reach READY");
    auto tel = output("TEL");
    require(!tel.empty() && tel.back().fields[4] == "0.700" && tel.back().fields[5] == "0.300" &&
            tel.back().fields[7] == "0.250" && tel.back().fields[8] == "0.800", "ADS A0/A1 accelerators and A2/A3 independent brakes");
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
    mockPedals[3] = false;
    run(800);
    require(protocol.state() == rgp::State::Recovering, "missing P2 brake stops both players");
    require(contains("ERR", "ERROR,ADS1115,READ_FAILED,P2_BRAKE"), "named pedal failure");
    Serial.output.clear(); run(100);
    require(output("TEL").empty(), "no substitute-zero or NaN TEL during pedal disconnect");
    mockPedals[3] = true; run(800);
    require(protocol.state() == rgp::State::Ready, "pedal reconnect resumes READY");
    mockDevices[0x69] = false; Serial.output.clear(); run(20);
    require(protocol.state() == rgp::State::Recovering, "IMU disconnect enters RECOVERING");
    require(contains("EVT", "I2C_LOST") && contains("ERR", "ERROR,I2C,CONNECTION_LOST"), "connection loss notifications");
    Serial.output.clear(); run(1200);
    require(output("TEL").empty() && output("ERR").empty(), "retry is quiet and TEL stays off");
    mockDevices[0x69] = true; run(2500);
    require(protocol.state() == rgp::State::Ready && contains("EVT", "I2C_RESTORED"), "IMU recovers at its own address");
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
    require(protocol.state() == rgp::State::Recovering && !adsOnline, "stuck ADC timeout detected");
    mockConversionStuck = false; run(2000);
    require(protocol.state() == rgp::State::Ready, "ADC timeout recovery");
    Serial.output.clear(); mockBusError = 4; command(8, "SCAN_I2C"); run(2);
    require(contains("ERR", "ERROR,I2C,READ_FAILED,I2C_SCAN,REQ,8") && !contains("EVT", "I2C_SCAN_END,8,0"), "failed scan has REQ, no end event");
    mockBusError = 0; run(3000);
    require(protocol.state() == rgp::State::Ready, "bus error recovery");
    protocol.setState(rgp::State::Fatal, millis()); Serial.output.clear(); command(9, "GET_STATUS");
    require(contains("ACK", "9,GET_STATUS,OK,FATAL,1,1,1,1") && output("TEL").empty(), "FATAL status remains accessible");
    std::cout << "PASS: firmware simulation, " << assertions << " assertions (hardware/filter doubles).\n";
  } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
