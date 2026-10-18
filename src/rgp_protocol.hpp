#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>
#include <vector>

// No Arduino dependencies: the exact firmware codec/command handler is host-testable.
namespace rgp {
constexpr size_t MaxFrameBytes = 256;
uint16_t crc16(const std::string& body);
std::string frame(const std::string& body);
bool unsignedNumber(const std::string& text, uint32_t& value);
bool decimalNumber(const std::string& text, double& value);

struct Message {
  std::string body;
  std::vector<std::string> fields;
  uint32_t seq = 0;
  uint32_t time = 0;
};

class Decoder {
public:
  bool feed(uint8_t byte, uint32_t now, Message& message);
  void expire(uint32_t now);
private:
  std::string buffer;
  uint32_t lastByte = 0;
  bool collecting = false;
};

enum class State { Booting, Initializing, Calibrating, WaitAngle, Ready, Recovering, Fatal };
const char* stateName(State state);
struct Status {
  bool ads = false;
  bool imu = false;
  bool angle = false;
  bool calibrated = false;
};
struct Inputs {
  float handle[2];
  float accel[2];
  float brake[2];
};
bool validInputs(const Inputs& inputs);
bool normalizePedal(float volts, float minimum, float maximum, float& result);

class Device {
public:
  virtual ~Device() = default;
  virtual void write(const std::string& wire) = 0;
  virtual Status status() const = 0;
  // Must check all connected references before changing any handle.
  virtual bool zeroAngles() = 0;
  virtual void setBeta(float value) = 0;
  // These only schedule work; serial input must keep being serviced.
  virtual void calibrate(uint32_t origin, const std::string& target) = 0;
  virtual void scan(uint32_t origin) = 0;
};

class Protocol {
public:
  explicit Protocol(Device& device) : device(device) {}
  void begin();
  void receive(uint8_t byte, uint32_t now);
  void tick(uint32_t now);
  void send(const char* type, const std::string& data, uint32_t now);
  void event(const std::string& data, uint32_t now) { send("EVT", data, now); }
  void error(const std::string& data, uint32_t now) { send("ERR", data, now); }
  void setState(State state, uint32_t now);
  State state() const { return currentState; }
  bool telemetry(const Inputs& inputs, uint32_t now);
  uint32_t telemetryPeriodUs() const { return 1000000U / telRate; }
  bool telemetryEnabled() const { return telStream; }
  float beta() const { return currentBeta; }
  bool busy() const { return !activeCommand.empty(); }
  void finishOperation() { activeCommand.clear(); }
private:
  struct Receipt {
    uint32_t seq;
    uint32_t at;
    uint64_t ordinal;
    std::string body;
    std::string result;
  };
  Device& device;
  Decoder decoder;
  State currentState = State::Booting;
  uint32_t txSequence = 0;
  uint64_t processed = 0;
  uint32_t telRate = 100;
  float currentBeta = 0.2f;
  bool telStream = true;
  std::string session;
  std::vector<Receipt> receipts;
  std::string activeCommand;
  uint32_t activeOrigin = 0;
  void command(const Message& message, uint32_t now);
};
} // namespace rgp
