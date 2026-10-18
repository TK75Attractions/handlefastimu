#include "rgp_protocol.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

static size_t checks = 0;
static void check(bool condition, const std::string& message) {
  ++checks;
  if (!condition) throw std::runtime_error(message);
}
static std::vector<std::string> split(const std::string& s, char delimiter) {
  std::vector<std::string> result;
  size_t start = 0;
  for (;;) {
    size_t end = s.find(delimiter, start);
    result.push_back(s.substr(start, end - start));
    if (end == std::string::npos) return result;
    start = end + 1;
  }
}
static std::string unescape(const std::string& text) {
  std::string result;
  for (size_t i = 0; i < text.size(); ++i) {
    if (text[i] != '\\') { result += text[i]; continue; }
    char c = text.at(++i);
    if (c == 'n') result += '\n';
    else if (c == 'r') result += '\r';
    else if (c == 'x') {
      result += char(std::stoul(text.substr(i + 1, 2), nullptr, 16));
      i += 2;
    } else throw std::runtime_error("unsupported escape");
  }
  return result;
}
static std::vector<std::vector<std::string>> records(const std::string& path) {
  std::ifstream input(path);
  check(bool(input), "open " + path);
  std::vector<std::vector<std::string>> rows;
  for (std::string line; std::getline(input, line);)
    if (!line.empty() && line[0] != '#') rows.push_back(split(line, '|'));
  return rows;
}
struct FakeDevice : rgp::Device {
  std::vector<std::string> output;
  rgp::Status current;
  FakeDevice() { current.ads = current.imu = current.angle = current.calibrated = true; }
  rgp::Protocol* protocol = nullptr;
  int betaCalls = 0, zeroCalls = 0, calCalls = 0, scanCalls = 0;
  bool referenceAvailable = true;
  float beta = .2f;
  void write(const std::string& wire) override { output.push_back(wire); }
  rgp::Status status() const override { return current; }
  bool zeroAngles() override {
    if (!referenceAvailable) return false;
    ++zeroCalls;
    return true;
  }
  void setBeta(float value) override { ++betaCalls; beta = value; }
  void calibrate(uint32_t, const std::string&) override {
    ++calCalls;
    protocol->setState(rgp::State::Calibrating, 0);
  }
  void scan(uint32_t) override { ++scanCalls; }
};
struct Rig {
  FakeDevice device;
  rgp::Protocol protocol{device};
  uint32_t now = 0;
  Rig() { device.protocol = &protocol; protocol.begin(); device.output.clear(); }
  void feed(const std::string& wire) {
    for (unsigned char c : wire) protocol.receive(c, now);
  }
  void cmd(uint32_t seq, const std::string& command) {
    feed(rgp::frame("CMD," + std::to_string(seq) + ",0," + command));
  }
  void open() { cmd(0, "OPEN_SESSION,0123456789ABCDEF"); device.output.clear(); }
  void ready() { protocol.setState(rgp::State::Ready, now); device.output.clear(); }
  std::vector<rgp::Message> messages(const std::string& type = "") {
    std::vector<rgp::Message> result;
    rgp::Decoder decoder;
    rgp::Message m;
    for (const auto& wire : device.output) {
      check(wire.size() <= 256 && wire.back() == '\n', "TX size/terminator");
      bool decoded = false;
      for (unsigned char c : wire) if (decoder.feed(c, now, m)) {
        decoded = true;
        if (type.empty() || m.fields[0] == type) result.push_back(m);
      }
      check(decoded, "TX frame is valid: " + wire);
    }
    return result;
  }
};

static void suppliedVectors(const std::string& directory) {
  size_t crcCount = 0, espCount = 0, ignoredHost = 0, unsupportedContexts = 0;
  for (const auto& row : records(directory + "/crc-vectors.txt")) {
    std::string body = row[1] == "<EMPTY>" ? "" : row[1];
    check(rgp::crc16(body) == std::stoul(row[2], nullptr, 16), row[0] + " CRC");
    ++crcCount;
  }
  for (const auto& file : {"valid-frames.txt", "invalid-frames.txt"}) {
    for (const auto& row : records(directory + "/" + file)) {
      const std::string id = row[0], context = row[2], expected = row[4];
      const std::string wire = unescape(row[3]);
      if (row[1] != "HOST_TO_ESP") {
        // This repository implements the ESP endpoint, not PC TEL/stale/UTF-8 handling.
        // Valid host-facing examples still round-trip through the shared framing codec.
        if (std::string(file) == "valid-frames.txt") {
          rgp::Decoder decoder;
          rgp::Message m;
          int decoded = 0;
          for (unsigned char c : wire) if (decoder.feed(c, 0, m)) {
            ++decoded;
            std::string canonical = wire;
            if (canonical.size() > 1 && canonical[canonical.size() - 2] == '\r')
              canonical.erase(canonical.size() - 2, 1);
            check(rgp::frame(m.body) == canonical, id + " framing round trip");
          }
          check(decoded == 1, id + " valid host-facing frame");
        }
        ++ignoredHost;
        continue;
      }
      if ((expected == "ESP_ACCEPT_OK" || expected == "ESP_ACCEPT_ACCEPTED") &&
          (context.find("MAG_SUPPORTED") != std::string::npos ||
           context.find("SET_RATE_IMU_SUPPORTED") != std::string::npos ||
           context.find("SET_STREAM_DBG_SUPPORTED") != std::string::npos)) {
        ++unsupportedContexts;
        continue;
      }
      Rig rig;
      if (context.find("NO_SESSION") == std::string::npos) rig.open();
      if (context.find("READY") != std::string::npos) rig.ready();
      rig.feed(wire);
      rig.now += 1000;
      rig.protocol.tick(rig.now);
      auto acks = rig.messages("ACK");
      auto hasResult = [&](const std::string& result) {
        for (const auto& a : acks) {
          const std::string prefix = "ACK," + std::to_string(a.seq) + "," + std::to_string(a.time) + "," + a.fields[3] + "," + a.fields[4] + ",";
          if (a.body.substr(prefix.size()) == result) return true;
        }
        return false;
      };
      if (expected == "ESP_DISCARD_NO_ACK" || expected == "ESP_DISCARD_AFTER_1000MS_NO_ACK" ||
          expected == "ESP_ACCEPT_FRAME_AND_IGNORE_DIRECTION") check(acks.empty(), id + " no ACK");
      else if (expected == "ESP_EXECUTE_ONCE_ACK_TWICE") {
        check(acks.size() == 2 && hasResult("OK,TEL,0"), id + " duplicate ACKs");
        auto events = rig.messages("EVT");
        check(events.size() == 1 && events[0].fields[3] == "CONFIG_CHANGED", id + " one side effect");
      } else if (expected == "ESP_EXECUTE_FIRST_SECOND_ACK_FAILED_SEQ_REUSE_AND_ERR") {
        check(acks.size() == 2 && hasResult("FAILED,SEQ_REUSE") && rig.device.betaCalls == 1, id + " SEQ reuse");
        check(rig.messages("ERR").size() == 1, id + " SEQ error");
      } else if (expected.find("ESP_ACK_INVALID_ARG_") == 0) {
        check(acks.size() == 1 && hasResult("INVALID_ARG," + expected.substr(20)), id + " argument index");
      } else if (expected == "ESP_ACK_UNSUPPORTED" || expected == "ESP_ACCEPT_UNSUPPORTED") {
        check(acks.size() == 1 && hasResult("UNSUPPORTED"), id + " unsupported");
      } else if (expected == "ESP_ACCEPT_NOT_READY") {
        check(acks.size() == 1 && acks[0].fields[5] == "NOT_READY", id + " no session");
      } else if (expected == "ESP_ACCEPT_ACCEPTED") {
        check(acks.size() == 1 && acks[0].fields[5] == "ACCEPTED", id + " accepted");
        check(rig.device.calCalls + rig.device.scanCalls == 1, id + " scheduled once");
      } else {
        check(expected == "ESP_ACCEPT_OK" || expected == "ESP_DISCARD_PREFIX_AND_ACCEPT_PING" ||
              expected == "ESP_DISCARD_PARTIAL_AND_ACCEPT_PING", id + " recognized expectation");
        check(acks.size() == 1 && acks[0].fields[5] == "OK", id + " OK");
      }
      ++espCount;
    }
  }
  std::cout << "Supplied vectors: " << crcCount << " CRC, " << espCount << " ESP behavior passed; "
    << unsupportedContexts << " optional capability contexts not applicable; " << ignoredHost
    << " PC-side records (valid frames checked for codec round trip only).\n";
}

static void regressionTests() {
  {
    FakeDevice d;
    rgp::Protocol p(d);
    p.begin();
    check(d.output[0] == "@SYS,0,0,HELLO,RGP,1.0,1.0.0*9F6B\n", "HELLO first with SEQ/TIME zero");
    rgp::Decoder decoder;
    rgp::Message m;
    uint32_t seq = 0;
    for (const auto& w : d.output)
      for (unsigned char c : w) if (decoder.feed(c, 0, m)) check(m.seq == seq++, "shared TX SEQ");
  }
  {
    Rig r; r.open(); r.ready();
    r.cmd(1, "CALIBRATE,ALL");
    r.cmd(1, "CALIBRATE,ALL");
    check(r.device.calCalls == 1, "async retry does not restart operation");
    r.cmd(2, "SCAN_I2C");
    auto acks = r.messages("ACK");
    check(acks.back().fields[5] == "BUSY" && acks.back().fields[6] == "CALIBRATE" &&
          acks.back().fields[7] == "1", "async conflict origin");
    r.cmd(3, "PING");
    check(r.messages("ACK").back().fields[6] == "PONG", "PING serviced during calibration");
    r.cmd(0, "OPEN_SESSION,FEDCBA9876543210");
    check(r.device.calCalls == 1 && r.protocol.busy(), "new session keeps active work");
    r.cmd(4, "CALIBRATE,GYRO");
    check(r.messages("ACK").back().fields[5] == "BUSY", "busy across session change");
    r.cmd(5, "SET_RATE,IMU,0");
    check(r.messages("ACK").back().fields[5] == "INVALID_ARG", "validation before unsupported/busy/state");
    r.cmd(6, "SET_STREAM,DBG,1");
    check(r.messages("ACK").back().fields[5] == "UNSUPPORTED", "optional DBG disabled");
  }
  {
    Rig r; r.open(); r.ready();
    r.cmd(1, "SET_BETA,0.300");
    r.cmd(2, "SET_BETA,0.400");
    r.cmd(0, "OPEN_SESSION,0123456789ABCDEF");
    r.cmd(1, "SET_BETA,0.300");
    check(r.device.betaCalls == 2 && r.device.beta == .4f, "same session preserves receipts and cached result");
    r.cmd(0, "OPEN_SESSION,FEDCBA9876543210");
    r.cmd(1, "SET_BETA,0.300");
    check(r.device.betaCalls == 3 && r.device.beta == .3f, "different session clears old SEQ namespace");
  }
  {
    Rig r; r.open(); r.ready();
    r.cmd(1, "SET_BETA,0.300");
    r.now = 10001;
    r.cmd(1, "SET_BETA,0.300");
    check(r.device.betaCalls == 1, "cache survives time alone");
    for (uint32_t i = 2; i < 18; ++i) r.cmd(i, "PING");
    r.cmd(1, "SET_BETA,0.300");
    check(r.device.betaCalls == 2, "cache expires after time AND 16 later commands");
    Rig young; young.open(); young.ready();
    young.cmd(1, "SET_BETA,0.300");
    for (uint32_t i = 2; i < 30; ++i) young.cmd(i, "PING");
    young.now = 9999;
    young.cmd(1, "SET_BETA,0.300");
    check(young.device.betaCalls == 1, "cache survives count alone");
  }
  {
    Rig r; r.open(); r.ready();
    r.device.referenceAvailable = false;
    r.cmd(1, "ANGLE_ZERO");
    check(r.device.zeroCalls == 0 && r.messages("ACK")[0].fields[5] == "FAILED", "zero fails atomically");
    check(r.messages("EVT").empty() && r.messages("ERR").size() == 1, "failed zero has no success event");
    r.protocol.setState(rgp::State::WaitAngle, 1);
    r.device.output.clear(); r.device.referenceAvailable = true;
    r.cmd(2, "ANGLE_ZERO");
    auto messages = r.messages();
    check(messages[0].fields[0] == "ACK" && messages[1].fields[3] == "ANGLE_RESET" &&
          messages[2].fields[3] == "ANGLE_LOCKED" && messages[3].fields[4] == "READY", "zero ACK/event/state order");
  }
  {
    Rig r; r.open(); r.ready();
    rgp::Inputs in{{-12.5f, 25}, {.75f, 0}, {0, .4f}};
    check(r.protocol.telemetry(in, 8210), "six-input snapshot emitted");
    auto tel = r.messages("TEL");
    check(tel.size() == 1 && tel[0].fields.size() == 9 &&
          tel[0].body.substr(tel[0].body.find(",-12.50")) == ",-12.50,0.750,0.000,25.00,0.000,0.400", "TEL order and precision");
    in.accel[0] = .7f; in.brake[0] = .3f;
    check(r.protocol.telemetry(in, 8220), "independent simultaneous pedals");
    for (size_t player = 0; player < 2; ++player) {
      auto bad = in; bad.handle[player] = 3600.01f;
      check(!r.protocol.telemetry(bad, 0), "out of range angle stops whole TEL");
      bad = in; bad.accel[player] = std::numeric_limits<float>::quiet_NaN();
      check(!r.protocol.telemetry(bad, 0), "NaN stops whole TEL");
      bad = in; bad.brake[player] = 1.001f;
      check(!r.protocol.telemetry(bad, 0), "invalid brake stops whole TEL");
    }
    r.device.current.ads = false;
    check(r.protocol.telemetry(in, 0), "optional pedals do not stop TEL");
    r.device.current.imu = false;
    check(!r.protocol.telemetry(in, 0), "missing all IMUs stops TEL");
    r.device.current.imu = true;
    r.device.current.ads = true;
    r.cmd(1, "SET_STREAM,TEL,0");
    check(!r.protocol.telemetry(in, 0), "TEL stream off");
    r.cmd(2, "SET_STREAM,TEL,1");
    r.protocol.setState(rgp::State::Recovering, 0);
    check(!r.protocol.telemetry(in, 0), "RECOVERING blocks TEL");
    r.protocol.setState(rgp::State::Fatal, 0);
    r.protocol.setState(rgp::State::Ready, 0);
    check(r.protocol.state() == rgp::State::Fatal && !r.protocol.telemetry(in, 0), "FATAL latched");
    r.cmd(3, "PING");
    check(r.messages("ACK").back().fields[6] == "PONG", "FATAL maintains serial");
  }
  {
    Rig r; r.open(); r.ready();
    r.cmd(1, "SET_RATE,TEL,1"); check(r.protocol.telemetryPeriodUs() == 1000000, "1 Hz");
    r.cmd(2, "SET_RATE,TEL,100"); check(r.protocol.telemetryPeriodUs() == 10000, "100 Hz");
    r.device.output.clear();
    r.cmd(3, "SET_RATE,TEL,100"); check(r.messages("EVT").empty(), "no unchanged config event");
    r.cmd(4, "GET_CONFIG");
    check(r.messages("ACK").back().body.find("TEL_RATE,100,IMU_RATE,20,TEL_STREAM,1,IMU_STREAM,0,DBG_STREAM,0,BETA,0.200") != std::string::npos, "GET_CONFIG order/defaults");
    r.device.output.clear();
    r.feed("zZbBcC"); check(r.device.output.empty(), "legacy ignored");
  }
  {
    rgp::Decoder decoder; rgp::Message m;
    auto feed = [&](const std::string& w, uint32_t now) {
      int count = 0;
      for (unsigned char c : w) if (decoder.feed(c, now, m)) ++count;
      return count;
    };
    const std::string wire = rgp::frame("CMD,4294967295,4294967295,PING");
    check(feed(wire.substr(0, 12), UINT32_MAX - 500) == 0, "partial before millis wrap");
    check(feed(wire.substr(12), 100) == 1, "partial across millis wrap");
    feed(wire.substr(0, 12), 1000);
    check(feed(wire.substr(12), 2000) == 0, "1000 ms inactivity expires before next byte");
    check(feed(wire, 2000) == 1, "resync after timeout");
    check(feed("@" + std::string(300, 'A') + "\n" + wire, 2000) == 1, "overlong frame then good");
    // Derive the exact maximum body independently of the content's prefix length.
    std::string maximum(249, 'A'); maximum.replace(0, 9, "DBG,0,0,X");
    const auto maxWire = rgp::frame(maximum);
    check(maxWire.size() == 256 && feed(maxWire, 2000) == 1, "256-byte LF frame");
    auto crlf = maxWire; crlf.insert(crlf.size() - 1, "\r");
    check(crlf.size() == 257 && feed(crlf, 2000) == 1, "257-byte physical CRLF frame");
    check(rgp::frame(maximum + "A").empty(), "TX refuses overlength");
    check(feed(rgp::frame("CMD,0001,0000,PING"), 2000) == 1, "leading zeros accepted");
  }
  {
    float value;
    check(rgp::normalizePedal(1.75f, .2f, 3.3f, value) && std::fabs(value - .5f) < .00001, "per-pedal voltage normalization");
    check(rgp::normalizePedal(-1, 0, 3.3f, value) && value == 0, "pedal clamp low");
    check(rgp::normalizePedal(5, 0, 3.3f, value) && value == 1, "pedal clamp high");
    check(!rgp::normalizePedal(1, 1, 1, value), "bad calibration invalid");
    check(!rgp::normalizePedal(INFINITY, 0, 3.3f, value), "bad voltage invalid");
  }
}
int main(int argc, char** argv) {
  try {
    suppliedVectors(argc > 1 ? argv[1] : "test/vectors");
    regressionTests();
    std::cout << "PASS: " << checks << " assertions including regression checks.\n";
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
