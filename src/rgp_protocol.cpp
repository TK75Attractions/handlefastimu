#include "rgp_protocol.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace rgp {
namespace {
std::string number(uint32_t n) { return std::to_string(n); }
std::string fixed(double value, int precision) {
  char text[40];
  // Canonicalize rounded negative zero too.
  if (std::fabs(value) < (precision == 2 ? 0.005 : 0.0005)) value = 0;
  std::snprintf(text, sizeof(text), "%.*f", precision, value);
  return text;
}
bool identifier(const std::string& s, bool firstLetter = false) {
  if (s.empty() || (firstLetter && (s[0] < 'A' || s[0] > 'Z'))) return false;
  for (char c : s)
    if (!(c >= 'A' && c <= 'Z') && !(c >= '0' && c <= '9') &&
        c != '_' && c != '-' && c != '.') return false;
  return true;
}
int hex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
bool sessionId(const std::string& s) {
  if (s.size() != 16) return false;
  for (char c : s) if (hex(c) < 0) return false;
  return true;
}
std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> fields;
  size_t start = 0;
  for (;;) {
    size_t end = s.find(',', start);
    fields.push_back(s.substr(start, end - start));
    if (end == std::string::npos) return fields;
    start = end + 1;
  }
}
} // namespace

uint16_t crc16(const std::string& body) {
  uint16_t crc = 0xFFFF;
  for (unsigned char c : body) {
    crc ^= uint16_t(c) << 8;
    for (int i = 0; i < 8; ++i)
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
  }
  return crc;
}
std::string frame(const std::string& body) {
  if (body.size() + 7 > MaxFrameBytes) return {};
  char suffix[8];
  std::snprintf(suffix, sizeof(suffix), "*%04X\n", unsigned(crc16(body)));
  return "@" + body + suffix;
}
bool unsignedNumber(const std::string& text, uint32_t& value) {
  if (text.empty() || text.size() > 24) return false;
  uint32_t n = 0;
  for (char c : text) {
    if (c < '0' || c > '9' || n > (UINT32_MAX - uint32_t(c - '0')) / 10) return false;
    n = n * 10 + uint32_t(c - '0');
  }
  value = n;
  return true;
}
bool decimalNumber(const std::string& text, double& value) {
  if (text.empty() || text.size() > 24) return false;
  size_t i = text[0] == '-' ? 1 : 0;
  size_t first = i;
  while (i < text.size() && text[i] >= '0' && text[i] <= '9') ++i;
  if (i == first) return false;
  if (i < text.size() && text[i] == '.') {
    first = ++i;
    while (i < text.size() && text[i] >= '0' && text[i] <= '9') ++i;
    if (i == first) return false;
  }
  if (i != text.size()) return false;
  value = std::strtod(text.c_str(), nullptr);
  return std::isfinite(value);
}
void Decoder::expire(uint32_t now) {
  if (collecting && uint32_t(now - lastByte) >= 1000) {
    collecting = false;
    buffer.clear();
  }
}
bool Decoder::feed(uint8_t byte, uint32_t now, Message& message) {
  expire(now);
  if (byte == '@') {
    collecting = true;
    buffer = "@";
    lastByte = now;
    return false;
  }
  if (!collecting) return false;
  lastByte = now;
  if (byte != '\n') {
    // Allow 256 pre-LF bytes only for the optional final CR.
    if (byte == 0 || byte > 127 || buffer.size() >= MaxFrameBytes ||
        (!buffer.empty() && buffer.back() == '\r')) {
      collecting = false;
      buffer.clear();
      return false;
    }
    buffer += char(byte);
    return false;
  }
  collecting = false;
  if (buffer.back() == '\r') buffer.pop_back();
  if (buffer.size() + 1 > MaxFrameBytes) return false;
  const size_t star = buffer.find('*');
  if (star == std::string::npos || buffer.size() - star != 5) return false;
  uint16_t check = 0;
  for (size_t i = star + 1; i < buffer.size(); ++i) {
    int digit = hex(buffer[i]);
    if (digit < 0) return false;
    check = uint16_t((check << 4) | digit);
  }
  const std::string body = buffer.substr(1, star - 1);
  for (unsigned char c : body) if (c < 32 || c > 126) return false;
  if (crc16(body) != check) return false;
  auto fields = split(body);
  uint32_t seq, time;
  if (fields.size() < 3 || !identifier(fields[0], true) ||
      !unsignedNumber(fields[1], seq) || !unsignedNumber(fields[2], time)) return false;
  // CMD empty arguments must reach INVALID_ARG; an unreadable command gets no ACK.
  if (fields[0] == "CMD" && (fields.size() < 4 || !identifier(fields[3]))) return false;
  if (fields[0] != "CMD")
    for (const auto& f : fields) if (f.empty()) return false;
  message.body = body;
  message.fields = fields;
  message.seq = seq;
  message.time = time;
  return true;
}
const char* stateName(State s) {
  switch (s) {
    case State::Booting: return "BOOTING";
    case State::Initializing: return "INITIALIZING";
    case State::Calibrating: return "CALIBRATING";
    case State::WaitAngle: return "WAIT_ANGLE";
    case State::Ready: return "READY";
    case State::Recovering: return "RECOVERING";
    case State::Fatal: return "FATAL";
  }
  return "FATAL";
}
bool validInputs(const Inputs& in) {
  for (size_t i = 0; i < 2; ++i) {
    if (!std::isfinite(in.handle[i]) || in.handle[i] < -3600 || in.handle[i] > 3600 ||
        !std::isfinite(in.accel[i]) || in.accel[i] < 0 || in.accel[i] > 1 ||
        !std::isfinite(in.brake[i]) || in.brake[i] < 0 || in.brake[i] > 1) return false;
  }
  return true;
}
bool normalizePedal(float volts, float minimum, float maximum, float& result) {
  if (!std::isfinite(volts) || !std::isfinite(minimum) || !std::isfinite(maximum) ||
      minimum >= maximum) return false;
  result = std::max(0.0f, std::min(1.0f, (volts - minimum) / (maximum - minimum)));
  return true;
}
void Protocol::begin() {
  txSequence = 0;
  send("SYS", "HELLO,RGP,1.0,1.0.0", 0);
  event("BOOT", 0);
  currentState = State::Booting;
  event("STATE,BOOTING", 0);
}
void Protocol::send(const char* type, const std::string& data, uint32_t now) {
  const std::string wire = frame(std::string(type) + "," + number(txSequence) + "," +
                                 number(now) + (data.empty() ? "" : "," + data));
  if (wire.empty()) return;
  ++txSequence;
  device.write(wire);
}
void Protocol::setState(State state, uint32_t now) {
  if (currentState == State::Fatal || currentState == state) return;
  currentState = state;
  event(std::string("STATE,") + stateName(state), now);
}
void Protocol::receive(uint8_t byte, uint32_t now) {
  Message message;
  if (decoder.feed(byte, now, message) && message.fields[0] == "CMD") command(message, now);
}
void Protocol::tick(uint32_t now) { decoder.expire(now); }
bool Protocol::telemetry(const Inputs& inputs, uint32_t now) {
  const Status s = device.status();
  if (currentState != State::Ready || !telStream || busy() || !s.imu ||
      !s.angle || !s.calibrated || !validInputs(inputs)) return false;
  send("TEL", fixed(inputs.handle[0], 2) + "," + fixed(inputs.accel[0], 3) + "," +
       fixed(inputs.brake[0], 3) + "," + fixed(inputs.handle[1], 2) + "," +
       fixed(inputs.accel[1], 3) + "," + fixed(inputs.brake[1], 3), now);
  return true;
}

void Protocol::command(const Message& m, uint32_t now) {
  const std::string& name = m.fields[3];
  const size_t argc = m.fields.size() - 4;
  auto arg = [&](size_t i) -> std::string { return argc > i ? m.fields[i + 4] : ""; };
  auto ack = [&](const std::string& result) {
    send("ACK", number(m.seq) + "," + name + "," + result, now);
  };
  int invalid = 0;
  size_t expected = 0;
  bool known = true;
  double betaValue = 0;
  uint32_t rate = 0;
  if (name == "OPEN_SESSION") {
    expected = 1;
    if (!sessionId(arg(0))) invalid = 1;
  } else if (name == "SET_BETA") {
    expected = 1;
    if (!decimalNumber(arg(0), betaValue) || betaValue < .01 || betaValue > 1) invalid = 1;
  } else if (name == "SET_RATE") {
    expected = 2;
    if (arg(0) != "TEL" && arg(0) != "IMU") invalid = 1;
    else if (!unsignedNumber(arg(1), rate) || rate < 1 || rate > (arg(0) == "TEL" ? 100U : 50U)) invalid = 2;
  } else if (name == "SET_STREAM") {
    expected = 2;
    if (arg(0) != "TEL" && arg(0) != "IMU" && arg(0) != "DBG") invalid = 1;
    else if (arg(1) != "0" && arg(1) != "1") invalid = 2;
  } else if (name == "CALIBRATE") {
    expected = 1;
    if (arg(0) != "GYRO" && arg(0) != "MAG" && arg(0) != "ALL") invalid = 1;
  } else if (name != "PING" && name != "GET_STATUS" && name != "GET_CONFIG" &&
             name != "ANGLE_ZERO" && name != "SCAN_I2C") known = false;
  if (known) {
    if (argc < expected && !invalid) invalid = int(argc + 1);
    if (argc > expected && !invalid) invalid = int(expected + 1);
  } else {
    for (size_t i = 0; i < argc; ++i) if (arg(i).empty()) { invalid = int(i + 1); break; }
  }

  // A different valid session bypasses the old SEQ space, including its OPEN_SESSION.
  const bool newSession = name == "OPEN_SESSION" && !invalid && arg(0) != session;
  if (newSession) { receipts.clear(); processed = 0; }
  receipts.erase(std::remove_if(receipts.begin(), receipts.end(), [&](const Receipt& r) {
    return processed - r.ordinal >= 16 && uint32_t(now - r.at) >= 10000;
  }), receipts.end());
  for (const auto& r : receipts) if (r.seq == m.seq) {
    if (r.body == m.body) ack(r.result);
    else {
      ack("FAILED,SEQ_REUSE");
      error("ERROR,PROTOCOL,SEQ_REUSE," + number(m.seq), now);
    }
    return;
  }

  std::string result;
  std::string changed;
  bool zeroed = false;
  bool startCal = false, startScan = false;
  if (name != "OPEN_SESSION" && session.empty()) result = "NOT_READY," + std::string(stateName(currentState));
  else if (invalid) result = "INVALID_ARG," + number(uint32_t(invalid));
  else if (!known || (name == "CALIBRATE" && arg(0) == "MAG") ||
           ((name == "SET_RATE" || name == "SET_STREAM") && arg(0) != "TEL")) result = "UNSUPPORTED";
  else if (busy() && (name == "CALIBRATE" || name == "SCAN_I2C"))
    result = "BUSY," + activeCommand + "," + number(activeOrigin);
  else if ((name == "ANGLE_ZERO" || name == "SET_BETA" || name == "CALIBRATE") &&
           currentState != State::Ready && currentState != State::WaitAngle)
    result = "NOT_READY," + std::string(stateName(currentState));
  else if (name == "OPEN_SESSION") {
    session = arg(0);
    result = "OK," + session;
  } else if (name == "PING") result = "OK,PONG";
  else if (name == "GET_STATUS") {
    const Status s = device.status();
    result = std::string("OK,") + stateName(currentState) + "," + (s.ads ? "1" : "0") + "," +
      (s.imu ? "1" : "0") + "," + (s.angle ? "1" : "0") + "," + (s.calibrated ? "1" : "0");
  } else if (name == "GET_CONFIG") {
    result = "OK,TEL_RATE," + number(telRate) + ",IMU_RATE,20,TEL_STREAM," + (telStream ? "1" : "0") +
      ",IMU_STREAM,0,DBG_STREAM,0,BETA," + fixed(currentBeta, 3);
  } else if (name == "ANGLE_ZERO") {
    zeroed = device.zeroAngles();
    result = zeroed ? "OK" : "FAILED,ANGLE_REFERENCE_INVALID";
  } else if (name == "SET_BETA") {
    // Use exactly the value reported to the peer, with the specified precision.
    const float actual = float(std::round(betaValue * 1000) / 1000);
    if (actual != currentBeta) changed = "BETA," + fixed(actual, 3);
    currentBeta = actual;
    device.setBeta(actual);
    result = "OK," + fixed(actual, 3);
  } else if (name == "SET_RATE") {
    if (telRate != rate) changed = "TEL_RATE," + number(rate);
    telRate = rate;
    result = "OK,TEL," + number(rate);
  } else if (name == "SET_STREAM") {
    const bool enabled = arg(1) == "1";
    if (telStream != enabled) changed = std::string("TEL_STREAM,") + (enabled ? "1" : "0");
    telStream = enabled;
    result = std::string("OK,TEL,") + (enabled ? "1" : "0");
  } else if (name == "CALIBRATE" || name == "SCAN_I2C") {
    activeCommand = name;
    activeOrigin = m.seq;
    startCal = name == "CALIBRATE";
    startScan = !startCal;
    result = startCal ? "ACCEPTED," + arg(0) : "ACCEPTED";
  }
  ++processed;
  receipts.push_back({m.seq, now, processed, m.body, result});
  ack(result);
  if (!changed.empty()) event("CONFIG_CHANGED," + changed, now);
  if (name == "ANGLE_ZERO" && result == "FAILED,ANGLE_REFERENCE_INVALID")
    error("WARN,HANDLE,ANGLE_REFERENCE_INVALID", now);
  if (zeroed) {
    event("ANGLE_RESET", now);
    event("ANGLE_LOCKED", now);
    const Status s = device.status();
    if (s.imu && s.angle && s.calibrated) setState(State::Ready, now);
  }
  if (startCal) device.calibrate(m.seq, arg(0));
  if (startScan) device.scan(m.seq);
}
} // namespace rgp
