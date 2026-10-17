#pragma once
#include <stddef.h>
#include <stdint.h>
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
using std::isfinite;
constexpr int INPUT = 0, OUTPUT = 1, LOW = 0, HIGH = 1;
constexpr float PI = 3.14159265358979323846f;
extern uint64_t mockTimeUs;
extern int mockPinMode, mockPinLevel;
inline uint32_t micros() { return uint32_t(mockTimeUs); }
inline uint32_t millis() { return uint32_t(mockTimeUs / 1000); }
inline void delay(uint32_t ms) { mockTimeUs += uint64_t(ms) * 1000; }
inline void pinMode(int, int mode) { mockPinMode = mode; }
inline void digitalWrite(int, int level) { mockPinLevel = level; }
struct MockSerial {
  std::deque<uint8_t> input;
  std::string output;
  void begin(int) {}
  int available() const { return int(input.size()); }
  int read() { const int b = input.front(); input.pop_front(); return b; }
  size_t write(const uint8_t* bytes, size_t count) { output.append(reinterpret_cast<const char*>(bytes), count); return count; }
};
extern MockSerial Serial;
