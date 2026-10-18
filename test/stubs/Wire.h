#pragma once
#include <stddef.h>
#include <stdint.h>
extern bool mockDevices[128];
extern int mockBusError;
struct TwoWire {
  uint8_t address = 0;
  uint8_t identities[128] = {};
  bool requiresRepeatedStart[128] = {};
  bool shortRead = false;
  uint8_t reg = 0;
  bool registerWritten = false, repeatedStart = false, readable = false;
  uint8_t remaining = 0, offset = 0;
  unsigned magnetometerReads = 0;
  bool shortSampleRead = false;
  void begin(int, int) {}
  void setClock(int) {}
  void setTimeOut(int) {}
  void beginTransmission(uint8_t value) { address = value; registerWritten = false; repeatedStart = false; }
  size_t write(uint8_t value) { reg = value; registerWritten = true; return 1; }
  uint8_t endTransmission(bool stop = true) {
    repeatedStart = registerWritten && !stop;
    return mockBusError ? uint8_t(mockBusError) : (mockDevices[address] ? 0 : 2);
  }
  uint8_t requestFrom(int target, int count) {
    if (target == 0x0C) ++magnetometerReads;
    const bool sample = count == 14 && reg == 0x3B && !shortSampleRead;
    readable = target == address && ((count == 1 && reg == 0x75) || sample) && registerWritten &&
      mockDevices[address] && !mockBusError && !shortRead &&
      (!requiresRepeatedStart[address] || repeatedStart);
    remaining = readable ? uint8_t(count) : 0;
    offset = 0;
    return remaining;
  }
  int read() {
    if (!remaining) return -1;
    --remaining;
    if (reg == 0x75) return identities[address];
    // Generic six-axis device: -1 g on X, +10 gyro counts on Z.
    const uint8_t sample[14] = {0xF8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10};
    return sample[offset++];
  }
};
extern TwoWire Wire;
