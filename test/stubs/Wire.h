#pragma once
#include <stdint.h>
extern bool mockDevices[128];
extern int mockBusError;
struct TwoWire {
  uint8_t address = 0;
  void begin(int, int) {}
  void setClock(int) {}
  void setTimeOut(int) {}
  void beginTransmission(uint8_t value) { address = value; }
  uint8_t endTransmission(bool) { return mockBusError ? uint8_t(mockBusError) : (mockDevices[address] ? 0 : 2); }
};
extern TwoWire Wire;
