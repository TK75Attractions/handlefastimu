#pragma once
#include <stdint.h>
extern bool mockDevices[128];
extern int mockBusError;
extern uint8_t mockImuIdentities[2];
struct TwoWire {
  uint8_t address = 0, reg = 0, readIndex = 0;
  void write(uint8_t value) { reg = value; }
  uint8_t requestFrom(uint8_t value, uint8_t count) { address = value; readIndex = 0; return mockDevices[value] && !mockBusError ? count : 0; }
  int read() { return reg == 0x75 && readIndex++ == 0 ? mockImuIdentities[address == 0x69 ? 1 : 0] : 0; }
  void begin(int, int) {}
  void setClock(int) {}
  void setTimeOut(int) {}
  void beginTransmission(uint8_t value) { address = value; }
  uint8_t endTransmission(bool) { return mockBusError ? uint8_t(mockBusError) : (mockDevices[address] ? 0 : 2); }
};
extern TwoWire Wire;
