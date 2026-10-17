#pragma once
#include "Arduino.h"
#include "Wire.h"
constexpr uint16_t ADS1X15_REG_CONFIG_MUX_SINGLE_0 = 0;
constexpr uint16_t ADS1X15_REG_CONFIG_MUX_SINGLE_1 = 1;
constexpr uint16_t ADS1X15_REG_CONFIG_MUX_SINGLE_2 = 2;
constexpr uint16_t ADS1X15_REG_CONFIG_MUX_SINGLE_3 = 3;
constexpr int GAIN_ONE = 1, RATE_ADS1115_860SPS = 860;
extern bool mockPedals[4], mockConversionStuck;
extern float mockPedalVolts[4];
struct Adafruit_ADS1115 {
  uint16_t channel = 0;
  int16_t conversion = 0;
  uint32_t started = 0;
  bool begin(uint8_t address, TwoWire*) { return mockDevices[address]; }
  void setGain(int) {}
  void setDataRate(int) {}
  void startADCReading(uint16_t mux, bool) {
    channel = mux;
    float volts = mockPedals[channel] ? mockPedalVolts[channel] :
      (mockPinMode == OUTPUT ? (mockPinLevel == HIGH ? 3.3f : 0) : .6f);
    conversion = int16_t(std::round(volts * 8000));
    started = millis();
  }
  bool conversionComplete() const { return !mockConversionStuck && uint32_t(millis() - started) >= 2; }
  int16_t getLastConversionResults() const { return conversion; }
  float computeVolts(int16_t raw) const { return raw / 8000.0f; }
};
