// Exercise the merged pedal filter and RGP diagnostics through production setup/loop.
#define main firmwareRegressionMain
#include "firmware_test.cpp"
#undef main

int main() {
  try {
    PedalFilter filtered;
    filtered.update(.5f, 0);
    float maxNoise = 0;
    for (uint32_t i = 1; i <= 100; ++i) {
      const float value = filtered.update(.5f + (i % 2 ? .01f : -.01f), i * 5000);
      maxNoise = std::max(maxNoise, std::fabs(value - .5f));
    }
    require(maxNoise < .004f, "stationary pedal noise is reduced");
    float response = 0;
    for (uint32_t i = 101; i <= 110; ++i) response = filtered.update(1, i * 5000);
    require(response > .95f, "intentional pedal movement responds within 50ms");
    filtered.reset(); require(!filtered.hasValue(), "disconnect clears the filter");
    require(filtered.update(.1f, 600000) == .1f, "reconnect starts at the new sample");

    mockDevices[0x48] = mockDevices[0x68] = mockDevices[0x69] = true;
    mockImuIdentities[1] = 0x70;
    setup(); run(5100);
    require(output("DBG").empty(), "diagnostics default off");
    command(0, "OPEN_SESSION,0123456789ABCDEF");
    command(1, "CALIBRATE,GYRO");
    command(2, "SET_STREAM,DBG,1");
    Serial.output.clear(); run(1000);
    require(output("DBG").empty(), "diagnostics defer during calibration");
    run(15000);
    require(protocol.state() == rgp::State::Ready, "merged features still reach READY");
    Serial.output.clear(); run(1100);
    auto logs = output("DBG");
    require(!logs.empty() && output("TEL").size() == 110, "diagnostics and 100Hz two-player TEL coexist");
    bool identity = false, pedal = false;
    for (const auto& log : logs) {
      require(log.fields.size() == 5 && rgp::frame(log.body).size() <= rgp::MaxFrameBytes,
              "diagnostic text fits a complete CRC-protected RGP frame");
      const auto& text = log.fields[4];
      for (size_t i = 0; i < text.size(); ++i) {
        const unsigned char c = text[i];
        if (c == '%') {
          require(i + 2 < text.size() && std::string("0123456789ABCDEF").find(text[i+1]) != std::string::npos &&
                  std::string("0123456789ABCDEF").find(text[i+2]) != std::string::npos, "uppercase percent encoding");
          i += 2;
        } else require((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                       (c >= '0' && c <= '9') || c == '-' || c == '.' || c == '_' || c == '~', "reserved text bytes are encoded");
      }
      identity = identity || text.find("WHO_AM_I%3D0x70") != std::string::npos;
      pedal = pedal || (log.fields[3] == "ADS1115" && text.find("samples%3D") != std::string::npos);
    }
    require(identity && pedal, "diagnostics include actual P2 identity and pedal samples");
    command(3, "SCAN_I2C"); Serial.output.clear(); run(50);
    require(output("DBG").empty(), "diagnostics do not interfere with an active I2C scan");
    run(200);
    command(4, "SET_STREAM,DBG,0"); Serial.output.clear(); run(1100);
    require(output("DBG").empty() && !output("TEL").empty(), "turning diagnostics off preserves game input");
    mockDevices[0x48] = false; run(10);
    require(!pedalFilters[0].hasValue() && !pedalFilters[1].hasValue(), "ADS loss clears both independent filters");
    std::cout << "PASS: merged filter/diagnostics features, " << assertions << " assertions.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n'; return 1;
  }
}
