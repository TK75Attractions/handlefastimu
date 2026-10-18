// Run the production application with different P1/P2 WHO_AM_I values.
#define FIRMWARE_REAL_IMU_MANAGER
#define main firmwareRegressionMain
#include "firmware_test.cpp"
#undef main
#include <cstdlib>

int main(int argc, char** argv) {
  try {
    require(argc == 2, "P2 identity argument required");
    mockImuIdentities[1] = static_cast<uint8_t>(std::strtoul(argv[1], nullptr, 0));
    Wire.identities[0x68] = 0x68;
    Wire.identities[0x69] = mockImuIdentities[1];
    // P2 ACKs scans but requires a repeated START for register reads.
    Wire.requiresRepeatedStart[0x69] = true;
    mockDevices[0x48] = mockDevices[0x68] = mockDevices[0x69] = true;
    setup(); run(5100);
    require(handles[0].online && handles[1].online, "both identities initialize at their own player addresses");
    const float expectedP2Gyro = mockImuIdentities[1] == 0x75 ? 10 * 2000.0f / 32768 : .6f;
    require(handles[0].gyro.gyroZ == .3f && handles[1].gyro.gyroZ == expectedP2Gyro,
            "real ImuManager reads independent samples from 0x68 and 0x69");
    require(contains("SYS", "DEVICE,IMU,0x68,0x68") &&
            contains("SYS", "DEVICE,IMU,0x69," + addressText(mockImuIdentities[1])),
            "report the actual identity independently from the I2C address");
    require(protocol.state() == rgp::State::WaitAngle, "different identity still requires calibration");
    command(0, "OPEN_SESSION,0123456789ABCDEF");
    command(1, "CALIBRATE,GYRO"); run(16000);
    require(protocol.state() == rgp::State::Ready && handles[0].calib.valid && handles[1].calib.valid,
            "both identities calibrate and reach READY");
    require(!contains("ERR", "ERROR,HANDLE,ANGLE_REFERENCE_INVALID"),
            "calibration reference reset does not emit a spurious angle error");
    size_t adsReadyEvents = 0;
    for (const auto& m : output("EVT")) if (m.fields[3] == "ADS_READY") ++adsReadyEvents;
    require(adsReadyEvents == 1, "blocking IMU reinitialization does not falsely time out ADS conversion");
    handles[0].angleDeg = -12.5f;
    handles[1].angleDeg = 37.5f;
    Serial.output.clear(); delay(10); outputLatestInputs();
    auto tel = output("TEL");
    require(tel.size() == 1 && tel[0].fields[3] == "-12.50" && tel[0].fields[6] == "37.50",
            "mixed identities preserve independent P1/P2 steering");
    command(2, "ANGLE_ZERO");
    require(contains("ACK", "2,ANGLE_ZERO,OK") && handles[0].angleDeg == 0 && handles[1].angleDeg == 0,
            "ANGLE_ZERO applies to both sensor types");
    mockDevices[0x69] = false; run(20);
    require(!handles[1].online, "disconnected P2 goes offline");
    mockDevices[0x69] = true; Serial.output.clear(); run(3000);
    require(handles[1].online && protocol.state() == rgp::State::Ready && !output("TEL").empty(),
            "P2 reconnect accepts the different identity and resumes telemetry");
    require(contains("SYS", "DEVICE,IMU,0x69," + addressText(mockImuIdentities[1])),
            "P2 reconnect reports the same identity at the same address");
    if (mockImuIdentities[1] == 0x75) {
      require(Wire.magnetometerReads == 0, "six-axis compatible IMU never reads an absent magnetometer");
      Wire.shortSampleRead = true; run(10);
      require(!handles[1].online && !isfinite(handles[1].gyro.gyroZ),
              "short compatible-IMU sample invalidates data and disconnects instead of accepting stale values");
      Wire.shortSampleRead = false; run(3000);
      require(handles[1].online && protocol.state() == rgp::State::Ready,
              "compatible IMU recovers after a short sample read");
      require(fabsf(handles[1].gyro.gyroZ) < .0001f,
              "compatible IMU reapplies measured gyro bias after reconnect");
    }
    ImuManager detector(Wire);
    Wire.shortRead = true;
    require(!detector.selectAtAddress(0x69) && detector.getWhoAmI() == 0xFF &&
            detector.initImu(handles[1].calib) != 0, "short identity read cannot select or initialize a driver");
    Wire.shortRead = false;
    Wire.identities[0x69] = 0x42;
    require(!detector.selectAtAddress(0x69), "unknown identity is rejected despite I2C ACK");
    mockDevices[0x69] = false;
    require(!detector.selectAtAddress(0x69) && detector.getImuAddress() == 0,
            "missing P2 never falls back to P1");
    std::cout << "PASS: P2 identity " << argv[1] << ", " << assertions << " assertions.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
