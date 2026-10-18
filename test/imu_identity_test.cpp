// Run the production application with different P1/P2 WHO_AM_I values.
#define main firmwareRegressionMain
#include "firmware_test.cpp"
#undef main
#include <cstdlib>

int main(int argc, char** argv) {
  try {
    require(argc == 2, "P2 identity argument required");
    mockImuIdentities[1] = static_cast<uint8_t>(std::strtoul(argv[1], nullptr, 0));
    mockDevices[0x48] = mockDevices[0x68] = mockDevices[0x69] = true;
    setup(); run(5100);
    require(handles[0].online && handles[1].online, "both identities initialize at their own player addresses");
    require(contains("SYS", "DEVICE,IMU,0x68,0x68") &&
            contains("SYS", "DEVICE,IMU,0x69," + addressText(mockImuIdentities[1])),
            "report the actual identity independently from the I2C address");
    require(protocol.state() == rgp::State::WaitAngle, "different identity still requires calibration");
    command(0, "OPEN_SESSION,0123456789ABCDEF");
    command(1, "CALIBRATE,GYRO"); run(16000);
    require(protocol.state() == rgp::State::Ready && handles[0].calib.valid && handles[1].calib.valid,
            "both identities calibrate and reach READY");
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
    mockDevices[0x69] = true; Serial.output.clear(); run(2500);
    require(handles[1].online && protocol.state() == rgp::State::Ready && !output("TEL").empty(),
            "P2 reconnect accepts the different identity and resumes telemetry");
    require(contains("SYS", "DEVICE,IMU,0x69," + addressText(mockImuIdentities[1])),
            "P2 reconnect reports the same identity at the same address");
    std::cout << "PASS: P2 identity " << argv[1] << ", " << assertions << " assertions.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
