#!/usr/bin/env python3
"""Exercise the real application with every identity supported by ImuManager.

I2C, sensor drivers and orientation math are mocked; the real ImuManager and
application verify register reads, calibration, player mapping and reconnect.
"""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent
with tempfile.TemporaryDirectory(prefix="imu-identity-tests-") as temporary:
    executable = Path(temporary) / "imu_identity_test"
    subprocess.run([
        os.environ.get("CXX", "c++"), "-std=c++11", "-Wall", "-Wextra", "-Werror",
        "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-I", str(ROOT / "test/stubs"), "-I", str(ROOT / "src"),
        str(ROOT / "src/rgp_protocol.cpp"), str(ROOT / "src/imu_manager.cpp"),
        str(ROOT / "test/imu_identity_test.cpp"),
        "-o", str(executable),
    ], check=True)
    for identity in ("0x68", "0x70", "0x71", "0x73", "0x74", "0x75", "0x19", "0x98", "0x20"):
        subprocess.run([str(executable), identity], check=True)
