#!/usr/bin/env python3
"""Compile and run the firmware's portable C++ protocol, using supplied RGP vectors."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent
with tempfile.TemporaryDirectory(prefix="rgp-tests-") as temporary:
    executable = Path(temporary) / "protocol_test"
    subprocess.run([
        os.environ.get("CXX", "c++"), "-std=c++11", "-Wall", "-Wextra", "-Werror",
        "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-I", str(ROOT / "src"), str(ROOT / "src/rgp_protocol.cpp"),
        str(ROOT / "test/protocol_test.cpp"), "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable), str(ROOT / "test/vectors")], check=True)

    firmware = Path(temporary) / "firmware_test"
    subprocess.run([
        os.environ.get("CXX", "c++"), "-std=c++11", "-Wall", "-Wextra", "-Werror",
        "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-I", str(ROOT / "test/stubs"), "-I", str(ROOT / "src"),
        str(ROOT / "src/rgp_protocol.cpp"), str(ROOT / "test/firmware_test.cpp"),
        "-o", str(firmware),
    ], check=True)
    for mode in ("both", "p1", "p2", "none", "steering-only"):
        subprocess.run([str(firmware), mode], check=True)

    merged = Path(temporary) / "merged_features_test"
    subprocess.run([
        os.environ.get("CXX", "c++"), "-std=c++11", "-Wall", "-Wextra", "-Werror",
        "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        "-I", str(ROOT / "test/stubs"), "-I", str(ROOT / "src"),
        str(ROOT / "src/rgp_protocol.cpp"), str(ROOT / "test/merged_features_test.cpp"),
        "-o", str(merged),
    ], check=True)
    subprocess.run([str(merged)], check=True)
