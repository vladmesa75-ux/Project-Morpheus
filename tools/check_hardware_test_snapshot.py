#!/usr/bin/env python3
"""Compare serial actuator bench-test support tabs against airplane-mode firmware.

Run from anywhere:
    python tools/check_hardware_test_snapshot.py

This is a source-synchronization check, not a compiler or hardware safety test.
A difference may be intentional, but it must be reviewed before bench testing.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
FLIGHT = ROOT / "firmware/airplane-mode/DueFlightComputerBase_v2"
TEST = ROOT / (
    "hardware-tests/serial-actuator-test/"
    "DueFlightComputerBase_v2_serial_actuator_test"
)

COMPANION_FILES = (
    "Config.h",
    "TypesAndGlobals.h",
    "FunctionPrototypes.h",
    "Servos.ino",
    "MotorDriver.ino",
    "LoraTelemetry.ino",
    "CommandDemo.ino",
)


def main():
    differing = []
    for filename in COMPANION_FILES:
        original = FLIGHT / filename
        snapshot = TEST / filename
        if not original.is_file() or not snapshot.is_file():
            print(f"MISSING: {filename}")
            differing.append(filename)
        elif original.read_bytes() == snapshot.read_bytes():
            print(f"OK: {filename}")
        else:
            print(f"DIFFERS: {filename}")
            differing.append(filename)

    if differing:
        print(
            "\nReview every difference before running the bench test. "
            "Do not overwrite the test configuration blindly."
        )
        return 1

    print("\nAll serial-test companion tabs match the current flight firmware.")
    print("The serial-test main tab remains separate and was not compared.")
    print("Note: source matching does not prove correct wiring or safe servo travel.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
