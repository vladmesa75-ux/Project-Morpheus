# Hardware test programs

This folder is for **bench testing the actual aircraft electronics and actuators**. It is deliberately separate from the airplane-mode flight firmware and from the experimental flight controllers in `experiments/`.

## Available tests

| Test | Arduino board | Purpose |
| --- | --- | --- |
| [Serial actuator test](serial-actuator-test/) | Arduino Due | Move Due servos, command the local DC motor, and send Nano flaperon/ESC commands using the USB Serial Monitor |
| [Emergency DC motor stop](emergency-motor-stop/) | Arduino Due | Minimal stand-alone sketch continuously setting both H-bridge inputs (pins 4 and 5) to zero |

The serial actuator test was recovered from the original `DueFlightComputerBase_v2_serial_actuator_test.ino` and packaged with copies of the relevant current airplane-mode support tabs. **The original test main tab was preserved, including historical help text and limitations.** See the detailed README before using it.

## Before using a hardware test

1. **Remove propellers and disconnect propulsion power** before uploading or inspecting the code.
2. Check actual pins, servo endpoints, linkage clearance, ESC arming behavior, and shared grounds against the assembled aircraft.
3. Be particularly careful with the **new H-tail folding servo on Due pin 43**. Its endpoint and safe state have not been calibrated.
4. The serial actuator test is **not a flight controller**. It does not provide the normal RC input handling, control mixer operation, or an automatic USB-command timeout.
5. Do not assume a generic 1500 µs servo pulse is safe for locking, folding or releasing mechanisms.
6. Test the emergency-stop program with propulsion power **physically disconnected** before relying on it. Flashing another sketch is not an emergency stop for a moving aircraft.

## Development rules

- Never overwrite the aircraft's main `firmware/airplane-mode/` sources to run a bench test.
- Give new tests their own sketch folder, local configuration, and a short README with pin mapping and safety notes.
- Hardware test configurations are local snapshots; changing the main flight firmware's `Config.h` does **not** automatically update them.
- Record which code and configuration were used for each bench test, and commit changes separately from the flight firmware.
