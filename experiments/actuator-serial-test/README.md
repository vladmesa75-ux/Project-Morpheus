# Serial actuator test (original 18-byte Due experiment)

**Status:** Archived exactly as generated in the 5 November 2026 conversation. **Not a ready-to-flash standalone sketch and not compatible as-is with the current airplane-mode/Nano firmware.** No code was altered for this import.

## Files

- `DueFlightComputerBase_v2_serial_actuator_test.ino` — untouched original temporary **replacement main tab** for Arduino Due.
- `serial_actuator_test_example_commands.txt` — untouched original Serial Monitor command examples.
- `README.md` — this compatibility and usage note, written for the repository import.

Source verification (Git blob SHA-1 of the exact original bytes, including final newline):

- Test sketch: `8b2cdb3df34e067d40a163eee8458499e5f2274e`
- Examples: `080145776f980723e2e2210730e21fcb79bf1f46`

## How it was intended to run

1. **Remove propellers and keep the aircraft restrained and actuators mechanically clear.** Treat this as a bench-only test.
2. Make a **separate local copy of the earlier 18-byte Due flight computer sketch folder**, including its matching support tabs. Do not use the repository's current 6-byte airplane-mode folder as though it were the old version.
3. In that throwaway copy, retain the original Arduino sketch folder name `DueFlightComputerBase_v2` and the main file name `DueFlightComputerBase_v2.ino`. Replace **only that local copy's main-file contents** with the exact contents of `DueFlightComputerBase_v2_serial_actuator_test.ino`. Do not place both files beside each other as `.ino` tabs: each defines `setup()` and `loop()`.
4. Compile for **Arduino Due** with the Due board package. The original test includes `Wire`, `SPI`, `LoRa` (Sandeep Mistry), `Servo`, `Adafruit_MPU6050`, and `Adafruit_Sensor`. Install their required dependencies.
5. Open Arduino Serial Monitor at **115200 baud**, select **Newline**, and use the companion examples file.
6. Stop the motor/ESC commands, disconnect actuator power safely, and flash the normal flight firmware again after testing.

**Do not paste the test into or overwrite the tracked files under `firmware/airplane-mode/`.** This experiment deliberately remains isolated.

## Required original companion firmware tabs

The `.ino` test file is **not self-contained**. It includes the original Due headers and calls functions/uses globals implemented in the original companion tabs:

| Original 18-byte Due file | Why the test needs it |
| --- | --- |
| `Config.h` | Due pin list, servo bounds, motor bounds, serial baud, LoRa settings, 18-byte packet definition |
| `TypesAndGlobals.h` | `rc`, `command`, `servos`, LoRa globals and telemetry packet buffer |
| `FunctionPrototypes.h` | Declarations for the flight computer's helper functions |
| `Servos.ino` | `setupServos()`, `centerAllServos()`, servo writes |
| `MotorDriver.ino` | `setupMotor()`, bidirectional `setMotorPower()` |
| `LoraTelemetry.ino` | `setupLoRaRadio()`, radio timeout, packet build/send for the **18-byte** frame |
| `CommandDemo.ino` | `initializeFlightCommand()` and `applyFlightCommand()` implementations; its `userFlightLogic()` is not called by the serial-test loop |

The complete matching old Due sketch also contains `RcReceiver.ino`, `ImuMpu6050.ino`, `EncoderAs5600.ino`, and `LoggingDebug.ino`. Preserve those tabs if using the original whole-sketch copy; the serial test does not start receiver capture, IMU, encoder or OpenLog logging. The older `CommandDemo.ino` may depend on symbols from the companion implementation tabs when compiled together.

These companion sources are **not copied into this experiment folder**. The two files requested for archival are present, but an independent build requires obtaining the matching 18-byte Due support sources separately. A separate, properly named sketch folder must be assembled for testing.

## Controls and configuration dependencies

- Due servos: `DS <index 0-6> <pulse_us>` or `DP <pin> <pulse_us>`. Original seven-output mapping: **0=39, 1=41, 2=43, 3=45, 4=47, 5=49, 6=51**.
- Due servo pulse values are clamped to `SERVO_MIN_US`/`SERVO_MAX_US` from that original `Config.h` (**1000–2000 µs** in the recovered version). `CENTER` uses `SERVO_CENTER_US` (**1500 µs**).
- Due bidirectional DC motor: `M <-255..255>` on the original `MOTOR_PIN_A`/`MOTOR_PIN_B` (**4/5**), with `MOTOR_MAX_POWER` and `MOTOR_DEADBAND` from `Config.h`.
- Nano servo values: `NS <1..4> <-127..127>` set `loraSignedValue1` through `loraSignedValue4`; their physical servo pulse widths and mapping are configured in the **matching older Nano firmware**, not in the Due test.
- Nano ESC values: `E <1..2> <0..255>` set `loraUnsignedValue1/2`; physical ESC endpoints are configured in the matching Nano firmware.
- `P` prints the commanded outputs; `CENTER` centers servos and stops motors; `SAFE`/`STOP` also sets the LoRa status byte to **255**; `?`/`HELP` prints help.
- This test sends the older **18-byte** LoRa packet every **20,000 µs (50 Hz)** and sets unused receiver values to 1500 µs. It does **not** read the RC receiver, update the IMU/encoder, or perform flight mixing.
- **Important:** Commands remain active until another serial command or reset; loss of the Serial Monitor connection does not automatically stop the motors. Radio receive timeout only helps if packets stop arriving. Use motor power isolation and test without propellers.

## Compatibility with the current repository baseline

**Not compatible as-is.** The live repository files under `firmware/` are a different airplane-mode generation:

| Concern | Original test and matching older support firmware | Current repository baseline |
| --- | --- | --- |
| Due servo outputs | 7: `39,41,43,45,47,49,51` | 6: `39,43,45,47,49,51`; **41 is directly receiver-driven elevator, not a Due servo output** |
| Due servo indexes | `DS 2` is pin 43 | In current config, index 2 is pin **45** |
| LoRa frame | **18 bytes**, signed values 1–4 and unsigned 1–2, plus attitude, RC, status and XOR | **6 bytes**, signed values 1–2 and unsigned 1–2, status and XOR |
| Nano spare servos | `NS 3`/`NS 4` transmitted in the old 18-byte frame | D3 and D5 are **locally centered**; signed values 3/4 are not transmitted |
| Nano ESCs | Old ESC command fields are bytes **4/5** | Current ESC command fields are bytes **2/3** |

Although the radio frequency and related LoRa settings may match, **packet length and field layout do not**. The current `NanoLoraServoReceiver_6Byte` rejects an 18-byte transmission as the wrong size. Replacing only the Due main tab while keeping current 6-byte `LoraTelemetry.ino`/`Config.h` would also change the test's behavior, remove the extra Nano servo commands, and invalidate its seven-servo pin documentation.

The current Nano config maps flaperons to **D4 and D6**, spare locally centered servos to **D3 and D5**, and ESCs to **D8 and D7**; its ESC input maps against a configured maximum of **254**, rather than the archived test's 255-range command assumption. The current airplane mode also uses pin **39** for tail unlock, **45** for the CH6 servo and reserves **43** for a planned H-tail folding servo. These are not the original V-tail assumptions. **Verify the assembled aircraft wiring before any actuation.**

No compatibility port, additional Nano firmware, changes to the current aircraft wiring, or modifications to the main firmware are included in this import.

## Safety and validation status

Archived source and static compatibility review only: **not compiled, not bench-tested, and not flight-tested in this import**. Confirm servo mechanical limits, motor polarity and ESC arming behavior with propellers removed. `SAFE` is a serial command, not an independent emergency stop or a guaranteed fail-safe.
