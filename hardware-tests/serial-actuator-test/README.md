# Serial actuator test – Arduino Due and Nano

This test allows **direct actuator commands from Arduino IDE's Serial Monitor**, with the Arduino Due connected by USB. It uses the original, recovered `DueFlightComputerBase_v2_serial_actuator_test.ino`, stored **unchanged**, and copies of the corresponding current airplane-mode support tabs so that the test is in a separate Arduino sketch folder.

## Open the test

Open this **main sketch file** in the Arduino IDE:

[`DueFlightComputerBase_v2_serial_actuator_test/DueFlightComputerBase_v2_serial_actuator_test.ino`](DueFlightComputerBase_v2_serial_actuator_test/DueFlightComputerBase_v2_serial_actuator_test.ino)

Select the **Arduino Due** board and its correct USB port. Open **Serial Monitor** at **115200 baud** with the line ending set to **Newline**. The required support files are already in the same sketch folder:

- `Config.h`, `TypesAndGlobals.h`, `FunctionPrototypes.h`
- `CommandDemo.ino` (command state / output application)
- `Servos.ino` and `MotorDriver.ino` (Due hardware outputs)
- `LoraTelemetry.ino` (current compact 6-byte LoRa packet transmitter)

They were copied from the existing `firmware/airplane-mode/DueFlightComputerBase_v2/` folder. **Do not also copy the production `DueFlightComputerBase_v2.ino` main tab** into this test folder: both mains define `setup()` and `loop()`.

The test does not require `RcReceiver.ino`, `ImuMpu6050.ino`, `EncoderAs5600.ino`, or `LoggingDebug.ino` to run its Serial commands. It does, however, retain the original Due header includes, so install the same Arduino Due board support and libraries used for the airplane-mode firmware (including `LoRa`, `Servo`, and Adafruit MPU6050 dependencies). **This test copy has not been freshly compiled or tested on the assembled hardware.**

## Supported Serial Monitor commands

Send **one command per line**, with no comments or leading `>` symbol.

| Example | Actual behavior with current Config.h |
| --- | --- |
| `?` or `HELP` | Print the original command help (contains outdated pin/index information) |
| `P` | Print current output commands |
| `DP 43 1500` | Set Due pin 43 to 1500 µs; verify tail-fold linkage first |
| `DP 39 1800` | Set Due tail unlock servo on pin 39 to 1800 µs |
| `DP 45 1300` | Set Due CH6-associated servo on pin 45 to 1300 µs |
| `DS 1 1500` | Set Due servo **index 1**, which is **pin 43** |
| `M 20`, `M -20`, `M 0` | Set local bidirectional DC motor PWM command (-255 to +255; zero to stop) |
| `NS 1 10`, `NS 1 0` | Move/neutralize Nano **D4** flaperon over LoRa (-127 to +127) |
| `NS 2 10`, `NS 2 0` | Move/neutralize Nano **D6** flaperon over LoRa |
| `E 1 10`, `E 1 0` | Command Nano ESC 1 (**D8**) in unsigned units |
| `E 2 10`, `E 2 0` | Command Nano ESC 2 (**D7**) |
| `CENTER` | Generic center pulse for Due servos; stop motors and reset Nano commands |
| `SAFE` or `STOP` | Generic center pulse for Due servos, zero motor/ESC commands, and send status byte 255 |

**Start with `P` and inspect the outputs before sending movement commands.** ESC commands are raw values, not percentages.

## Pin assignments of the current configuration

The local `Config.h` includes six Due servo outputs, in this **actual index order**:

| DS index | Due pin | Current airplane-mode role |
| --- | --- | --- |
| 0 | 39 | Tail unlocking servo |
| 1 | 43 | Newly installed H-tail folding servo; no calibrated fold positions yet |
| 2 | 45 | CH6-controlled existing servo |
| 3 | 47 | Attached servo output |
| 4 | 49 | Attached servo output |
| 5 | 51 | Attached servo output |

The **original help text mistakenly says indices 0–6 and includes pin 41**. That was from an older setup. With the supplied current configuration, **index 6 and pin 41 are not valid**. `DS 2` controls **pin 45**, not pin 43, even though one old source-code comment says otherwise.

The current Nano LoRa receiver uses a **6-byte** packet. It receives only signed flaperon controls 1–2 and ESC controls 1–2:

- `NS 1` → Nano D4; `NS 2` → Nano D6.
- `NS 3` and `NS 4` can be entered into the original test, but **do not move Nano D3/D5**: those commands are not transmitted in the current six-byte protocol. The Nano holds both outputs at their configured center pulses.
- `E 1` → Nano D8; `E 2` → Nano D7.
- The test sends commands at approximately 50 Hz with the same SX1278 configuration as the current airplane-mode transmitter. Nano LoRa receiver firmware must be installed and powered for Nano commands to work.


Before testing after any firmware updates, compare the local test snapshot with the airplane-mode support tabs:

```bash
python tools/check_hardware_test_snapshot.py
```

This reports changed copies; it **does not automatically overwrite** deliberately different test configurations.

## Critical safety limitations

**This is a manual test and has no automatic serial-command-loss timeout.** A motor/ESC command will remain active until another Serial command is processed, another safety system intervenes, or power is removed. Simply disconnecting the USB cable **does not guarantee a stop**. Keep an independent physical power cutoff within reach.

`SAFE`, `STOP` and `CENTER` drive all Due servos to the **generic `SERVO_CENTER_US = 1500` pulse**. They are not guaranteed to put the tail locks or the new folding stabilizer into mechanically safe positions. Furthermore, the initial setup briefly centers all Due servos before applying the initialization values. **Do not attach the uncalibrated pin-43 folding mechanism until its movement and clearance have been checked.**

The original test's `SAFE` command transmits special byte 255, but the current Nano firmware does not use that status byte as a separate immediate failsafe trigger. It does transmit zero throttle and neutral flaperon commands. Do not confuse this with a hardware emergency-stop circuit.

The Nano uses its own 500 ms **LoRa packet-loss** timeout, but the Due-local motor and servos have no equivalent Serial-command timeout in this test.

## Configuration ownership

Edit the **local** `DueFlightComputerBase_v2_serial_actuator_test/Config.h` to adjust test pin assignments and servo limits. This is a copy of the currently imported airplane-mode configuration, **not a live shared header**. Future updates to `firmware/airplane-mode/.../Config.h` will not automatically change the test. Compare both before bench testing.

The actual original test `.ino` is preserved for historical traceability. A future revision with per-actuator limits, an explicit serial watchdog, and corrected help text should be created as **a new test version** rather than silently rewriting it.
