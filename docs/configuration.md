# Configuration guide

The imported firmware already uses one `Config.h` per controller. Keep configuration data here rather than scattering raw numbers across implementation files. This is the **current arrangement**, not yet a single cross-board generated configuration system.

## Edit these files

- **Arduino Due:** [`firmware/airplane-mode/DueFlightComputerBase_v2/Config.h`](../firmware/airplane-mode/DueFlightComputerBase_v2/Config.h)
- **Arduino Nano:** [`firmware/nano-receiver/NanoLoraServoReceiver_6Byte/Config.h`](../firmware/nano-receiver/NanoLoraServoReceiver_6Byte/Config.h)
- **Experimental tests:** each future test gets its *own* clearly named local config; do not import flight-ready calibration into an experiment implicitly.

## Where the settings are

| Setting | Due | Nano |
| --- | --- | --- |
| Arduino pin assignments | `MOTOR_PIN_*`, `SERVO_PINS`, `RC_PINS`, `LORA_*_PIN` | `SERVO_PINS`, `ESC_PINS`, `LORA_*_PIN` |
| Servo endpoints and centers | `SERVO_MIN_US`, `SERVO_CENTER_US`, `SERVO_MAX_US`; individual pin 39 and 45 position settings | `servoMinUs[]`, `servoCenterUs[]`, `servoMaxUs[]` |
| ESCs / motor output | `MOTOR_*`, throttle RC mapping, differential thrust gains, motor scale/trim | `escMinUs[]`, `escMaxUs[]`, `UNSIGNED_INPUT_MAX` |
| RC channels and switch thresholds | `RC_PINS`, `RC_*_US`, `CH6_POSITION_2_MIN_US`, CH5 thresholds | Not read directly by Nano |
| IMU / filter | `MPU6050_ADDRESS`, calibration, deadband, attitude alpha, I2C speed | No IMU in this sketch |
| AS5600 encoder | `AS5600_*`, I2C1 speed | No encoder in this sketch |
| LoRa | Frequency, spreading factor, bandwidth, coding rate, power, packet bytes, TX timeout | Frequency, spreading factor, bandwidth, coding rate, CRC, packet size |
| Logging | `ENABLE_USB_DEBUG`, `ENABLE_OPENLOG`, respective intervals / baud rates | `SERIAL_PRINT_INTERVAL_MS` |
| Failsafe | RC validity/timeout; actuator-safe positions in the command mixer | `FAILSAFE_TIMEOUT_MS` and centered servo/min-ESC outputs |
| Flaperon tuning | `FLAPERON_*`, `FLAP_SETTING_*` | Actual pulse calibration in the per-servo min/center/max arrays |
| PID | **No active airplane-mode PID calibration block in the supplied main sketch.** Future controllers should group their gains by axis/mode in configuration files. | No PID in this sketch. |

## How to make a safe change

1. Record the previous values (Git history does this automatically once committed).
2. Change **one** setting or tightly related set at a time, preferably in `Config.h`.
3. For a LoRa or packet change, update **both** boards together and run `python tools/check_config.py`.
4. Compile for the correct board. For control changes, first do motor-disconnected/propeller-free bench checks.
5. Verify neutral, full positive/negative travel, reversed directions, switch states, and signal-loss behavior.
6. Commit with a descriptive message such as `Tune Nano D4 flaperon center to 1430 us`.

## Current important exceptions

- **New H-tail folding servo:** physical Due pin **43**. The imported Due firmware already *attaches* servo output 43 (`SERVO_PINS[1]`) but currently drives it to generic center. There is no fold/unfold command, endpoint tuning or verified failsafe yet. **Do not connect/operate the new folding mechanism using this unmodified code without checking mechanical clearance.** Implement separately after we decide the control source, direction, travel limits, timing/interlocks, and safe state.
- **Elevator:** direct RC control in the supplied design; physical pin 41 is intentionally *not* attached by the Due sketch.
- **Nano flaperon centers:** D4 = 1430 us and D6 = 1470 us in the supplied firmware. Its piecewise mapping gives each center independent meaning.
- **ESC input scaling:** Due can produce up to 255, while Nano's `UNSIGNED_INPUT_MAX` is 254. The Nano constrains larger values; do not alter this during import without a deliberate bench check.
- **Shared settings are copied, not linked** across the two `Config.h` files. A shared protocol schema or generated header is a future improvement, to be introduced with tests rather than a large initial rewrite.
