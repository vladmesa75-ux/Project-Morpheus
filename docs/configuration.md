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
| Servo endpoints and centers | `SERVO_MIN_US`/`SERVO_MAX_US` limits (500/2500 µs), `SERVO_CENTER_US`; fixed H-tail pin 43 at 2200 µs unfolded and existing pin 39/45 settings | `SERVO_MIN_US`/`SERVO_MAX_US` limits (500/2500 µs); per-servo `servoMinUs[]`, `servoCenterUs[]`, `servoMaxUs[]` calibrations unchanged |
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

- **H-tail folding servo:** physical Due pin **43** (`SERVO_PINS[1]`), commanded **2200 µs unfolded** for airplane startup, normal operation and RC failsafe. The five tracked helicopter/rotor tests separately command **1000 µs folded** on pin 43. No automated mode transition or fold-completion interlock is implemented. Verify geometry, pulse direction, end stops and clearance before powered testing.
- **Servo pulse limits:** Due and Nano servo outputs allow **500–2500 µs**. Existing per-servo Nano flaperon endpoint/center calibrations and ESC pulse ranges remain unchanged; a wider hard limit does not increase normal travel by itself.
- **Elevator:** direct RC control in the supplied design; physical pin 41 is intentionally *not* attached by the Due sketch.
- **Nano flaperon centers:** D4 = 1430 us and D6 = 1470 us in the supplied firmware. Its piecewise mapping gives each center independent meaning.
- **ESC input scaling:** Due can produce up to 255, while Nano's `UNSIGNED_INPUT_MAX` is 254. The Nano constrains larger values; do not alter this during import without a deliberate bench check.
- **Shared settings are copied, not linked** across the two `Config.h` files. A shared protocol schema or generated header is a future improvement, to be introduced with tests rather than a large initial rewrite.
