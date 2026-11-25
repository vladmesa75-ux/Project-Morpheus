# Morpheus helicopter firmware

The Arduino Due commands the two wing ESCs over LoRa to control rotor RPM. A separate PIDF drives the bidirectional axle motor to control fuselage yaw. CH1 and CH2 give the pilot direct cyclic. Roll and pitch auto-level and automatic airplane/helicopter transitions are future work.

Open [MorpheusHelicopter.ino](MorpheusHelicopter/MorpheusHelicopter.ino) for the main firmware. The flight and tuning sketches use the same [Config.h](MorpheusHelicopter/Config.h), [RPM governor](MorpheusHelicopter/RpmGovernor.cpp), [yaw controller](MorpheusHelicopter/YawControl.cpp), [collective](MorpheusHelicopter/Collective.cpp), [cyclic](MorpheusHelicopter/Cyclic.cpp), hardware drivers, mixer and interlocks. The two tuning sketches select different control modes at startup.

## Which sketch to open

| Sketch | Wing motors | CH3 | CH4 and axle motor | CH1 / CH2 |
| --- | --- | --- | --- | --- |
| [Helicopter flight](MorpheusHelicopter/MorpheusHelicopter.ino) | RPM governor by default | Collective | Bounded heading target | Roll / pitch cyclic |
| [RPM tuning](MorpheusHelicopter/examples/RpmTuning/RpmTuning.ino) | RPM PIDF | Collective | Off | Off |
| [Yaw tuning](MorpheusHelicopter/examples/YawTuning/YawTuning.ino) | Direct manual thrust | Common wing ESC command | Yaw PIDF | Off, collective fixed |

Edit the `flight` entry in `Config.h` to enable each function. For manual thrust select `RotorMode::Manual` and `CollectiveMode::Fixed` together. Select `YawMode::Off` for zero axle power. The yaw tuning sketch uses the configured yaw mode, or heading mode if the flight setting is off. CH5 is spare. The directly connected elevator still follows CH2.

**Initial values are for commissioning.** Both PIDF controllers ship with zero gains and feedforward. CH6 reports `untuned` if a selected controller has no coefficients. Gain values can be tried on a tuning sketch with CH6 off. Transfer the measured coefficients to the shared configuration and rebuild the flight sketch. The updated Nano also requires two complete measured servo calibration rows before it accepts motor or surface commands. [Nano bench procedure and receiver](../nano-receiver/README.md)

## Configuration

| Group | Parameters and interpretation |
| --- | --- |
| RC and CH6 | Each stick has a channel, physical low/center/high pulses and deadband. Swap the low and high pulses to reverse an axis. Set CH6 channel, polarity and switch thresholds. Governed startup supports low, center or custom CH3 pulse and a tolerance. Initial selection is center, 1500 ±35 µs. Manual thrust starts at throttle-off, 2000 ±15 µs. |
| Collective | Low/center/high angles initially 0° / +5° / +10°. Fixed collective initially +5°. Physical stick travel is reversed electrically on CH3. Low = 2000 µs, center = 1500 µs, high = 1000 µs. |
| Cyclic | Separate enable, center trim and positive/negative travel per axis. Initial travel ±5° each. Geometric phase offset in degrees and measured command-to-motion delay in ms are independent. Cyclic enables above 40 RPM and drops out below 30 RPM initially. |
| Wings | Set min, max, lift trim, stopped angle, fault angle and signed Nano direction for each wing. The outer calibrated range is ±20°. Each surface is clipped independently after collective, cyclic and lift trim. A request of +25° with a +20° limit produces +20°. |
| RPM PIDF | PID gains, integral cap in ESC units, derivative filter time, update interval, target, target slew rate, feedforward per RPM and feedforward per RPM per degree of collective deviation from center. Initial target 50 RPM, ramp 10 RPM/s, governed ESC cap 64/255 and command rise limit 25 units/s. Separate manual ESC cap and per-wing trim/scale. |
| Rotor supervision | Overspeed threshold, minimum running RPM, spool-up deadline and sustained low-RPM timeout. Initially 75 RPM, 20 RPM, 12 s and 2 s. The unfiltered RPM window and filtered RPM both feed overspeed detection. Change the target and its operating envelope together. |
| Yaw PIDF | Heading and rate modes have independent gain sets, integral caps and derivative filters. CH4 has a center deadband, response curve, max target rate (initially 30°/s) and max target lead (initially 3°). Configure axle polarity, output cap, deadband, slew, update interval, constant bias, rotor-RPM feedforward and reference-rate feedforward. |
| Sensors | Encoder ratio/sign/zero trim, sample interval, plausible motion, stale window, RPM measurement window/filter/deadband. IMU ranges, bandwidth, bus speed, gyro calibration, bias, yaw sign and complementary-filter time constant. Yaw is relative and uses the body Z gyro. Drift and bank angle affect its accuracy. |
| Hardware and logging | Due RC pins, six servo pins and mechanism pulses, servo bounds, H-bridge pins, I²C addresses and clocks, radio pins/settings/TX timeout and optional minimum transmit spacing, serial rates, views, intervals and field filters. All six receiver channels must supply fresh pulses. |

### Rotor phase and surface direction

At power-up the fuselage must be still for gyro calibration and the right wingtip must be at aircraft right for indexing. The geared AS5600 cannot resolve the aircraft's physical zero by itself. The firmware accumulates signed encoder increments across 4095/0, then divides by **1.34375 encoder turns per rotor turn**. A counterclockwise hand turn viewed from above should give right = 0°, nose = 90°, left = 180°, tail = 270°. Implausible jumps are rejected. Loss of phase for the configured stale interval latches until a reboot at the known alignment.

At zero phase correction, positive roll adds cyclic to the left wing when the right wing is at aircraft right. Positive pitch adds cyclic to the left wing when the right wing is at the nose. The opposite wing receives the opposite cyclic term. Confirm transmitter polarity, measured surface motion, and the physical rotor response before enabling both axes. The initial zero phase and delay are placeholders. The mixer predicts phase at packet construction using elapsed time since the last encoder sample plus the configured delay.

The two signed LoRa bytes are calibrated flaperon commands. The initial helicopter direction signs are −1/−1 from the earlier lift test. If the measured Nano rows already define positive deflection in the helicopter lift convention, set the corresponding Due signs to +1. Verify this once per wing. For control verification compare the logged requested angle, clamped angle and signed byte with the physical angle gauge.

### Yaw target behavior

On enable the controller seeds the target from the measured fuselage heading. CH4 advances that target at the requested rate. With a 3° lead limit, actual heading 40° and continued positive stick, the target stops at 43° until the fuselage follows. Centered stick holds the last target. Opposite stick moves it back immediately at the commanded rate. The limiter uses signed circular error across 0°/360° and applies only to stick-driven increments. A disturbance after releasing CH4 leaves the original heading target in place. In heading mode D uses measured yaw rate and the optional target-rate feedforward uses the movement actually admitted by the lead limiter.

## Starting and stopping

1. Begin with propulsion disconnected. Set the mechanical helicopter orientation and verify the first outputs. Pin 39 holds the tail locked at 1000 µs, pin 43 holds the H-tail folded at 1000 µs, pin 45 releases the wing at 900 µs. Pins 47, 49 and 51 hold 1500 µs. These mechanisms have open-loop servo commands. Verify physical completion on the aircraft.
2. With CH6 off, inspect the receiver, IMU, encoder, radio transmission and configuration in Serial Monitor. Check encoder phase at all four positions. Complete the Nano calibration and verify the actual direction of both surfaces and axle motor separately.
3. Set CH3 to the configured start position and deliberately switch CH6 on after the mechanism settling interval. The governor starts with zero drive and a ramped target. A failed start requires an off/on cycle. A running manual-thrust profile follows CH3 directly.
4. CH6 off cuts the Due axle immediately and queues zero wing drive in the next available packet. The wing remains released as it coasts. RC loss, encoder loss, radio TX failure, control-loop overrun, overspeed, and governor run-up or sustained underspeed faults also stop the run. Stale IMU data stops a profile that uses yaw. The Due requires a fresh CH6 off/on sequence after a fault.

The radio is one-way. The Due reports **completed local transmissions**. A receiver-only dropout is invisible to the Due. The Nano currently holds the last command for up to **500 ms** after the last valid packet, then commands ESC minimum and measured surface neutral. It resumes on valid packets. At 240 RPM that timeout spans two rotor turns. Measure actual receiver packet gaps, pulse timing and actuator lag on the bench before powered cyclic testing. Due shutdown cannot clear a packet already in flight. Receiver failsafe programming also matters if the RC receiver continues generating valid-looking held PWM after a transmitter loss.

## Serial tuning and telemetry

Use the Due programming USB port at **115200 baud**, newline line ending. Commands are case-insensitive.

| Command | Effect |
| --- | --- |
| `VIEW STATUS`, `VIEW RPM`, `VIEW YAW`, `VIEW CYCLIC`, `VIEW ALL` | Change the USB view without changing flight behavior. RPM, yaw and cyclic views emit labeled Serial Plotter values. |
| `GAINS` | Show the current tuning coefficients. The RPM example shows governor gains. The yaw example shows the selected yaw gains. |
| `SET P value`, `SET I value`, `SET D value` | Set the selected PID gains in RAM on the appropriate tuning sketch with CH6 off. |
| `SET FR value`, `SET FC value` | The RPM sketch sets thrust per target RPM, and thrust per target RPM per degree of collective deviation from center. |
| `SET F0 value`, `SET FR value`, `SET FV value` | The yaw sketch sets axle bias, axle units per measured rotor RPM, axle units per admitted target °/s. |

Configuration-file gain units use seconds, degrees, °/s and rotor RPM. Controller output is in ESC command units or signed axle PWM units. The Due logs commanded thrust and axle PWM. There are no thrust, torque, electrical power or surface-position sensors in this setup.

OpenLog receives timestamped CSV on Serial2. Its default `All` view includes six raw RC inputs, states, fault reason, RPM target/ramp/measurement, both controllers' P/I/D/F terms, both ESC bytes, signed axle PWM, measured attitude/body rates, target/measured yaw, lead limiting, phase, requested/clipped flaperon angles and signed commands. Set `usbFields` or `openLogFields` to comma-separated field names from the CSV header to narrow a view. `log_skips` counts samples skipped when the previous row has yet to drain. State codes 0–4 mean waiting for off, off, spooling, running and faulted. Fault codes follow the `Fault` enum in [Control.h](MorpheusHelicopter/Control.h), with their names in the USB status view.

## Building

Install Arduino SAM Boards for the Due and the Servo, LoRa (Sandeep Mistry) and Adafruit MPU6050 libraries with their dependencies. In Arduino IDE, open the flight sketch directly. For the tuning examples, install or symlink the **same** `MorpheusHelicopter` directory under the sketchbook's `libraries` folder and open its examples. Edit the single installed/source copy of `Config.h` used by all three builds. The CLI can use the repository folder as a library directly.

Run these from the repository root.

```sh
heli_dir=firmware/helicopter-mode/MorpheusHelicopter
arduino-cli compile --fqbn arduino:sam:arduino_due_x "$heli_dir"
arduino-cli compile --fqbn arduino:sam:arduino_due_x --library "$heli_dir" "$heli_dir/examples/RpmTuning"
arduino-cli compile --fqbn arduino:sam:arduino_due_x --library "$heli_dir" "$heli_dir/examples/YawTuning"
firmware/helicopter-mode/tests/run.sh
```

The native tests exercise the same control tabs and the complete flight loop with simulated time and hardware inputs. They cover encoder wrap and gearing, per-wing limits, packet layout, lead-limited yaw and reversal, PIDF saturation, arming, shutdown and sensor faults. Bench measurements are still required for controller gains, actuator direction, cyclic phase and radio timing.
