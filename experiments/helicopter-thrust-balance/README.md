# Helicopter motor-thrust balance (Due, existing Nano unchanged)

This standalone calibration sketch sends **two independently adjustable motor commands** over the aircraft's existing six-byte LoRa protocol. It is deliberately isolated from production airplane-mode firmware.

**Open in Arduino IDE:** [`Project_Morpheus_Helicopter_Thrust_Balance/Project_Morpheus_Helicopter_Thrust_Balance.ino`](Project_Morpheus_Helicopter_Thrust_Balance/Project_Morpheus_Helicopter_Thrust_Balance.ino). Keep the adjacent `Config.h` in the same sketch folder. Upload to the **Arduino Due**; keep the existing `NanoLoraServoReceiver_6Byte` firmware installed on the Nano. There is no Nano serial connection or reflashing step.

## What it commands

| Hardware | Command during test |
| --- | --- |
| Wing orientation | **Physically leave the wing oriented as in airplane mode** (the code does not flip or position it) |
| Due pin 45, wing lock | **850 µs, unlocked** (same as the existing helicopter V3 experiment; check physical travel) |
| Due pin 39, tail unlock | **1700 µs, unlocked** |
| Due pin 43, H-tail fold | **1000 µs, folded** (required for helicopter experiments) |
| Due pins 47, 49, 51 | 1500 µs, same neutral as the airplane-mode mixer |
| Nano D4 and D6, flaperons | Both at the currently calibrated **airplane neutral** (signed command 0) |
| Nano D8, left ESC | Motor byte 2, 0–255, with trim applied by the Due |
| Nano D7, right ESC | Motor byte 3, 0–255, with trim applied by the Due |
| CH3 receiver on Due D44 | 2000 µs = stop; 1000 µs = maximum (same direction as airplane mode) |

The Due transmits at up to 50 Hz using the same radio frequency, packet layout, XOR checksum, LoRa CRC, and ESC wiring as the airplane-mode firmware. The Nano still converts its two unsigned motor bytes to ESC pulse widths. The unused DC axle motor (Due D4/D5) is not commanded by this experiment.

## Calibration procedure

1. **First test with both propellers removed.** Verify servo motion, especially full wing unlock at 850 µs and folding at 1000 µs. Neither mechanism has position feedback. Verify that Nano D8 and D7 respond to the expected motor bytes and that stopping CH3 or switching off the transmitter returns both ESCs to minimum. Do not let the unlocked wing move unexpectedly during these checks.
2. With propulsion still disabled, arrange a **properly restrained test fixture** that permits observing any asymmetry without an uncontrolled wing spin, tip-over, or USB-cable entanglement. Verify clearance and the physical wing orientation.
3. Connect **only the Due** to Serial Monitor: **115200 baud**, newline enabled. With the transmitter on and CH3 fully off, wait at least 3 seconds for the servo-position settling interlock. `armed:YES` appears once a fresh, off-position CH3 pulse has been seen; increase CH3 gradually.
4. Observe the pull, then issue small steps such as `+1`, `+2`, `-1`, or an exact offset such as `SET 8`. **Positive trim increases the left (D8) command relative to the right (D7).** Use `STATUS` to see the commanded values and the *actual* difference.
5. Record the offset that best reduces the asymmetry, test it at multiple low/medium throttle points, and stop both motors by returning CH3 to OFF. Tell me the final signed number so I can apply it consistently to every active helicopter-mode mixer, while leaving airplane-mode mixer values unchanged.

Serial commands (one per line):

```text
HELP
STATUS
SET 10
SET -6
+1
-1
ZERO
```

Default trim is **0**, with a permitted serial range of **−100 to +100 command units**. Change `MAX_ABS_HELICOPTER_MOTOR_TRIM_UNITS` in local `Config.h` only after reassessing the test fixture. `SET` replaces the trim; `+N` and `-N` change it relative to the current trim. For example, at base command 120 and `SET 10`, the Due transmits **left = 125, right = 115**. At zero CH3 throttle it transmits **0, 0** regardless of trim. Near 0 or 255, output clamping can reduce the actual difference; the Serial Monitor displays it.

**Important:** trim units are **0–255 command-byte units**, **not ESC pulse microseconds** and not measured thrust. The Nano currently maps 0–254 to its ESC endpoints and caps 255 at its maximum. This trim therefore provides roughly 4 µs command resolution in the middle of the span, subject to ESC calibration and integer rounding.

## Failsafe and limitations

- Without a fresh CH3 RC pulse for 100 ms, both motor bytes go to zero and the throttle must be returned to OFF before the test will re-arm.
- No motor output is allowed until CH3 was observed OFF after the three-second startup settle interval; this is a time delay, **not** physical confirmation that folding or unlocking has completed.
- The Nano's existing 500 ms loss-of-LoRa failsafe still applies. If LoRa initialization fails, the Due does not transmit and the Nano stays in or enters its failsafe.
- The old CH6 switch is not used here: the wing remains **unlocked** throughout this experiment. No automatic airplane-to-helicopter transition, wing orientation change, flaperon collective/cyclic, or axle motor PID is used.
- A value entered in Serial Monitor is **not retained after reboot**. The default for this experiment is `INITIAL_HELICOPTER_MOTOR_TRIM_UNITS` in local `Config.h`. Once calibration is finalized, transfer the final signed offset into the **active helicopter-mode code paths only**. Existing historical experiments have independent configuration and will not magically inherit this new value.
- This is a static thrust-balance aid, not a flight controller. Pulse-command equality is not proof of equal thrust; propeller, ESC, voltage, aerodynamic, and fixture effects can all influence the observed torque.

The code is intended for ground testing only and must not replace the flight-ready airplane-mode `DueFlightComputerBase_v2` sketch.
