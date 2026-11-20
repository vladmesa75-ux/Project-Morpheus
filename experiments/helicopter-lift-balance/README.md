# Helicopter aerodynamic lift-balance test

This **Due-only** test adds serial-adjustable differential collective to the existing manual helicopter control idea. It uses the aircraft's **unchanged six-byte Nano receiver**; no new Nano program and no Nano serial connection are required.

Open [`Project_Morpheus_Helicopter_Lift_Balance.ino`](Project_Morpheus_Helicopter_Lift_Balance/Project_Morpheus_Helicopter_Lift_Balance.ino) in Arduino IDE, keeping the adjacent `Config.h` in the same sketch directory. Upload it to the Arduino **Due**, not the Nano.

## Configuration and controls

| Input / actuator | Behavior |
| --- | --- |
| RC CH3, Due pin 44 | Direct throttle to **both** motors equally, 0–255; 2000 µs OFF, 1000 µs FULL |
| RC CH2, Due pin 46 | Collective for both flaperons, signed command −127 to +127 |
| Nano D4, left flaperon | Helicopter V3 direction `-1` (left wing physically flipped 180°) plus half of trim |
| Nano D6, right flaperon | Helicopter V3 direction `-1` minus half of trim |
| Nano D8 and D7 ESCs | **Exactly the same motor byte**; no motor trim |
| Due pin 45 | Wing **unlocked**, 850 µs |
| Due pin 39 | Tail **locked**, 1000 µs (only unlock during landing with a different controller) |
| Due pin 43 | H-tail **folded**, 1000 µs |
| Due pins 47, 49, 51 | 1500 µs neutral |

The left wing must already be **mechanically flipped into helicopter configuration**. The sketch does not control a wing-flip actuator. CH6 is ignored and there is no cyclic control, axle motor PID, or automatic transition.

The serial trim is a **difference in signed Nano flaperon commands**, not pulse-width microseconds, deflection degrees, or measured lift. Positive trim means the *left command byte decoded as signed* is greater than the right command. The actual increase/decrease in lift must be established on the aircraft because the servo calibrations and blade geometry are not identical.

For example, if CH2 requests `collective = +20`, both baseline Nano commands are `−20`. With `SET 5`, the Due sends **left = −18** and **right = −23**. Both motors still receive the same CH3 value. Odd differences work; the integer half-unit is allocated to one side. At ±127 command saturation, the actual difference may be smaller than requested.

## Serial monitor

Open the **Due's** Serial Monitor at **115200 baud** with newline enabled:

```text
HELP
STATUS
SET 5
SET -5
+1
-1
ZERO
```

`SET N` sets the total left-minus-right difference; `+N` and `-N` nudge the existing difference. `STATUS` prints CH2 and CH3 pulse widths, receiver freshness, arm state, motors, collective, flap trim, left/right commands, and their actual difference.

The initial trim is `0`. The serial-adjustable limit is ±40 signed units, controlled by `MAX_ABS_HELI_FLAPERON_TRIM_UNITS` in the local `Config.h`. Trim is **temporary** until the default `INITIAL_HELI_FLAPERON_TRIM_UNITS` is changed and the sketch reuploaded.

If your actual receiver's full-off CH3 pulse is not 2000 µs, change **only** `RC_THROTTLE_OFF_US` to the observed value; the arm threshold automatically follows, 15 µs below that value. Do not change the calibration without confirming motors still command zero at OFF.

## Ground test sequence

1. **Remove propellers for the first tests.** Confirm pin 39 remains locked, pin 43 is folded, and pin 45 has fully released the wing before operating anything that could spin it. These are open-loop servo commands, not feedback that a mechanism completed its motion.
2. Verify mechanically that the **left wing is in helicopter orientation**. Move CH2 through a small range and confirm both flaperons change *physical blade pitch in the intended collective direction*. Their Nano pulse calibrations are different; matching signed commands is not proof of matching lift.
3. With CH3 at OFF, power on and wait 3 seconds. The test automatically arms once **both CH2 and CH3 signals are fresh** and CH3 is at or very near OFF (`armed:YES`). It resets the arm latch on RC loss; CH3 must return to OFF to rearm.
4. After correct servo travel, motor shutdown, and LoRa-loss tests with propellers removed, use a suitable restrained ground test fixture for low-power rotor tests. Increase CH3 slowly to a repeatable RPM and vary CH2 gently to introduce collective.
5. At comparable RPM and CH2, adjust trim in **small steps** and observe the rotor's repeatable 1/rev tracking/vibration response. As a practical caution, vibration may also reflect residual mass imbalance, blade tracking, structural resonance, or mounting effects; this test **does not itself measure lift**.
6. Record the signed trim that minimizes the imbalance across several repeatable conditions. Once found, it can be applied to the **active helicopter-mode collective mixers only**, not the airplane mixer.

Failsafe behavior: if either CH2 or CH3 stops providing fresh pulses for 100 ms, the Due transmits motor command 0 and neutral flaperon commands. The Nano additionally stops the ESC outputs if valid LoRa packets are absent for 500 ms. If the Due's LoRa fails to initialize, the Nano's own failsafe applies. The code has no physical mechanism interlock or independent lift sensor; this is a **ground calibration sketch, not a flight controller**.
