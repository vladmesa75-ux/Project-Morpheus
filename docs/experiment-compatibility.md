# Experimental sketch compatibility audit

**Reviewed 5 November 2026.** The reference configuration is the supplied airplane-mode Arduino Due `Config.h` and six-byte Nano LoRa receiver `Config.h`. Findings are from static source review, **not** a fresh compile, hardware verification or flight test. Historical sketches are archived without modifying their behavior.

## Current configuration reference

- **Due inputs:** RC channels 1–6 on pins **48, 46, 44, 42, 40, 38**.
- **Due motor driver:** bidirectional H-bridge PWM input pins **4 and 5** (not one PWM plus a digital direction pin).
- **Due sensors:** MPU6050 on `Wire` at `0x68`, AS5600 on `Wire1` at `0x36`; both I2C buses at 400 kHz.
- **Due servo outputs:** 39, **43**, 45, 47, 49, 51. Airplane-mode CH6 controls pin **45** using **1300 and 1950 µs**. The new H-tail folding servo uses **pin 43**, but folding control is **not yet implemented**.
- **Nano radio protocol:** 6-byte packet: signed left/right flaperons in bytes 0/1; unsigned two ESC throttle commands in bytes 2/3; byte 4 status; byte 5 XOR checksum. 433 MHz, SF7, 500 kHz bandwidth, code rate 4/5, LoRa hardware CRC enabled.
- **Encoder gearing:** 1.34375 encoder revolutions per real wing revolution; tests that measure actual wing RPM should apply this ratio.

## Imported experimental code

| Experiment | Hardware comparison | Radio protocol | Action before reuse |
| --- | --- | --- | --- |
| Yaw PIDF original | MPU6050, AS5600, DC motor 4/5, wing unlock servo 45 | Not applicable | **850 µs wing unlock at startup**, rather than airplane-mode CH6 endpoints; verify mechanics |
| Yaw PIDF updated | Same pins and I2C; includes correct 1.34375:1 gearing and OPEN/PARTIAL/LOCK commands | Not applicable | Lock pulses **850 / 1300 / 1950 µs**; startup commands OPEN; motor disabled until unlocked. PIDF starts at zero |
| Helicopter manual V1 | RC pitch 46 and throttle 44 match; unlock pin 45 at 850 µs | **Mismatch — 18 bytes** | Keep as historical code; not compatible with the current Nano |
| Helicopter manual V2 | Same RC and servo pins as V1 | **Mismatch — 18 bytes** | Keep as historical code; not compatible with the current Nano |
| Helicopter manual V3 6-byte | Same RC/sensor/pin reference; 1.34375:1 encoder gearing | **Matches 6-byte layout, packet XOR, hardware CRC and radio settings** | Verify 850 µs servo motion. Byte 4 uses `0` for normal and `255` for failsafe, whereas airplane mode normally uses CH5 position; Nano does not act on byte 4 |

**Critical difference:** The older experiments explicitly command the wing-lock servo on pin **45** to **850 µs**, outside the main Due sketch's generic **1000 µs** lower servo bound. This may have been appropriate for the older mechanical setup but must not be assumed safe for the present mechanism.

The added H-tail fold servo on pin **43** is not actuated by the imported experiments. The required fold positions, power-up state, failsafe behavior and interlock remain to be designed and checked.

## Additional original code located; full-file import still pending

The following sketches were initially readable and inspected, but subsequent file retrieval failed. These are **not** committed yet, and no replacements were invented.

| Experiment | Observed configuration issue |
| --- | --- |
| `Project_Morpheus_Wing_RPM_PID_Test.ino` | **18-byte** LoRa protocol, incompatible with current Nano |
| `Project_Morpheus_Wing_RPM_PID_Test_Serial_Plotter.ino` | **18-byte** LoRa protocol |
| `Project_Morpheus_Wing_RPM_PID_Unified.ino` | **18-byte** LoRa protocol; supports local motor selection |
| `Project_Morpheus_Wing_RPM_PID_Unified_Failsafe_Toggle.ino` | **18-byte** LoRa protocol; review selectable failsafe behavior |
| `Project_Morpheus_Wing_RPM_PID_No_Failsafes.ino` | **18-byte** LoRa protocol; **deliberately has no automatic failsafes** |
| `Project_Morpheus_Local_DC_Motor_RPM_PID_Test.ino` | **Pin-mode mismatch:** assumes PWM pin 4 and direction pin 5, while current H-bridge needs two PWM inputs |
| `Project_Morpheus_Local_DC_RPM_PID_SAFE_v2.ino` | Corrected two-PWM motor driver pins 4/5; wing unlock servo 45 at 850 µs |
| `Emergency_Local_DC_Motor_Stop.ino` | Drives motor pins 4 and 5 to zero; test-only emergency stop sketch |
| `DueFlightComputerBase_v2_serial_actuator_test.ino` | Replacement main tab requiring the rest of the Due project; not independently compilable |

Other mentioned but not fully recovered: standalone AS5600 serial test, `NanoSerialOutputTest.zip`, a yaw PIDF duplicate variant and an unnamed Arduino sketch.

### How to use the audit

- Never fix the old 18-byte packet simply by changing the size constant; the **whole packet layout** differs.
- Do not run a historical `No_Failsafes` sketch on an aircraft before adding appropriate safety protections.
- The standalone experimental sketches store their own settings; **changes to flight-mode `Config.h` do not update them**.
- Preserve original test behavior. Future compatibility ports should be committed as separate variants with their own `Config.h`, a clear version and bench-test notes.
- Remove propellers for initial powered tests and check actuator travel, lock interlocks and failsafe behavior before any flight.
