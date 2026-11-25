# RPM PIDF tuning on the Due

This sketch runs the helicopter firmware’s rotor governor. CH6 starts or stops it. CH3 changes collective so you can apply a repeatable load. Cyclic and axle motor output stay at zero. The Nano receives the usual six-byte packet.

## First run

1. Build the RPM tuning example using the [shared helicopter firmware](../../../README.md#building). Open the Due programming port in Serial Monitor at **115200 baud**, with **Newline** selected.
2. Power up with the wing at its known encoder zero. With propulsion disconnected, verify the released wing, locked and folded tail, positive RPM direction and calibrated Nano flaperon movement. The Nano needs both completed servo calibration rows to command its ESCs.
3. Hold CH6 off. Enter `STREAM OFF` if you want a quiet screen, then `STATUS` and `GAINS`. Check receiver freshness, encoder indexing and the CH3 startup position. The default is **1500 ±35 µs**. The initial target is **50 rotor RPM** and the governor cap is **64 ESC units out of 255**.
4. If you have a measured steady operating point from manual testing at center collective, enter `FFEST <common ESC command> <measured rotor RPM>`. It prints an estimate for FR. Enter `SET FR <estimate>` to apply it. With P, I and D zero, this lets you observe feedforward alone. The zero-gain configuration reports `untuned` until you supply a coefficient. Angle-dependent FC contributes zero at center collective.
5. For a restrained run, enter `MONITOR` to resume the readable stream. Set CH3 to its configured start position and switch CH6 on. Watch measured RPM follow the ramp. Switch CH6 off before changing a coefficient. Use the same starting collective to compare runs and then vary CH3 to examine load response.
6. Enter `REPORT` after stopping. Once satisfied, enter `EXPORT` and replace the two matching declarations in the flight firmware's shared `Config.h`. All serial changes live in RAM and reset on power-up. Target and output cap are printed as comments for you to review separately.

## Serial commands

| Command | Result |
| --- | --- |
| `HELP` | Show the command list again. |
| `STATUS` | Show state, last reason, switch/stick pulse widths, RC, encoder and local radio TX health, target ramp, measured RPM, ESC bytes and collective angle. |
| `GAINS` | Show PIDF coefficients, active target and ESC limit, legal ranges, ramp and sampling settings. |
| `SET P n`, `SET I n`, `SET D n` | Set proportional, integral and derivative gains. Output is in ESC byte units, error is in RPM and time is in seconds. |
| `SET FR n`, `SET FC n` | Set base feedforward per target RPM and additional feedforward per target RPM per degree of absolute collective deviation from center. |
| `FFEST esc rpm` | Divide common ESC command by measured steady RPM at center collective. This assumes equal, untrimmed ESC commands as configured initially. The result is displayed only. |
| `SET TARGET rpm` | Set the next run's target. With the supplied configuration it must be above 20 and below 75 RPM. |
| `SET CAP byte` | Limit both governor output and transmitted wing commands. Accepts whole numbers from 1 through the compiled cap of 64 by default. |
| `MONITOR`, `VIEW RPM` | Select readable Monitor rows or labeled Serial Plotter rows. Both resume the USB stream. `VIEW ALL` includes roll, pitch, yaw and detailed system telemetry. |
| `STREAM OFF`, `STREAM ON`, `RATE hz` | Pause/resume USB telemetry or set its output rate to 1–20 Hz. OpenLog and the governor continue at their configured rates. |
| `REPORT`, `EXPORT` | Summarize the current or most recent run, or print copy-ready gain and feedforward declarations. |

The readable stream shows **measured RPM / ramped target / final goal**, signed tracking error, P/I/D/F contributions, left/right ESC commands, collective angle and output limiting. The error follows the ramp. An output-limit flag can indicate slew limiting during spool-up or reaching an output bound. The ESC numbers are commands. Actual thrust and Nano receive frequency require separate measurements.

For reference, `FFEST 25 50` prints 0.5 ESC units/RPM. The numbers illustrate the calculation. Use the ESC command and RPM measured on your hardware.
