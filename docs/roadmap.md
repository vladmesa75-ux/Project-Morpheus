# Development roadmap

This file records **planned work**; nothing here implies that flight code already implements it.

## Initial imported baseline (2026-11-05)

- Arduino Due airplane mode, direct elevator RC, CH6 tail unlock and another two-position servo.
- Nano receiver, D4/D6 flaperons and D8/D7 ESCs; compact six-byte LoRa commands.
- Configuration is local to each sketch; protocol consistency checks are separate.

## Next controlled changes

1. **H-tail 90-degree fold:** fixed **2200 µs unfolded** in airplane mode (including startup/RC failsafe), **1000 µs folded** in the five helicopter/rotor tests, implemented 18 November 2026. **Still required:** add verified fold-completion feedback or an interlock, design transition sequencing and validate mechanical end stops, travel duration, power-up and loss-of-signal behavior on hardware. Fixed positions alone do not implement transition control.
2. **Complete output inventory:** identify the currently centered Due servos on 47/49/51, remove assumptions from stale source comments without changing logic.
3. **Config cleanup:** retain per-device config ownership; consider named per-actuator settings rather than positional arrays; introduce reusable protocol definitions with regression tests when changing packet format.
4. **Helicopter mode:** add separate mode-specific control functions and PID gains, telemetry and mode handling. Define commands and safe states before connecting them to outputs.
5. **Flipped wing compensation:** conditional flaperon orientation/direction adjustment for the left wing after its physical flip; explicitly include linkage direction, actuator saturation, calibration and transition timing.
6. **Transitions:** require deterministic mode state, actuator interlocks, timeout behavior, sensor validation and systematic bench and restrained tests.

## Engineering principles

- No hidden mode changes in low-level output functions.
- Every new actuator should have a name, pin, min/center/max, direction, power-up/failsafe state, and test procedure.
- Do not assume generic 1500 us center is mechanically safe for new linkages.
- Keep independent calibration for each mode if necessary, and share only well-defined hardware limits.
- Preserve the current airplane-mode baseline until all new features are separately verified.
