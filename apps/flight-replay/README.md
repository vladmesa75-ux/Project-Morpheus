# Morpheus flight replay

This directory contains the original **MorpheusFlightReplay.html** browser-based replay app created on **31 July 2026**. It was recovered from the original project-file library and committed **without functional changes**.

## Open the app

1. Clone or pull `Project-Morpheus` using GitHub Desktop.
2. Open `MorpheusFlightReplay.html` in a modern desktop browser (double-click it or drag it into a browser).
3. The app initially loads an embedded example/recorded flight, labeled `LOG00084.TXT`.
4. Click **Open another flight log** to load a new `.TXT` or `.CSV` log file from your computer.
5. Use **Play**, **Pause**, **Restart**, the timeline slider and playback-speed menu (0.25×–4×).

The app shows a 3D representation of aircraft attitude, roll/pitch/yaw readouts, reversed CH3 throttle and all six RC channels. It also includes an attitude history graph.

This is a **post-flight replay tool**, not a live LoRa telemetry dashboard. The original HTML includes the recorded flight dataset and is relatively large. It also references externally hosted UI libraries for some interface functionality, so fully offline behavior is not guaranteed.

## Required flight log format

The app expects comma-separated text with this **exact header order**:

```csv
time_us,rc1,rc2,rc3,rc4,rc5,rc6,roll_deg,pitch_deg,yaw_deg
```

Each subsequent line must contain exactly 10 numeric values. `time_us` is a timestamp in microseconds; `rc1`–`rc6` are RC pulse widths in µs; `roll_deg`, `pitch_deg` and `yaw_deg` are angles in degrees.

The current Due airplane-mode OpenLog writer in `firmware/airplane-mode/DueFlightComputerBase_v2/LoggingDebug.ino` emits the same column names and order **with the supplied configuration**: `OPENLOG_RC_CHANNEL_COUNT = 6` and `OPENLOG_RC_CHANNEL_INDEXES = {0,1,2,3,4,5}`. Changing those constants, or extending the logging format, may break compatibility; change the parser deliberately in a new app version.

The app converts reversed throttle on channel 3 as **2000 µs = 0%** and **1000 µs = 100%**. This matches the imported Due RC throttle configuration.

The parser handles 32-bit microsecond timestamp rollover and attempts to preserve elapsed playback time across board resets. It skips invalid data rows; this is useful for replay but should **not** be used as validation of flight sensor health.

## Status

- **Original source archived:** yes.
- **Static format comparison with current Due logger:** matches.
- **Rebuilt or rewritten:** no.
- **Fresh browser functionality tests in this repository:** not performed.
- **Live telemetry dashboard:** discussed previously; no original completed implementation has been recovered.

### Protect recorded data

The original app contains embedded recorded flight data. The repository is private, but the data would become visible if the repository were made public.
