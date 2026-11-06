# Morpheus flight replay v2

Redesigned browser-based post-flight review app. This is **not** the current aircraft firmware, and it does not connect to live LoRa telemetry.

## Open

1. Download `index.html` from this directory using GitHub's **Download raw file** control, or use GitHub Desktop to pull the repository.
2. Double-click the downloaded `index.html` to open it in Chrome, Edge, or Firefox.
3. The original embedded `LOG00084.TXT` recording opens automatically. Use **Open flight log** for another file.
4. Play/pause, drag the time slider, or click and drag charts to seek. Drag the aircraft to orbit the camera and use the view presets.

GitHub's normal file preview **does not run HTML applications**. You need to open the downloaded HTML in a browser, not view the GitHub code page.

## Log format

The required CSV header is exactly:

```csv
time_us,rc1,rc2,rc3,rc4,rc5,rc6,roll_deg,pitch_deg,yaw_deg
```

Timestamps are microseconds; RC channels are pulse widths in microseconds; attitude angles are in degrees. The current airplane-mode Due logger with its default six-channel logging configuration uses this format.

Like the original app, this replay handles 32-bit timer rollover and estimates elapsed time across apparent timestamp resets. It skips malformed rows and reports their count. This is for viewing, **not** a substitute for sensor or log validation.

Throttle channel 3 remains reversed: 2000 µs = 0%, 1000 µs = 100%.

## What's new

- New responsive dark flight-analysis layout.
- Interactive perspective aircraft view with orbit, zoom, and preset camera views.
- Synchronized attitude and RC command history, individually selectable traces, hover readings, and click-to-seek.
- Playback speed control, scrubber, current telemetry, and log diagnostics.
- Self-contained HTML with no network libraries required.

## Technical limitations and verification

The 3D aircraft is **illustrative**, not the original CAD model and not dimensionally verified. Model attitude follows the logged roll/pitch/yaw angles; orientation and sign conventions still need to be checked against known physical aircraft attitudes. No GPS trajectory or altitude is estimated.

Original log data from the archived app are embedded in the v2 HTML. Imported files are read locally and not uploaded to a server. Treat any published repository copy as containing this flight recording.

**Verification:** File existence on GitHub and JavaScript syntax were checked. Full interactive browser testing and calibration against flight hardware have **not** yet been performed. If any visualization appears inverted, do not use it to make flight-safety decisions until axes are verified.

## Source preservation

- Original app: [`../flight-replay/MorpheusFlightReplay.html`](../flight-replay/MorpheusFlightReplay.html).
- New app: [`index.html`](index.html).
- Firmware was not modified by this change.
