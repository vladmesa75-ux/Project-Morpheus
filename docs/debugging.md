# Debugging workflow

Work from the physical input toward the actuator. Change only one thing at a time and keep an exact record of the code/configuration used.

1. **Power and wiring:** verify supply voltage, common ground, connector orientation and board pins. Disconnect propellers.
2. **Receiver and RC:** verify all Due channel pulse widths, valid range, reverse throttle, CH5 and CH6 switch states; test unplugged-receiver failsafe.
3. **Sensors:** check MPU6050 startup/calibration with the aircraft still; confirm AS5600 counts/direction and I2C buses.
4. **Mixer:** inspect airplane-mode commands for roll/flaps, differential thrust, and CH6 lock actuators; confirm hard limits.
5. **Radio:** verify the two matching `Config.h` radio parameter sets, successful 6-byte packets, hardware CRC and XOR.
6. **Nano outputs:** verify servo centers, reversed endpoints, each ESC pulse and 500 ms packet-loss failsafe using the Nano serial output.
7. **Mechanics:** verify full servo travel against actual linkages, never just serial numbers.
8. **Regression:** re-check existing airplane-mode behavior after any change intended for helicopter mode.

## Existing diagnostics

- **Due:** `ENABLE_USB_DEBUG`, `USB_DEBUG_INTERVAL_US`, `ENABLE_OPENLOG`, `OPENLOG_INTERVAL_US` in Due `Config.h`. Note that the imported main loop also prints loop timing and LoRa statistics periodically.
- **Nano:** serial output frequency, D4/D6 and D3/D5 output pulses, ESC D8/D7 pulses. Configure `SERIAL_PRINT_INTERVAL_MS` in Nano `Config.h`.
- **Source provenance:** see `docs/provenance/` for the original compile-repair and receiver notes.

## Useful bug report format

- Firmware: full Git commit SHA and branch
- Controller: Due / Nano / both; Arduino board package and library versions
- Configuration change: named constants, old → new
- Input stimulus: switch, stick positions, estimated command values
- Expected behavior vs observed behavior
- Serial log and whether propellers/actuators were disconnected
- Whether failsafe occurred or was deliberately tested

## Release convention

Use Git branches for experiments. Label a commit `hardware-bench-verified` or `flight-verified` **only after** real corresponding testing; compiling or static checks do not justify either claim.
