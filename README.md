# Project Morpheus

Experimental VTOL aircraft firmware and technical notes.

## Repository status

**Imported baseline (5 November 2026).** The Arduino Due airplane-mode and Arduino Nano LoRa receiver firmware were imported from the files supplied for this project. The source code was not behaviorally changed as part of the initial repository organization. This is a **supplied / previously used baseline, not a freshly hardware-validated release**. The new H-tail fold servo on Due pin 43 is **not yet implemented**.

### Start here

| Location | Purpose |
| --- | --- |
| [Due airplane-mode sketch](firmware/airplane-mode/DueFlightComputerBase_v2/) | Current main airplane-mode flight computer; Arduino Due |
| [Nano receiver sketch](firmware/nano-receiver/NanoLoraServoReceiver_6Byte/) | Current compact 6-byte LoRa receiver; Arduino Nano |
| [Configuration guide](docs/configuration.md) | Where to change pins, servo endpoints, RC inputs, sensors, LoRa, logging, etc. |
| [Wiring reference](docs/hardware.md) | As-coded board pin maps and hardware notes |
| [LoRa protocol](docs/lora-protocol.md) | Shared 6-byte packet, checksum and radio compatibility |
| [Debugging guide](docs/debugging.md) | Safe, incremental fault isolation |
| [Planned features](docs/roadmap.md) | Helicopter mode, transitions, and tail-fold design |
| [Experimental sketches](experiments/) | Separate sandbox for tests, not flight-ready firmware |
| [Flight replay app](apps/flight-replay/) | Original browser-based 3D flight log replay and analysis app |
| [Configuration consistency check](tools/check_config.py) | Static Due/Nano consistency checks |

## Opening in Arduino IDE

Open the main `.ino` file *inside* the sketch folder. Keep the other `.ino` and `.h` tabs beside it. Arduino requires the primary `.ino` to have the same name as its containing folder.

- Due: `firmware/airplane-mode/DueFlightComputerBase_v2/DueFlightComputerBase_v2.ino`
- Nano: `firmware/nano-receiver/NanoLoraServoReceiver_6Byte/NanoLoraServoReceiver_6Byte.ino`

Install the correct board support package and libraries. Due requires `Adafruit MPU6050`, `Adafruit Unified Sensor`, `LoRa` (Sandeep Mistry), and their dependencies; both sketches use `Servo` and `SPI`. The Nano sketch also uses the `LoRa` library. Match the physical board/processor and library versions before flashing.

## Design principles

1. **Keep known code intact.** Do not modify the imported flight-control behavior silently.
2. **Configuration belongs near each sketch.** Edit its `Config.h` rather than searching through implementation tabs. This also keeps experiments independent.
3. **Shared protocol must remain compatible.** Due and Nano are separately compiled; they must agree on the exact LoRa settings and packet definition. Run `python tools/check_config.py` when changing either.
4. **Develop new flight modes separately.** Add helicopter/transition features incrementally and bench-test before integrating them.
5. **Prioritize diagnostics and failsafe.** Check RC, sensors, command mixer, radio, outputs and control mode in that order.

> **Safety:** This is experimental aircraft code. Source preservation and automated static checks do not prove safe flight. Remove propellers for first powered bench checks, verify every control direction and failsafe, and revalidate any actuator/pin/configuration change before flight.
