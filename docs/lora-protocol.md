# Due → Nano LoRa command protocol (imported baseline)

Both supplied sketches use a **6-byte** compact command frame. Byte offsets below are zero-based.

| Byte | Type | Meaning |
| --- | --- | --- |
| 0 | int8 encoded as raw byte | Left flaperon command, -127 through +127 |
| 1 | int8 encoded as raw byte | Right flaperon command, -127 through +127 |
| 2 | uint8 | Left motor / Nano ESC D8 throttle |
| 3 | uint8 | Right motor / Nano ESC D7 throttle |
| 4 | uint8 | Special byte: CH5 switch position normally, 255 for RC failsafe/status |
| 5 | uint8 | XOR checksum of bytes 0–4 |

The receiver rejects any packet of the wrong length or checksum. It generates zero commands locally for Nano servo outputs D3 and D5. The receiver code itself does not interpret byte 4 to command additional outputs.

## Radio settings

- Frequency: 433,000,000 Hz
- Spreading factor: 7
- Bandwidth: 500,000 Hz
- Coding rate denominator: 5 (4/5)
- Hardware LoRa CRC: enabled at both ends
- Manual XOR checksum: enabled
- Due TX power: 17 dBm in this firmware
- Nano receiver failsafe after 500 ms without a good packet

The Due uses asynchronous TX with a timeout, as documented in its original source. Modifying LoRa settings or packet layout **requires coordinated changes to both sketches**.

## Compatibility checks

Run `python tools/check_config.py` to compare transmitted/received packet sizes and radio settings. This script is a static check and does not exercise a real radio. It also flags the current upper throttle scaling discrepancy (Due 255, Nano 254) for deliberate review.

## Future protocol evolution

Do not silently reinterpret a byte. If helicopter mode requires additional commands, define a **versioned protocol** and update both endpoints together. The current 6-byte layout must remain recoverable as a known airplane-mode baseline.
