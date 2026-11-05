# Hardware and pin map (as coded)

The source files, not the drawing, are the authority for **what the imported firmware actually drives**. The uploaded aircraft configuration sketches document intended hardware; compare these against real wiring before powering actuators.

## Arduino Due flight computer

| Function | Pin(s) | Imported firmware behavior |
| --- | --- | --- |
| RC CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw | 48, 46, 44, 42 | RC pulse inputs |
| RC CH5 flap, CH6 switch | 40, 38 | RC pulse inputs |
| Tail unlocking servo | 39 | Two configured positions driven by CH6 |
| New H-tail folding servo | 43 | **Planned feature**; pin already attached in imported code, currently centered |
| Existing CH6 servo | 45 | Two configured positions driven by CH6 |
| Other servo outputs | 47, 49, 51 | Attached, currently centered |
| Elevator | 41 | **Direct RC**, not driven by Due Servo library |
| LoRa SX1278 SS/RST/DIO0 | 10 / 9 / 2 | Radio control; SPI pins are board-specific |
| DC motor-driver inputs | 4, 5 | Configured, current airplane mixer commands zero |
| MPU6050 | Wire (I2C0) | 0x68, default 400 kHz |
| AS5600 | Wire1 (I2C1) | 0x36, default 400 kHz |
| OpenLog | Serial2 | 115200 baud |

Due servo output list is `{39,43,45,47,49,51}`, six total. The imported `Servos.ino` still has an outdated comment claiming seven servos; the array in `Config.h` is authoritative.

## Arduino Nano receiver

| Function | Nano pin | Imported firmware behavior |
| --- | --- | --- |
| Servo output 1 | D3 | Locally generated signed zero (center) |
| Left flaperon command | D4 | LoRa packet byte 0 |
| Servo output 3 | D5 | Locally generated signed zero (center) |
| Right flaperon command | D6 | LoRa packet byte 1 |
| ESC output 1 | D8 | LoRa packet byte 2 |
| ESC output 2 | D7 | LoRa packet byte 3 |
| LoRa SS/RST/DIO0 | D10 / D9 / D2 | Radio control |
| LoRa SPI | D11 / D12 / D13 | MOSI / MISO / SCK |

**Electrical caution:** a typical Nano operates with 5 V logic, while SX1278 modules use 3.3 V logic. Check the breakout board and level shifting. Do not assume an unprotected SX1278 is 5 V tolerant.

## Verify before connecting actuators

- Servo rail power and grounds are not specified in the uploaded firmware. Do not source motor/servo currents from MCU pins.
- Confirm which onboard servo uses pins 47, 49 and 51; the imported airplane mixer holds them at center.
- Check pin 43's existing center pulse against the new tail-fold geometry before attaching the servo horn.
- Confirm receiver CH numbering with the actual transmitter mapping, and check reverse throttle (2000 us off, 1000 us full on the Due).
