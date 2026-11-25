# Morpheus Nano flaperon calibration

For the classic 16 MHz ATmega328P Nano and the existing six-byte Due transmitter. In Arduino IDE, install the Arduino AVR Boards package and **LoRa by Sandeep Mistry**. Select Arduino Nano and the bootloader setting used by your board. Each sketch folder is self-contained. Open its matching .ino file with all sibling headers present. The sketches use Timer1 directly.

## Measure the linkage

1. Disconnect propulsion power. Power the servos from their normal supply with a common ground. Upload NanoFlaperonBench and open Serial Monitor at 115200 baud with Newline.
2. The bench sketch starts D4 (left) at 1430 µs and D6 (right) at 1470 µs. Use commands such as `L 1435` and `R 1465` to adjust one side at a time. Commands accept 500–2500 µs. Approach each angle in small steps within the linkage's free travel.
3. Establish zero with your angle gauge. Use the Due's intended positive direction for each surface. Measure the pulse width for every angle in the table below, including neutral. Repeat from the opposite direction if you want to check linkage backlash.
4. With the Due transmitting, read the valid packet frequency from the monitor once per second. D4 and D6 continue to hold your manual setpoints. Both ESCs stay at 1000 µs. Without a transmitter the bench keeps pulsing the servos at approximately 50 Hz.
5. Copy the nine measurements for each side into its row in NanoFlaperonReceiver/Calibration.h, in the listed order. Each row can increase or decrease. With an incomplete or nonmonotonic row the receiver holds the original neutral settings and minimum throttle.

| Physical angle (°) | −20 | −15 | −10 | −5 | 0 | +5 | +10 | +15 | +20 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Due command | −127 | −95 | −64 | −32 | 0 | 32 | 64 | 95 | 127 |

The eight-bit protocol gives about 0.157° per step. The table assigns each 5° mark to its nearest representable signed command. Commands at the marks reproduce the measured widths exactly. Intermediate commands use straight-line interpolation between adjacent measurements, rounded to the nearest microsecond. This is open-loop surface calibration. Actual angle depends on linkage repeatability and load.

## Check the receiver on the bench

Upload NanoFlaperonReceiver after filling in the table. With propulsion power still disconnected, check zero, both endpoints, an intermediate angle and both surface directions. Stop the transmitter and check that the flaperons return to measured neutral and both ESC signals drop to 1000 µs after 500 ms.

A valid packet starts simultaneous pulses on D4/D6 as soon as it is decoded at the expected approximately 80 Hz. A 5 ms minimum start-to-start period limits bursts to 200 Hz and keeps the latest pending command. A 20 ms idle refresh repeats the last width between sparse packets. At a collision with an idle refresh the new command waits for the minimum period. The 200 Hz ceiling describes the scheduler, not a tested servo rating. Verify D4/D6 pulse widths and spacing on the real Nano with a scope or logic analyzer. The serial RX rate counts accepted packets, not shaft movements.

D3 and D5 stay centered at 1500 µs. The left ESC (D8) and right ESC (D7) retain a separate 50 Hz frame and the baseline throttle scaling (0 to 254 maps to 1000–2000 µs, with 255 saturated). The sketches retain the 433 MHz radio configuration, hardware CRC, six-byte packet length and XOR check. Byte 4 retains its existing status meaning and has no local actuator action.
