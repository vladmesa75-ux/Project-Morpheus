# Emergency local DC motor stop – Arduino Due

This is the original minimal `Emergency_Local_DC_Motor_Stop.ino` test preserved in its own Arduino sketch folder.

[Open the Arduino sketch](Emergency_Local_DC_Motor_Stop/Emergency_Local_DC_Motor_Stop.ino) using Arduino IDE with **Arduino Due** selected.

The sketch configures the two H-bridge input pins **4** and **5** as outputs, sets 8-bit PWM resolution, and repeatedly writes **zero to both**.

## Safety limitations

- **Disconnect the DC motor's power before flashing the Arduino.** A software upload does not safely interrupt a motor that is already moving.
- This is **not an emergency stop system**: a true emergency cutoff must independently remove actuator power.
- It does not operate the Nano LoRa receiver, ESCs or any other motors.
- It matches the current two-PWM-input H-bridge wiring on Due pins 4 and 5. Recheck wiring before use.
- It contains no serial commands and no aircraft flight-control functions.

The source was recovered from an earlier Project Morpheus test and stored without changing its intended behavior. It has not been compiled or tested on hardware during this repository update.
