# Morpheus experimental sketches

This directory is for **standalone experiments and archived development variants**, not production flight firmware. The current airplane-mode programs remain untouched in `firmware/`.

Each imported Arduino sketch is in its own directory with the same name as its main `.ino` file, so the Arduino IDE can open it directly. **Test sketches retain their original configuration constants.** Editing the main airplane-mode `Config.h` does **not** automatically update these experiments.

## Original source recovered and committed

| Experiment | Source | Notes |
| --- | --- | --- |
| IMU yaw heading hold / axle motor PIDF | [original](yaw-heading-hold/Project_Morpheus_Yaw_PIDF_Test/Project_Morpheus_Yaw_PIDF_Test.ino) | MPU6050 yaw integration; AS5600; pin 45 wing unlock at 850 µs; H-bridge pins 4/5 |
| IMU yaw heading hold / axle motor PIDF (updated) | [updated](yaw-heading-hold/Project_Morpheus_Yaw_PIDF_Test_Updated/Project_Morpheus_Yaw_PIDF_Test_Updated.ino) | Adds explicit OPEN/PARTIAL/LOCK positions, 1.34375:1 encoder gearing and lock/motor interlock |
| Manual helicopter collective/RPM feedforward | [V1](helicopter-manual/Project_Morpheus_Helicopter_Manual_RPM_FF_Test/Project_Morpheus_Helicopter_Manual_RPM_FF_Test.ino) | **Legacy 18-byte LoRa packet; incompatible with current Nano** |
| Manual helicopter collective/RPM feedforward | [V2](helicopter-manual/Project_Morpheus_Helicopter_Manual_RPM_FF_Test_V2/Project_Morpheus_Helicopter_Manual_RPM_FF_Test_V2.ino) | **Legacy 18-byte LoRa packet; incompatible with current Nano** |
| Manual helicopter collective/RPM feedforward | [V3](helicopter-manual/Project_Morpheus_Helicopter_Manual_RPM_FF_Test_V3_6Byte/Project_Morpheus_Helicopter_Manual_RPM_FF_Test_V3_6Byte.ino) | **6-byte LoRa format matches the current Nano packet layout**; wing unlock remains 850 µs |

These are source-preservation imports, **not newly compiled or hardware-tested releases**. In particular, the wing-lock pulse used in the experiments differs from the airplane-mode CH6 servo settings and from the general 1000 µs minimum; review mechanical clearance before running them.

## Other experiment versions located but not yet importable

Their filenames and key configuration values were identified during the 2026-11-05 audit, but the previous project file-library service stopped returning their complete source after the initial inspection. These should **not** be recreated from memory or replaced with guessed code:

- `Project_Morpheus_Wing_RPM_PID_Test.ino`
- `Project_Morpheus_Wing_RPM_PID_Test_Serial_Plotter.ino`
- `Project_Morpheus_Wing_RPM_PID_Unified.ino`
- `Project_Morpheus_Wing_RPM_PID_Unified_Failsafe_Toggle.ino`
- `Project_Morpheus_Wing_RPM_PID_No_Failsafes.ino`
- `Project_Morpheus_Local_DC_Motor_RPM_PID_Test.ino`
- `Project_Morpheus_Local_DC_RPM_PID_SAFE_v2.ino`
- `Emergency_Local_DC_Motor_Stop.ino`
- `DueFlightComputerBase_v2_serial_actuator_test.ino` (a replacement Arduino Due main tab that needs the surrounding airplane-mode tabs)

Also found but not yet readable: `Project_Morpheus_Yaw_PIDF_Test(1).ino` (duplicate/variant, identity not verified), an unnamed `.ino` upload, and the `NanoSerialOutputTest.zip` test bundle.

## Before running an experimental sketch

1. Read [the experiment configuration audit](../docs/experiment-compatibility.md).
2. Confirm every pin, output-neutral position and input direction against the **actual** assembled aircraft.
3. If a sketch transmits LoRa, confirm its packet size **and byte interpretation** against the installed Nano firmware. Legacy 18-byte code cannot communicate correctly with the current 6-byte receiver.
4. Check failsafes, motor startup state, and whether a wing-lock command is issued immediately at boot. Do not energize propulsion with propellers installed during initial checks.
5. Treat any `No_Failsafes` experiment as a historical unsafe test and do not run it on the aircraft without deliberately adding appropriate protections.
6. Make new experiments in a separate folder with independent local `Config.h` and a README. Do not overwrite the recovered originals.

See the [debugging guide](../docs/debugging.md) for repeatable testing.
