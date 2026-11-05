# Experimental sketches

Store isolated bench and development sketches here, each in its **own Arduino sketch directory** (folder name matches its main `.ino` file). An experiment must never overwrite the flight-ready airplane-mode source.

Suggested future directories (not yet supplied, so not included as code): `imu-yaw-heading-hold/`, `rotor-pid/`, `servo-sweep/`, `as5600-rpm/`, `transition-test/`.

Every experiment should have its own configuration and a brief README stating hardware pins, what it tests, expected outputs, known risks and whether it was verified. Do not copy production RC/LoRa failsafe assumptions without reviewing them.

**No test program has been invented or uploaded by this repository import.** Previously discussed tests can be added once their exact source files are recovered.
