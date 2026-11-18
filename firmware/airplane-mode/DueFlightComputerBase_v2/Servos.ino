/*
  Servos.ino

  This tab controls all six Due servo outputs.
  Set mode-specific commanded positions in CommandDemo.ino
  and their bounds in Config.h.
*/

void setupServos() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    servos[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    setServoUs(i, command.servoUs[i]);
  }
}

bool setServoUs(uint8_t servoIndex, int16_t pulseUs) {
  /*
    Sends one servo a pulse in microseconds.

    servoIndex:
    - 0 means the first servo in SERVO_PINS
    - 1 means the second servo, etc.
    pulseUs:
    - permitted range is 500 to 2500 us
    - 1500 us is the generic neutral, not a safe position for every linkage
  */

  if (servoIndex >= SERVO_COUNT) {
    return false;
  }

  pulseUs = constrain(pulseUs, SERVO_MIN_US, SERVO_MAX_US);
  servos[servoIndex].writeMicroseconds(pulseUs);
  return true;
}

void centerAllServos() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    setServoUs(i, SERVO_CENTER_US);
  }
}