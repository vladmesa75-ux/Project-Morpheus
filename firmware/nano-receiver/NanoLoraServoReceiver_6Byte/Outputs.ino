// ============================================================
// Outputs.ino
//
// This tab converts received packet values into servo and ESC signals.
// ============================================================

void setupOutputs() {
  // Attach all four servo outputs.
  // The order is the same as SERVO_PINS[] in Config.h:
  // servo 0 = D3
  // servo 1 = D4, flapperon
  // servo 2 = D5
  // servo 3 = D6, flapperon

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    servos[i].attach(SERVO_PINS[i]);
  }

  // Attach both ESC outputs.
  // ESC 0 = D8
  // ESC 1 = D7

  for (uint8_t i = 0; i < ESC_COUNT; i++) {
    escs[i].attach(ESC_PINS[i]);
  }
}

int mapSignedToServoUs(int8_t value, uint8_t servoOutputIndex) {
  // The Due sends signed servo values from -127 to +127.
  //
  // IMPORTANT:
  // The configured center is a real independent calibration point:
  //
  // -127 -> servoMinUs[]
  //    0 -> servoCenterUs[]
  // +127 -> servoMaxUs[]
  //
  // This is intentionally piecewise instead of one straight min-to-max map,
  // so changing servoCenterUs[] actually changes the physical neutral point.
  //
  // Reversing a servo by swapping min and max still works correctly.

  int constrainedValue = constrain((int)value, -127, 127);

  if (constrainedValue <= 0) {
    // Map -127...0 between MIN...CENTER.
    long span =
      (long)servoCenterUs[servoOutputIndex] -
      (long)servoMinUs[servoOutputIndex];

    return (int)(
      (long)servoCenterUs[servoOutputIndex] +
      ((long)constrainedValue * span) / 127L
    );
  }

  // Map 0...+127 between CENTER...MAX.
  long span =
    (long)servoMaxUs[servoOutputIndex] -
    (long)servoCenterUs[servoOutputIndex];

  return (int)(
    (long)servoCenterUs[servoOutputIndex] +
    ((long)constrainedValue * span) / 127L
  );
}

int mapUnsignedToEscUs(uint8_t value, uint8_t escOutputIndex) {
  // The Due sends throttle values from 0 to 254.
  // This function converts that into a standard ESC pulse.
  //
  // Example with normal endpoints:
  //   0 -> 1000 us, stopped or minimum throttle
  // 254 -> 2000 us, full throttle

  int constrainedValue = constrain((int)value, 0, UNSIGNED_INPUT_MAX);

  return map(
    constrainedValue,
    0,
    UNSIGNED_INPUT_MAX,
    escMinUs[escOutputIndex],
    escMaxUs[escOutputIndex]
  );
}

void applyFailsafe() {
  // Failsafe output:
  // - all servos go to their configured center positions
  // - both ESCs go to minimum throttle
  //
  // This is used on startup and if valid LoRa packets stop arriving.

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    servoOutputUs[i] = servoCenterUs[i];
    servos[i].writeMicroseconds(servoOutputUs[i]);
  }

  for (uint8_t i = 0; i < ESC_COUNT; i++) {
    escOutputUs[i] = escMinUs[i];
    escs[i].writeMicroseconds(escOutputUs[i]);
  }

  failsafeActive = true;
}

void applyOutputs() {
  // Convert each of the four signed packet values into a servo output.
  //
  // The mapping is controlled by SERVO_SIGNED_VALUE_INDEX[] in Config.h.
  // Current corrected mapping:
  // D4 uses signed value 1, packet byte 0, flapperon
  // D6 uses signed value 2, packet byte 1, flapperon
  // D3 uses signed value 3, packet byte 2
  // D5 uses signed value 4, packet byte 3

  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    uint8_t packetValueIndex = SERVO_SIGNED_VALUE_INDEX[i];

    servoOutputUs[i] = mapSignedToServoUs(
      signedValues[packetValueIndex],
      i
    );

    servos[i].writeMicroseconds(servoOutputUs[i]);
  }

  // Convert the two unsigned packet values into ESC outputs.
  // Byte 4 controls D8.
  // Byte 5 controls D7.

  for (uint8_t i = 0; i < ESC_COUNT; i++) {
    escOutputUs[i] = mapUnsignedToEscUs(unsignedValues[i], i);
    escs[i].writeMicroseconds(escOutputUs[i]);
  }

  failsafeActive = false;
}