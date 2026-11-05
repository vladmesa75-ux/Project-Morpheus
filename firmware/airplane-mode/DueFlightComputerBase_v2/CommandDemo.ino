/*
  CommandDemo.ino

  Aircraft command mixer for the current H-tail arrangement.

  Main-board outputs:
  - Pin 39: tail unlocking servo, controlled by CH6.
  - Pin 41: not attached or driven by the Arduino Servo library.
  - Pin 45: existing two-position servo, also controlled by CH6.
  - Elevator: not controlled by the Arduino.

  Nano outputs through LoRa:
  - Signed value 1: left flaperon.
  - Signed value 2: right flaperon.
  - Unsigned value 1: left motor throttle.
  - Unsigned value 2: right motor throttle.

  CH4 yaw is used only for differential thrust.
*/

void initializeFlightCommand() {
  // Start every Due servo centered.
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    command.servoUs[i] = SERVO_CENTER_US;
  }

  // Use the correct safe values for the active CH6 servos.
  command.servoUs[TAIL_UNLOCK_SERVO_INDEX] = TAIL_UNLOCK_FAILSAFE_US;
  command.servoUs[CH6_SERVO_INDEX] = CH6_SERVO_FAILSAFE_US;

  command.motorPower = 0;

  loraSignedValue1 = 0;
  loraSignedValue2 = 0;
  loraSignedValue3 = 0;
  loraSignedValue4 = 0;

  loraUnsignedValue1 = 0;
  loraUnsignedValue2 = 0;

  loraArbitraryByte = 255;
}

void userFlightLogic() {
  // Immediately use the safe outputs if any receiver channel is stale.
  if (!rc.fresh) {
    applyReceiverFailsafe();
    return;
  }

  // ----------------------------------------------------------
  // Convert the receiver channels into normalized commands.
  // ----------------------------------------------------------

  int16_t rollCommand = rcPulseToSignedCommand(
    rc.us[0],
    RC_ROLL_MIN_US,
    RC_ROLL_CENTER_US,
    RC_ROLL_MAX_US,
    ROLL_INPUT_DIRECTION
  );

  uint8_t throttleCommand = rcThrottleToUnsignedCommand(rc.us[2]);

  int16_t yawCommand = rcPulseToSignedCommand(
    rc.us[3],
    RC_YAW_MIN_US,
    RC_YAW_CENTER_US,
    RC_YAW_MAX_US,
    YAW_INPUT_DIRECTION
  );

  int16_t flapCommand = ch5ToFlapCommand(rc.us[4]);

  // ----------------------------------------------------------
  // CH6 two-position servos.
  // Low and middle switch positions use position 1.
  // High switch position uses position 2.
  // ----------------------------------------------------------

  bool ch6Position2 = (rc.us[5] >= CH6_POSITION_2_MIN_US);

  // Tail unlocking servo on pin 39.
  command.servoUs[TAIL_UNLOCK_SERVO_INDEX] = ch6Position2
    ? TAIL_UNLOCK_POSITION_2_US
    : TAIL_UNLOCK_POSITION_1_US;

  // Existing two-position servo on pin 45.
  command.servoUs[CH6_SERVO_INDEX] = ch6Position2
    ? CH6_SERVO_POSITION_2_US
    : CH6_SERVO_POSITION_1_US;

  // Keep every other unused Due servo centered.
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (i != TAIL_UNLOCK_SERVO_INDEX &&
        i != CH6_SERVO_INDEX) {
      command.servoUs[i] = SERVO_CENTER_US;
    }
  }

  // The old onboard DC motor output is not used.
  command.motorPower = 0;

  // ----------------------------------------------------------
  // Full-span flaperon mixer sent to the Nano.
  // ----------------------------------------------------------

  int16_t flaperonRollCommand = applyPercent(
    rollCommand,
    FLAPERON_ROLL_GAIN_PERCENT
  );

  int16_t leftFlaperonCommand =
    (flaperonRollCommand * FLAPERON_LEFT_ROLL_SIGN) + flapCommand;

  int16_t rightFlaperonCommand =
    (flaperonRollCommand * FLAPERON_RIGHT_ROLL_SIGN) + flapCommand;

  leftFlaperonCommand = limitSignedCommand(
    (int32_t)leftFlaperonCommand * FLAPERON_LEFT_DIRECTION
  );

  rightFlaperonCommand = limitSignedCommand(
    (int32_t)rightFlaperonCommand * FLAPERON_RIGHT_DIRECTION
  );

  loraSignedValue1 = constrainSignedByte(leftFlaperonCommand);
  loraSignedValue2 = constrainSignedByte(rightFlaperonCommand);
  loraSignedValue3 = 0;
  loraSignedValue4 = 0;

  // ----------------------------------------------------------
  // Differential thrust from CH4 yaw.
  // There is no Arduino-controlled rudder or elevator.
  // ----------------------------------------------------------

  int16_t differentialCommand = applyPercent(
    yawCommand,
    DIFFERENTIAL_THRUST_YAW_GAIN_PERCENT
  );

  differentialCommand *= DIFFERENTIAL_THRUST_DIRECTION;

  int16_t leftMotorCommand =
    (int16_t)throttleCommand + differentialCommand;

  int16_t rightMotorCommand =
    (int16_t)throttleCommand - differentialCommand;

  leftMotorCommand = applyMotorScaleAndTrim(
    leftMotorCommand,
    LEFT_MOTOR_SCALE_PERCENT,
    LEFT_MOTOR_OFFSET_BYTE
  );

  rightMotorCommand = applyMotorScaleAndTrim(
    rightMotorCommand,
    RIGHT_MOTOR_SCALE_PERCENT,
    RIGHT_MOTOR_OFFSET_BYTE
  );

  loraUnsignedValue1 = constrainUnsignedByte(leftMotorCommand);
  loraUnsignedValue2 = constrainUnsignedByte(rightMotorCommand);

  // Keep the existing CH5 flap-position information byte.
  loraArbitraryByte = ch5ToPositionNumber(rc.us[4]);
}

void applyReceiverFailsafe() {
  // Center all ordinary Due servos first.
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    command.servoUs[i] = SERVO_CENTER_US;
  }

  // Apply the correct safe values for the two active CH6 servos.
  command.servoUs[TAIL_UNLOCK_SERVO_INDEX] = TAIL_UNLOCK_FAILSAFE_US;
  command.servoUs[CH6_SERVO_INDEX] = CH6_SERVO_FAILSAFE_US;

  command.motorPower = 0;

  // Neutral flaperons and stopped Nano motors.
  loraSignedValue1 = 0;
  loraSignedValue2 = 0;
  loraSignedValue3 = 0;
  loraSignedValue4 = 0;

  loraUnsignedValue1 = 0;
  loraUnsignedValue2 = 0;

  // 255 remains the receiver-failsafe/status marker.
  loraArbitraryByte = 255;
}

int16_t rcPulseToSignedCommand(
  uint16_t pulseUs,
  uint16_t minUs,
  uint16_t centerUs,
  uint16_t maxUs,
  int8_t direction
) {
  pulseUs = (uint16_t)constrainInt16(pulseUs, minUs, maxUs);

  int32_t commandValue;

  if (pulseUs <= centerUs) {
    int32_t inputRange = (int32_t)centerUs - minUs;

    if (inputRange <= 0) {
      commandValue = 0;
    } else {
      commandValue = -127L +
        ((int32_t)(pulseUs - minUs) * 127L) / inputRange;
    }
  } else {
    int32_t inputRange = (int32_t)maxUs - centerUs;

    if (inputRange <= 0) {
      commandValue = 0;
    } else {
      commandValue =
        ((int32_t)(pulseUs - centerUs) * 127L) / inputRange;
    }
  }

  commandValue *= direction;
  commandValue = limitSignedCommand(commandValue);

  if (commandValue >= -CONTROL_DEADBAND_BYTE &&
      commandValue <= CONTROL_DEADBAND_BYTE) {
    commandValue = 0;
  }

  return (int16_t)commandValue;
}

uint8_t rcThrottleToUnsignedCommand(uint16_t pulseUs) {
  uint16_t lowUs = RC_THROTTLE_FULL_US;
  uint16_t highUs = RC_THROTTLE_OFF_US;

  if (lowUs > highUs) {
    uint16_t temporary = lowUs;
    lowUs = highUs;
    highUs = temporary;
  }

  pulseUs = (uint16_t)constrainInt16(pulseUs, lowUs, highUs);

  int32_t inputRange =
    (int32_t)RC_THROTTLE_OFF_US - RC_THROTTLE_FULL_US;

  if (inputRange == 0) {
    return 0;
  }

  int32_t commandValue =
    ((int32_t)RC_THROTTLE_OFF_US - pulseUs) * 255L / inputRange;

  return constrainUnsignedByte(commandValue);
}

int16_t ch5ToFlapCommand(uint16_t ch5Us) {
  if (ch5Us < RC_CH5_LOW_MAX_US) {
    return FLAP_SETTING_LOW_BYTE;
  }

  if (ch5Us > RC_CH5_HIGH_MIN_US) {
    return FLAP_SETTING_HIGH_BYTE;
  }

  return FLAP_SETTING_MID_BYTE;
}

uint8_t ch5ToPositionNumber(uint16_t ch5Us) {
  if (ch5Us < RC_CH5_LOW_MAX_US) {
    return 0;
  }

  if (ch5Us > RC_CH5_HIGH_MIN_US) {
    return 2;
  }

  return 1;
}

int16_t applyMotorScaleAndTrim(
  int16_t value,
  int16_t scalePercent,
  int16_t offset
) {
  int32_t adjustedValue =
    ((int32_t)value * scalePercent) / 100L + offset;

  return constrainInt16(
    adjustedValue,
    UNSIGNED_COMMAND_MIN,
    UNSIGNED_COMMAND_MAX
  );
}

int16_t applyPercent(int16_t value, int16_t percent) {
  return (int16_t)(((int32_t)value * percent) / 100L);
}

int16_t limitSignedCommand(int32_t value) {
  return constrainInt16(
    value,
    SIGNED_COMMAND_MIN,
    SIGNED_COMMAND_MAX
  );
}

int16_t constrainInt16(int32_t value, int16_t low, int16_t high) {
  if (value < low) {
    return low;
  }

  if (value > high) {
    return high;
  }

  return (int16_t)value;
}

void applyFlightCommand() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    setServoUs(i, command.servoUs[i]);
  }

  setMotorPower(command.motorPower);
}