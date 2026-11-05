/*
  RcReceiver.ino

  This tab reads the 6 used RC receiver PWM channels.

  It uses interrupts instead of pulseIn().
  This is important because pulseIn() waits for a pulse and can stop the flight loop.

  CH1: right joystick left-right = 1000->2000
  CH2: right joystick up-down = 1000->2000
  CH3: left joystick up-down = 1000->2000
  CH4: left joystick left-right = 1000->2000
  CH5: remote CH5
  CH6: remote CH6

  CH7 and CH8 are not used and are intentionally not read.
*/

void setupRcInputs() {
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
    pinMode(RC_PINS[i], INPUT);
  }

  attachInterrupt(digitalPinToInterrupt(RC_PINS[0]), readRc0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_PINS[1]), readRc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_PINS[2]), readRc2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_PINS[3]), readRc3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_PINS[4]), readRc4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_PINS[5]), readRc5, CHANGE);
}

void handleRcInterrupt(uint8_t index) {
  /*
    This function is called whenever one receiver pin changes.

    Rising edge:
    - the signal went HIGH
    - save the start time

    Falling edge:
    - the signal went LOW
    - subtract the start time from the current time
    - the result is the channel pulse width in microseconds
  */

  uint32_t now = micros();

  if (digitalRead(RC_PINS[index]) == HIGH) {
    rcRiseUs[index] = now;
  } else {
    uint32_t width = now - rcRiseUs[index];

    if (width >= RC_MIN_VALID_US && width <= RC_MAX_VALID_US) {
      rcPulseUs[index] = (uint16_t)width;
      rcLastEdgeUs[index] = now;
    }
  }
}

// Tiny wrapper functions because attachInterrupt() cannot pass a channel number.
void readRc0() { handleRcInterrupt(0); }
void readRc1() { handleRcInterrupt(1); }
void readRc2() { handleRcInterrupt(2); }
void readRc3() { handleRcInterrupt(3); }
void readRc4() { handleRcInterrupt(4); }
void readRc5() { handleRcInterrupt(5); }

void updateRcFrame() {
  /*
    Copies the latest interrupt values into the normal rc structure.

    We briefly turn off interrupts while copying, so a value cannot change
    halfway through being read.
  */

  uint16_t pulseCopy[RC_CHANNEL_COUNT];
  uint32_t edgeCopy[RC_CHANNEL_COUNT];

  noInterrupts();
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
    pulseCopy[i] = rcPulseUs[i];
    edgeCopy[i] = rcLastEdgeUs[i];
  }
  interrupts();

  uint32_t now = micros();
  bool allFresh = true;
  uint32_t maxAge = 0;

  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
    rc.us[i] = pulseCopy[i];

    uint32_t age = now - edgeCopy[i];

    if (edgeCopy[i] == 0 || age > RC_TIMEOUT_US) {
      allFresh = false;
    }

    if (age > maxAge) {
      maxAge = age;
    }
  }

  rc.fresh = allFresh;
  rc.maxAgeUs = maxAge;
}