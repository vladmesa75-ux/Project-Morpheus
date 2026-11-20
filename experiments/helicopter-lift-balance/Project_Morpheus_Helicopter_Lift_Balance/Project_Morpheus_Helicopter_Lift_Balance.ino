/*
  Project Morpheus: helicopter aerodynamic lift-balance test (Arduino Due).

  CH3 -> both wing ESCs equally (0..255; 2000 us OFF / 1000 us FULL).
  CH2 -> helicopter collective, using the existing V3 mapping:
         LEFT and RIGHT signed flaperon commands both = -collective.
  Due Serial Monitor -> signed difference between flaperon commands.

  The left wing must PHYSICALLY be flipped into helicopter configuration.
  This sketch does not actuate the wing-flip mechanism or use cyclic control.
  It holds pin 45 unlocked (850 us), pin 39 tail LOCKED (1000 us),
  and pin 43 H-tail folded (1000 us); other Due servos are centered.

  Upload to the Due only. The current Nano 6-byte LoRa firmware is unchanged.
  The signed trim is not a measured physical flap angle or measured lift.

  Serial Monitor, 115200 baud (newline):
    SET 5, SET -5  set the LEFT-minus-RIGHT signed-command difference
    +1, -1         nudge the difference by one unit
    ZERO           clear the difference
    STATUS, HELP   display current settings

  Equal motor commands are maintained throughout the test. The difference
  is split across both flaperons, with saturation to -127..+127.
  At RC loss BOTH motors stop and flaperons return to neutral.
*/

#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "Config.h"

Servo tailLockServo;
Servo hTailFoldServo;
Servo wingUnlockServo;
Servo otherServos[3];

volatile uint32_t pitchRiseUs = 0;
volatile uint32_t pitchLastValidUs = 0;
volatile uint16_t pitchPulseUs = RC_PITCH_CENTER_US;
volatile uint32_t throttleRiseUs = 0;
volatile uint32_t throttleLastValidUs = 0;
volatile uint16_t throttlePulseUs = RC_THROTTLE_OFF_US;

bool loraReady = false;
volatile bool loraTxBusy = false;
uint32_t lastLoraTxStartUs = 0;
uint32_t lastLoraSendUs = 0;
uint32_t servoStartMs = 0;

bool rcFresh = false;
bool throttleArmed = false;
uint16_t currentPitchUs = RC_PITCH_CENTER_US;
uint16_t currentThrottleUs = RC_THROTTLE_OFF_US;
uint8_t motorCommand = 0;
int16_t collectiveCommand = 0;
int8_t leftFlaperonCommand = 0;
int8_t rightFlaperonCommand = 0;
int16_t flaperonTrimUnits = INITIAL_HELI_FLAPERON_TRIM_UNITS;

char serialBuffer[32];
uint8_t serialLength = 0;

void onPitchEdge() {
  uint32_t nowUs = micros();
  if (digitalRead(RC_PITCH_PIN) == HIGH) {
    pitchRiseUs = nowUs;
  } else {
    uint32_t width = nowUs - pitchRiseUs;
    if (width >= RC_MIN_VALID_US && width <= RC_MAX_VALID_US) {
      pitchPulseUs = (uint16_t)width;
      pitchLastValidUs = nowUs;
    }
  }
}

void onThrottleEdge() {
  uint32_t nowUs = micros();
  if (digitalRead(RC_THROTTLE_PIN) == HIGH) {
    throttleRiseUs = nowUs;
  } else {
    uint32_t width = nowUs - throttleRiseUs;
    if (width >= RC_MIN_VALID_US && width <= RC_MAX_VALID_US) {
      throttlePulseUs = (uint16_t)width;
      throttleLastValidUs = nowUs;
    }
  }
}

void onLoRaTxDone() {
  loraTxBusy = false;
}

int16_t clampSigned(int32_t value) {
  if (value < -127) return -127;
  if (value > 127) return 127;
  return (int16_t)value;
}

int16_t convertPitchToCollective(uint16_t pitchUs) {
  // Preserve the existing helicopter V3 piecewise ±127 CH2 mapping.
  int32_t value;
  if (pitchUs <= RC_PITCH_CENTER_US) {
    int32_t span = (int32_t)RC_PITCH_CENTER_US - RC_PITCH_MIN_US;
    if (span <= 0) return 0;
    pitchUs = (uint16_t)constrain((int32_t)pitchUs,
                                  (int32_t)RC_PITCH_MIN_US,
                                  (int32_t)RC_PITCH_CENTER_US);
    value = -127L + ((int32_t)(pitchUs - RC_PITCH_MIN_US) * 127L) / span;
  } else {
    int32_t span = (int32_t)RC_PITCH_MAX_US - RC_PITCH_CENTER_US;
    if (span <= 0) return 0;
    pitchUs = (uint16_t)constrain((int32_t)pitchUs,
                                  (int32_t)RC_PITCH_CENTER_US,
                                  (int32_t)RC_PITCH_MAX_US);
    value = ((int32_t)(pitchUs - RC_PITCH_CENTER_US) * 127L) / span;
  }

  value = clampSigned(value * COLLECTIVE_INPUT_DIRECTION);
  if (value >= -COLLECTIVE_DEADBAND_BYTE &&
      value <= COLLECTIVE_DEADBAND_BYTE) return 0;
  return (int16_t)value;
}

void updateRcAndMix() {
  uint16_t pitchUsCopy;
  uint16_t throttleUsCopy;
  uint32_t pitchLastUsCopy;
  uint32_t throttleLastUsCopy;

  noInterrupts();
  pitchUsCopy = pitchPulseUs;
  pitchLastUsCopy = pitchLastValidUs;
  throttleUsCopy = throttlePulseUs;
  throttleLastUsCopy = throttleLastValidUs;
  interrupts();

  currentPitchUs = pitchUsCopy;
  currentThrottleUs = throttleUsCopy;
  uint32_t nowUs = micros();
  bool pitchFresh = pitchLastUsCopy != 0 &&
    (uint32_t)(nowUs - pitchLastUsCopy) <= RC_TIMEOUT_US;
  bool throttleFresh = throttleLastUsCopy != 0 &&
    (uint32_t)(nowUs - throttleLastUsCopy) <= RC_TIMEOUT_US;
  rcFresh = pitchFresh && throttleFresh;

  if (!rcFresh) throttleArmed = false;

  // As in the previous test, CH3 must first be observed at its off position.
  if (rcFresh && !throttleArmed &&
      (uint32_t)(millis() - servoStartMs) >= SERVO_SETTLE_TIME_MS &&
      currentThrottleUs >= THROTTLE_OFF_ARM_THRESHOLD_US) {
    throttleArmed = true;
  }

  motorCommand = 0;
  collectiveCommand = 0;
  leftFlaperonCommand = 0;
  rightFlaperonCommand = 0;
  if (!rcFresh) return;

  // CH2 controls the collective even when CH3 is OFF, allowing a safe
  // propeller-off direction check before powered testing.
  collectiveCommand = convertPitchToCollective(currentPitchUs);

  int16_t leftBase = clampSigned(
    (int32_t)collectiveCommand * LEFT_HELI_FLAPERON_DIRECTION);
  int16_t rightBase = clampSigned(
    (int32_t)collectiveCommand * RIGHT_HELI_FLAPERON_DIRECTION);

  // Equal split around each blade's configured collective command.
  // leftDelta - rightDelta equals trim exactly for odd/even values.
  int16_t leftDelta = flaperonTrimUnits / 2;
  int16_t rightDelta = leftDelta - flaperonTrimUnits;
  leftFlaperonCommand = (int8_t)clampSigned(leftBase + leftDelta);
  rightFlaperonCommand = (int8_t)clampSigned(rightBase + rightDelta);

  if (!throttleArmed) return;

  // CH3 uses the exact reversed-throttle idea from airplane mode.
  int32_t safePulse = constrain((int32_t)currentThrottleUs,
                                (int32_t)RC_THROTTLE_FULL_US,
                                (int32_t)RC_THROTTLE_OFF_US);
  int32_t range = (int32_t)RC_THROTTLE_OFF_US - RC_THROTTLE_FULL_US;
  if (range <= 0) return;
  int32_t command = ((int32_t)RC_THROTTLE_OFF_US - safePulse) * 255L / range;
  motorCommand = (uint8_t)constrain(command, 0L, 255L);
}

void sendLoRaCommand() {
  if (!loraReady || loraTxBusy) return;

  uint8_t packet[LORA_PACKET_BYTES] = {
    (uint8_t)leftFlaperonCommand,  // signed command for Nano D4
    (uint8_t)rightFlaperonCommand, // signed command for Nano D6
    motorCommand,                  // left ESC, Nano D8
    motorCommand,                  // right ESC, Nano D7: always equal
    rcFresh ? (uint8_t)0 : (uint8_t)255,
    0
  };

  for (uint8_t i = 0; i < LORA_PACKET_BYTES - 1; i++) {
    packet[LORA_PACKET_BYTES - 1] ^= packet[i];
  }

  LoRa.beginPacket();
  LoRa.write(packet, LORA_PACKET_BYTES);
  loraTxBusy = true;
  lastLoraTxStartUs = micros();
  if (LoRa.endPacket(true) == 0) loraTxBusy = false;
}

void serviceLoRaTimeout() {
  if (loraTxBusy &&
      (uint32_t)(micros() - lastLoraTxStartUs) > LORA_TX_TIMEOUT_US) {
    loraTxBusy = false;
    LoRa.idle();
  }
}

void printStatus() {
  Serial.print("CH2_us:"); Serial.print(currentPitchUs);
  Serial.print("\tCH3_us:"); Serial.print(currentThrottleUs);
  Serial.print("\tRC:"); Serial.print(rcFresh ? "OK" : "LOST");
  Serial.print("\tarmed:"); Serial.print(throttleArmed ? "YES" : "NO");
  Serial.print("\tLoRa:"); Serial.print(loraReady ? "OK" : "FAILED");
  Serial.print("\tmotors:"); Serial.print(motorCommand);
  Serial.print("\tcollective:"); Serial.print(collectiveCommand);
  Serial.print("\tflap_trim:"); Serial.print(flaperonTrimUnits);
  Serial.print("\tleft_D4:"); Serial.print((int)leftFlaperonCommand);
  Serial.print("\tright_D6:"); Serial.print((int)rightFlaperonCommand);
  Serial.print("\tactual_diff:");
  Serial.println((int)leftFlaperonCommand - (int)rightFlaperonCommand);
}

void printHelp() {
  Serial.println("Helicopter lift balance: CH3 = motors; CH2 = collective.");
  Serial.println("Commands: SET 5, SET -5, +1, -1, ZERO, STATUS, HELP");
  Serial.println("Positive trim = left signed flap command above right.");
  Serial.println("No claim of measured blade lift: verify physical effect.");
  Serial.println("Trim resets after reboot unless saved in Config.h.");
  Serial.println("Keep CH3 OFF at startup; both CH2 and CH3 RC must be fresh.");
}

bool parseInteger(const char *source, long *result) {
  if (!source || !*source) return false;
  char *end;
  long n = strtol(source, &end, 10);
  if (end == source) return false;
  while (*end && isspace((unsigned char)*end)) end++;
  if (*end) return false;
  *result = n;
  return true;
}

void changeTrim(long newValue) {
  if (newValue < -MAX_ABS_HELI_FLAPERON_TRIM_UNITS ||
      newValue > MAX_ABS_HELI_FLAPERON_TRIM_UNITS) {
    Serial.print("Trim rejected. Limit: +/-");
    Serial.println(MAX_ABS_HELI_FLAPERON_TRIM_UNITS);
    return;
  }
  flaperonTrimUnits = (int16_t)newValue;
  updateRcAndMix();
  printStatus();
}

void processCommand(char *line) {
  while (*line && isspace((unsigned char)*line)) line++;
  for (char *p = line; *p; ++p) *p = (char)toupper((unsigned char)*p);

  if (strcmp(line, "HELP") == 0 || strcmp(line, "?") == 0) {
    printHelp();
  } else if (strcmp(line, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(line, "ZERO") == 0) {
    changeTrim(0);
  } else {
    const char *number = line;
    bool relative = false;
    if (strncmp(line, "SET ", 4) == 0) number = line + 4;
    else if (line[0] == '+' || line[0] == '-') relative = true;

    long n;
    if (parseInteger(number, &n)) {
      changeTrim(relative ? (long)flaperonTrimUnits + n : n);
    } else if (*line) {
      Serial.println("Unknown command. Type HELP.");
    }
  }
}

void serviceSerialCommands() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (serialLength) {
        serialBuffer[serialLength] = '\0';
        processCommand(serialBuffer);
        serialLength = 0;
      }
    } else if (serialLength < sizeof(serialBuffer) - 1) {
      serialBuffer[serialLength++] = ch;
    } else {
      serialLength = 0;
      Serial.println("Command too long. Type HELP.");
    }
  }
}

void setup() {
  Serial.begin(USB_BAUD);

  // Servo positions fixed for helicopter testing, independent of CH6.
  tailLockServo.attach(TAIL_LOCK_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  hTailFoldServo.attach(HTAIL_FOLD_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  wingUnlockServo.attach(WING_UNLOCK_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  tailLockServo.writeMicroseconds(TAIL_LOCKED_US);
  hTailFoldServo.writeMicroseconds(HTAIL_FOLDED_US);
  wingUnlockServo.writeMicroseconds(WING_UNLOCK_US);
  for (uint8_t i = 0; i < 3; ++i) {
    otherServos[i].attach(OTHER_SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    otherServos[i].writeMicroseconds(OTHER_SERVO_CENTER_US);
  }
  servoStartMs = millis();

  pinMode(RC_PITCH_PIN, INPUT);
  pinMode(RC_THROTTLE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RC_PITCH_PIN), onPitchEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_THROTTLE_PIN), onThrottleEdge, CHANGE);

  LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  loraReady = LoRa.begin(LORA_FREQUENCY_HZ);
  if (loraReady) {
    LoRa.setTxPower(LORA_TX_POWER_DBM);
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH_HZ);
    LoRa.setCodingRate4(LORA_CODING_RATE_DENOMINATOR);
    LoRa.enableCrc();
    LoRa.onTxDone(onLoRaTxDone);
  }

  Serial.println("Morpheus helicopter lift-balance test ready");
  Serial.println("Wing UNLOCK 850us; tail LOCKED 1000us; H-tail FOLD 1000us");
  Serial.println("Physically flip the left wing into helicopter configuration.");
  Serial.println("Check control directions and failsafe with propellers removed.");
  printHelp();
  if (!loraReady) Serial.println("ERROR: LoRa failed; Nano ESC failsafe applies.");
}

void loop() {
  static uint32_t lastPrintMs = 0;

  serviceSerialCommands();
  updateRcAndMix();
  serviceLoRaTimeout();

  uint32_t nowUs = micros();
  if (loraReady && !loraTxBusy &&
      (uint32_t)(nowUs - lastLoraSendUs) >= LORA_SEND_PERIOD_US) {
    sendLoRaCommand();
    lastLoraSendUs = nowUs;
  }

  if ((uint32_t)(millis() - lastPrintMs) >= 500UL) {
    lastPrintMs = millis();
    printStatus();
  }
}
