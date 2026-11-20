/*
  Project Morpheus: helicopter thrust-balance experiment (Arduino Due).

  Purpose: test left/right wing-motor command imbalance with the wing in its
  airplane orientation but with its rotation lock RELEASED. This sketch does
  not command the normal helicopter wing-flip or cyclic/collective control.

  Upload this sketch to the DUE only. Keep the current 6-byte LoRa Nano code.

  Due: CH3 at D44, tail unlock servo D39, H-tail fold D43, wing unlock D45.
  Nano: left ESC D8, right ESC D7, flaperon servos D4/D6 held at neutral.

  CH3 throttle is reversed: 2000 us OFF, 1000 us FULL. With fresh RC and
  throttle initially OFF, it directly controls the shared motor base command.

  Serial Monitor (115200, newline):
    SET 10   -> left command minus right command = +10 units
    SET -6   -> left command minus right command = -6 units
    +1       -> increase difference by 1 unit
    -1       -> decrease difference by 1 unit
    ZERO     -> reset difference to zero
    STATUS   -> print current configuration and motor commands
    HELP     -> print commands

  Trim is split across both ESC commands so that the average remains as
  close as possible to the CH3 command. Both ESCs are exactly zero at throttle
  OFF, and both commands are constrained to the 0..255 range. At either limit,
  the actual difference can be smaller than the requested difference.

  Safeguards: no motor output before a fresh, OFF CH3 signal has been seen
  after the startup servo-settle period; loss of RC re-arms only at OFF;
  Nano's own 500 ms LoRa failsafe still applies. There is NO wing-lock or
  fold-position feedback: visually verify motion before running propellers.
*/

#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "Config.h"

Servo tailUnlockServo;
Servo hTailFoldServo;
Servo wingUnlockServo;
Servo otherServos[3];

volatile uint32_t throttleRiseUs = 0;
volatile uint32_t throttleLastValidUs = 0;
volatile uint16_t throttlePulseUs = RC_THROTTLE_OFF_US;

bool loraReady = false;
volatile bool loraTxBusy = false;
uint32_t lastLoraTxStartUs = 0;
uint32_t lastLoraSendUs = 0;
uint32_t servoStartMs = 0;

bool throttleFresh = false;
bool throttleArmed = false;
uint16_t currentThrottleUs = RC_THROTTLE_OFF_US;
uint8_t baseMotorCommand = 0;
uint8_t leftMotorCommand = 0;
uint8_t rightMotorCommand = 0;
int16_t motorTrimUnits = INITIAL_HELICOPTER_MOTOR_TRIM_UNITS;

char serialBuffer[32];
uint8_t serialLength = 0;

void onThrottleEdge() {
  uint32_t now = micros();

  if (digitalRead(RC_THROTTLE_PIN) == HIGH) {
    throttleRiseUs = now;
  } else {
    uint32_t pulseUs = now - throttleRiseUs;
    if (pulseUs >= RC_MIN_VALID_US && pulseUs <= RC_MAX_VALID_US) {
      throttlePulseUs = (uint16_t)pulseUs;
      throttleLastValidUs = now;
    }
  }
}

void onLoRaTxDone() {
  loraTxBusy = false;
}

uint8_t clampMotorCommand(int16_t command) {
  if (command <= 0) return 0;
  if (command >= 255) return 255;
  return (uint8_t)command;
}

void updateThrottleAndMix() {
  uint16_t pulseCopy;
  uint32_t lastValidCopy;

  noInterrupts();
  pulseCopy = throttlePulseUs;
  lastValidCopy = throttleLastValidUs;
  interrupts();

  currentThrottleUs = pulseCopy;
  throttleFresh = lastValidCopy != 0 &&
    (uint32_t)(micros() - lastValidCopy) <= RC_TIMEOUT_US;

  if (!throttleFresh) {
    throttleArmed = false;
  }

  // Prevent a high-stick startup or restart following an RC-signal loss.
  if (throttleFresh && !throttleArmed &&
      (uint32_t)(millis() - servoStartMs) >= SERVO_SETTLE_TIME_MS &&
      currentThrottleUs >= THROTTLE_OFF_ARM_THRESHOLD_US) {
    throttleArmed = true;
  }

  baseMotorCommand = 0;
  leftMotorCommand = 0;
  rightMotorCommand = 0;

  if (!throttleFresh || !throttleArmed) return;

  // Exactly the current airplane-mode sense: 2000=off; 1000=full.
  int32_t safePulse = constrain((int32_t)currentThrottleUs,
                                (int32_t)RC_THROTTLE_FULL_US,
                                (int32_t)RC_THROTTLE_OFF_US);
  baseMotorCommand = (uint8_t)(
    ((int32_t)RC_THROTTLE_OFF_US - safePulse) * 255L /
    (RC_THROTTLE_OFF_US - RC_THROTTLE_FULL_US)
  );

  // Keep both ESCs at their untrimmed minimum for arming and stopping.
  if (baseMotorCommand == 0) return;

  // leftDelta - rightDelta == motorTrimUnits, even for odd trim values.
  int16_t leftDelta = motorTrimUnits / 2;
  int16_t rightDelta = leftDelta - motorTrimUnits;
  leftMotorCommand = clampMotorCommand((int16_t)baseMotorCommand + leftDelta);
  rightMotorCommand = clampMotorCommand((int16_t)baseMotorCommand + rightDelta);
}

void sendMotorPacket() {
  if (!loraReady || loraTxBusy) return;

  // Unchanged 6-byte Nano protocol. Flaperons held at airplane neutral.
  uint8_t packet[LORA_PACKET_BYTES] = {
    0,                    // signed left flaperon = neutral
    0,                    // signed right flaperon = neutral
    leftMotorCommand,     // Nano D8 (left ESC)
    rightMotorCommand,    // Nano D7 (right ESC)
    throttleFresh ? (uint8_t)0 : (uint8_t)255, // status; Nano ignores it
    0                     // XOR checksum below
  };

  for (uint8_t i = 0; i < LORA_PACKET_BYTES - 1; i++) {
    packet[LORA_PACKET_BYTES - 1] ^= packet[i];
  }

  LoRa.beginPacket();
  LoRa.write(packet, LORA_PACKET_BYTES);
  loraTxBusy = true;
  lastLoraTxStartUs = micros();

  if (LoRa.endPacket(true) == 0) {
    loraTxBusy = false;
  }
}

void serviceLoRaTxTimeout() {
  if (loraTxBusy &&
      (uint32_t)(micros() - lastLoraTxStartUs) > LORA_TX_TIMEOUT_US) {
    loraTxBusy = false;
    LoRa.idle();
  }
}

void printStatus() {
  Serial.print("CH3_us:");
  Serial.print(currentThrottleUs);
  Serial.print("\tRC:");
  Serial.print(throttleFresh ? "OK" : "LOST");
  Serial.print("\tarmed:");
  Serial.print(throttleArmed ? "YES" : "NO");
  Serial.print("\tLoRa:");
  Serial.print(loraReady ? "OK" : "FAILED");
  Serial.print("\tbase:");
  Serial.print(baseMotorCommand);
  Serial.print("\ttrim:");
  Serial.print(motorTrimUnits);
  Serial.print("\tleft_D8:");
  Serial.print(leftMotorCommand);
  Serial.print("\tright_D7:");
  Serial.print(rightMotorCommand);
  Serial.print("\tactual_diff:");
  Serial.println((int)leftMotorCommand - (int)rightMotorCommand);
}

void printHelp() {
  Serial.println("Motor trim (units on 0..255 Due->Nano commands):");
  Serial.println("  SET 10, SET -6, +1, -1, ZERO, STATUS, HELP");
  Serial.println("Positive trim = LEFT D8 command above RIGHT D7.");
  Serial.println("Trim is temporary until entered in Config.h and re-uploaded.");
  Serial.println("CH3: 2000us OFF, 1000us FULL. Start with CH3 OFF.");
}

bool parseInteger(const char *text, long *value) {
  if (!text || !*text) return false;
  char *end;
  long parsed = strtol(text, &end, 10);
  if (end == text) return false;
  while (*end && isspace((unsigned char)*end)) end++;
  if (*end) return false;
  *value = parsed;
  return true;
}

void changeTrim(long newTrim) {
  if (newTrim < -MAX_ABS_HELICOPTER_MOTOR_TRIM_UNITS ||
      newTrim > MAX_ABS_HELICOPTER_MOTOR_TRIM_UNITS) {
    Serial.print("Trim rejected: allowed range is +/-");
    Serial.println(MAX_ABS_HELICOPTER_MOTOR_TRIM_UNITS);
    return;
  }
  motorTrimUnits = (int16_t)newTrim;
  updateThrottleAndMix();
  printStatus();
}

void processSerialCommand(char *line) {
  while (*line && isspace((unsigned char)*line)) line++;
  for (char *p = line; *p; ++p) *p = (char)toupper((unsigned char)*p);

  if (strcmp(line, "HELP") == 0 || strcmp(line, "?") == 0) {
    printHelp();
  } else if (strcmp(line, "STATUS") == 0) {
    printStatus();
  } else if (strcmp(line, "ZERO") == 0) {
    changeTrim(0);
  } else {
    const char *numberText = line;
    bool relative = false;
    if (strncmp(line, "SET ", 4) == 0) {
      numberText = line + 4;
    } else if (line[0] == '+' || line[0] == '-') {
      relative = true;
    }
    long value;
    if (parseInteger(numberText, &value)) {
      changeTrim(relative ? (long)motorTrimUnits + value : value);
    } else if (*line) {
      Serial.println("Unknown command. Type HELP.");
    }
  }
}

void pollSerial() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (serialLength) {
        serialBuffer[serialLength] = '\0';
        processSerialCommand(serialBuffer);
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

  // Use fixed helicopter-test mechanical positions (CH6 is not read).
  tailUnlockServo.attach(TAIL_UNLOCK_PIN, SERVO_MIN_US, SERVO_MAX_US);
  hTailFoldServo.attach(HTAIL_FOLD_PIN, SERVO_MIN_US, SERVO_MAX_US);
  wingUnlockServo.attach(WING_LOCK_PIN, SERVO_MIN_US, SERVO_MAX_US);
  tailUnlockServo.writeMicroseconds(TAIL_UNLOCK_US);
  hTailFoldServo.writeMicroseconds(HTAIL_FOLDED_US);
  wingUnlockServo.writeMicroseconds(WING_UNLOCK_US);
  for (uint8_t i = 0; i < 3; i++) {
    otherServos[i].attach(OTHER_SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    otherServos[i].writeMicroseconds(OTHER_SERVO_CENTER_US);
  }
  servoStartMs = millis();

  pinMode(RC_THROTTLE_PIN, INPUT);
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

  Serial.println("Morpheus helicopter motor balance: Due test ready");
  Serial.println("Wing UNLOCK 850us; tail unlock 1700us; H-tail FOLD 1000us");
  Serial.println("Wing orientation is mechanical: leave it in AIRPLANE orientation.");
  Serial.println("Remove propellers for first servo / ESC / failsafe checks.");
  printHelp();
  if (!loraReady) Serial.println("ERROR: LoRa start failed, motors stay off.");
}

void loop() {
  static uint32_t lastPrintMs = 0;

  pollSerial();
  updateThrottleAndMix();
  serviceLoRaTxTimeout();

  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastLoraSendUs) >= LORA_SEND_PERIOD_US) {
    if (!loraTxBusy && loraReady) {
      sendMotorPacket();
      lastLoraSendUs = nowUs;
    }
  }

  if ((uint32_t)(millis() - lastPrintMs) >= 500UL) {
    lastPrintMs = millis();
    printStatus();
  }
}
