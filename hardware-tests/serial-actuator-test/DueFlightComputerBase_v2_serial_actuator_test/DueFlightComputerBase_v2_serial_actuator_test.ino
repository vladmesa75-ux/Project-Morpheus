/*
  DueFlightComputerBase_v2_serial_actuator_test.ino

  Temporary main tab for actuator testing from the Arduino Serial Monitor.

  What this test code controls:
  - Due servos on pins 39, 41, 43, 45, 47, 49, 51
  - Due DC motor on MOTOR_PIN_A / MOTOR_PIN_B
  - Nano servos through LoRa signed values 1 to 4
  - Nano ESCs through LoRa unsigned values 1 to 2

  Serial Monitor settings:
  - Baud: 115200, or whatever USB_BAUD is in Config.h
  - Line ending: Newline

  Safety:
  - Remove propellers before testing ESCs or motors.
  - Start with very small motor/ESC values.
*/

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ctype.h>
#include <stdlib.h>

#include "Config.h"
#include "TypesAndGlobals.h"
#include "FunctionPrototypes.h"

// Send the current Nano commands over LoRa at 50 Hz.
const uint32_t SERIAL_TEST_LORA_INTERVAL_US = 20000;

// Serial input line buffer.
const uint8_t SERIAL_LINE_MAX = 80;
char serialLine[SERIAL_LINE_MAX];
uint8_t serialLinePos = 0;

uint32_t lastLoraSendUs = 0;

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------

void setup() {
  Serial.begin(USB_BAUD);
  Serial.setTimeout(50);

  // Use 8-bit PWM values for the DC motor driver: 0 to 255.
  analogWriteResolution(8);

  // Start outputs in a safe state.
  setupMotor();
  setupServos();
  centerAllServos();
  initializeFlightCommand();

  // This test sketch does not read the RC receiver, but the LoRa packet
  // still contains receiver bytes. Set them to neutral so packet bytes 13-16
  // look normal instead of being zero.
  for (uint8_t i = 0; i < RC_CHANNEL_COUNT; i++) {
    rc.us[i] = 1500;
  }

  // Start LoRa so the Nano receives the test commands.
  loraOk = setupLoRaRadio();

  Serial.println();
  Serial.println("Serial actuator test ready.");
  Serial.println(loraOk ? "LoRa: OK" : "LoRa: FAILED");
  printHelp();
  printStatus();
}

// ------------------------------------------------------------
// Main loop
// ------------------------------------------------------------

void loop() {
  // Read commands typed into the Serial Monitor.
  readSerialCommands();

  // Apply the latest Due servo and DC motor values every loop.
  applyFlightCommand();

  // Keep the LoRa transmitter from getting stuck if DIO0/interrupt fails.
  serviceLoRaTxTimeout();

  // Send the current Nano servo/ESC commands repeatedly.
  uint32_t nowUs = micros();

  if (loraOk && !loraTxBusy && nowUs - lastLoraSendUs >= SERIAL_TEST_LORA_INTERVAL_US) {
    lastLoraSendUs = nowUs;

    buildLoRaPacket(telemetryPacket);

    if (sendLoRaPacket(telemetryPacket)) {
      packetsSentThisSecond++;
    }
  }
}

// ------------------------------------------------------------
// Serial command reader
// ------------------------------------------------------------

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\r') {
      // Ignore carriage return.
      continue;
    }

    if (c == '\n') {
      serialLine[serialLinePos] = '\0';
      processSerialCommand(serialLine);
      serialLinePos = 0;
      continue;
    }

    if (serialLinePos < SERIAL_LINE_MAX - 1) {
      serialLine[serialLinePos] = c;
      serialLinePos++;
    }
  }
}

void processSerialCommand(char *line) {
  char *cmd = strtok(line, " ,\t");

  if (cmd == NULL) {
    return;
  }

  if (tokenEquals(cmd, "?") || tokenEquals(cmd, "HELP")) {
    printHelp();
    return;
  }

  if (tokenEquals(cmd, "P") || tokenEquals(cmd, "PRINT")) {
    printStatus();
    return;
  }

  if (tokenEquals(cmd, "SAFE") || tokenEquals(cmd, "STOP")) {
    setAllSafe();
    Serial.println("OK: all outputs safe");
    printStatus();
    return;
  }

  if (tokenEquals(cmd, "CENTER")) {
    centerEverything();
    Serial.println("OK: all servos centered, motors stopped");
    printStatus();
    return;
  }

  if (tokenEquals(cmd, "DS")) {
    // Due servo by index.
    // Example: DS 2 1600 means servo index 2, pin 43, to 1600 us.
    bool ok = true;
    long index = readLongArgument(ok);
    long pulseUs = readLongArgument(ok);

    if (!ok || index < 0 || index >= SERVO_COUNT) {
      Serial.println("ERROR: use DS <index 0-6> <1000-2000 us>");
      return;
    }

    command.servoUs[index] = constrain((int)pulseUs, SERVO_MIN_US, SERVO_MAX_US);
    Serial.println("OK: Due servo set");
    printStatus();
    return;
  }

  if (tokenEquals(cmd, "DP")) {
    // Due servo by physical pin.
    // Example: DP 43 1600 means Due pin 43 to 1600 us.
    bool ok = true;
    long pin = readLongArgument(ok);
    long pulseUs = readLongArgument(ok);

    if (!ok) {
      Serial.println("ERROR: use DP <pin> <1000-2000 us>");
      return;
    }

    int8_t index = dueServoIndexFromPin((uint8_t)pin);

    if (index < 0) {
      Serial.println("ERROR: pin must be 39, 41, 43, 45, 47, 49, or 51");
      return;
    }

    command.servoUs[index] = constrain((int)pulseUs, SERVO_MIN_US, SERVO_MAX_US);
    Serial.println("OK: Due servo set");
    printStatus();
    return;
  }

  if (tokenEquals(cmd, "M")) {
    // Due DC motor power.
    // Example: M 80 or M -80.
    bool ok = true;
    long power = readLongArgument(ok);

    if (!ok) {
      Serial.println("ERROR: use M <-255 to 255>");
      return;
    }

    command.motorPower = constrain((int)power, -MOTOR_MAX_POWER, MOTOR_MAX_POWER);
    Serial.println("OK: Due DC motor set");
    printStatus();
    return;
  }

  if (tokenEquals(cmd, "NS")) {
    // Nano servo command through LoRa signed values.
    // Example: NS 1 40 means Nano signed value 1 = +40.
    bool ok = true;
    long servoNumber = readLongArgument(ok);
    long value = readLongArgument(ok);

    if (!ok || servoNumber < 1 || servoNumber > 4) {
      Serial.println("ERROR: use NS <servo 1-4> <-127 to 127>");
      return;
    }

    setNanoServoCommand((uint8_t)servoNumber, (int16_t)value);
    Serial.println("OK: Nano servo command set");
    printStatus();
    return;
  }

  if (tokenEquals(cmd, "E")) {
    // Nano ESC command through LoRa unsigned values.
    // Example: E 1 30 means Nano ESC 1 = 30/255.
    bool ok = true;
    long escNumber = readLongArgument(ok);
    long value = readLongArgument(ok);

    if (!ok || escNumber < 1 || escNumber > 2) {
      Serial.println("ERROR: use E <ESC 1-2> <0-255>");
      return;
    }

    setNanoEscCommand((uint8_t)escNumber, (int16_t)value);
    Serial.println("OK: Nano ESC command set");
    printStatus();
    return;
  }

  Serial.println("ERROR: unknown command. Type ? for help.");
}

// ------------------------------------------------------------
// Command helpers
// ------------------------------------------------------------

long readLongArgument(bool &ok) {
  char *token = strtok(NULL, " ,\t");

  if (token == NULL) {
    ok = false;
    return 0;
  }

  char *endPointer;
  long value = strtol(token, &endPointer, 10);

  if (*endPointer != '\0') {
    ok = false;
    return 0;
  }

  return value;
}

bool tokenEquals(const char *a, const char *b) {
  while (*a != '\0' && *b != '\0') {
    if (toupper(*a) != toupper(*b)) {
      return false;
    }

    a++;
    b++;
  }

  return (*a == '\0' && *b == '\0');
}

int8_t dueServoIndexFromPin(uint8_t pin) {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    if (SERVO_PINS[i] == pin) {
      return (int8_t)i;
    }
  }

  return -1;
}

void setNanoServoCommand(uint8_t servoNumber, int16_t value) {
  int8_t limitedValue = (int8_t)constrain(value, SIGNED_COMMAND_MIN, SIGNED_COMMAND_MAX);

  if (servoNumber == 1) {
    loraSignedValue1 = limitedValue;
  } else if (servoNumber == 2) {
    loraSignedValue2 = limitedValue;
  } else if (servoNumber == 3) {
    loraSignedValue3 = limitedValue;
  } else if (servoNumber == 4) {
    loraSignedValue4 = limitedValue;
  }

  loraArbitraryByte = 0;
}

void setNanoEscCommand(uint8_t escNumber, int16_t value) {
  uint8_t limitedValue = (uint8_t)constrain(value, UNSIGNED_COMMAND_MIN, UNSIGNED_COMMAND_MAX);

  if (escNumber == 1) {
    loraUnsignedValue1 = limitedValue;
  } else if (escNumber == 2) {
    loraUnsignedValue2 = limitedValue;
  }

  loraArbitraryByte = 0;
}

void setAllSafe() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    command.servoUs[i] = SERVO_CENTER_US;
  }

  command.motorPower = 0;

  loraSignedValue1 = 0;
  loraSignedValue2 = 0;
  loraSignedValue3 = 0;
  loraSignedValue4 = 0;

  loraUnsignedValue1 = 0;
  loraUnsignedValue2 = 0;

  // 255 can be used by the Nano as a failsafe/status marker.
  loraArbitraryByte = 255;
}

void centerEverything() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    command.servoUs[i] = SERVO_CENTER_US;
  }

  command.motorPower = 0;

  loraSignedValue1 = 0;
  loraSignedValue2 = 0;
  loraSignedValue3 = 0;
  loraSignedValue4 = 0;

  loraUnsignedValue1 = 0;
  loraUnsignedValue2 = 0;

  loraArbitraryByte = 0;
}

// ------------------------------------------------------------
// Printing
// ------------------------------------------------------------

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  DS <index> <us>       Due servo by index, index 0-6, us 1000-2000");
  Serial.println("  DP <pin> <us>         Due servo by pin: 39,41,43,45,47,49,51");
  Serial.println("  M <power>             Due DC motor power, -255 to 255");
  Serial.println("  NS <servo> <value>    Nano servo command, servo 1-4, value -127 to 127");
  Serial.println("  E <esc> <value>       Nano ESC command, ESC 1-2, value 0 to 255");
  Serial.println("  CENTER                Center all servos and stop all motors");
  Serial.println("  SAFE or STOP          Center servos, stop motors, send failsafe byte");
  Serial.println("  P                     Print current output values");
  Serial.println("  ?                     Show this help");
  Serial.println();
  Serial.println("Examples:");
  Serial.println("  DP 43 1600");
  Serial.println("  DS 0 1500");
  Serial.println("  M 80");
  Serial.println("  M -80");
  Serial.println("  NS 1 40");
  Serial.println("  NS 2 -40");
  Serial.println("  E 1 25");
  Serial.println("  E 2 25");
  Serial.println();
}

void printStatus() {
  Serial.println("Current outputs:");

  Serial.print("  Due servos us: ");
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    Serial.print("[");
    Serial.print(i);
    Serial.print("/pin ");
    Serial.print(SERVO_PINS[i]);
    Serial.print("=");
    Serial.print(command.servoUs[i]);
    Serial.print("] ");
  }
  Serial.println();

  Serial.print("  Due DC motor: ");
  Serial.println(command.motorPower);

  Serial.print("  Nano servos signed: ");
  Serial.print("S1=");
  Serial.print(loraSignedValue1);
  Serial.print(" S2=");
  Serial.print(loraSignedValue2);
  Serial.print(" S3=");
  Serial.print(loraSignedValue3);
  Serial.print(" S4=");
  Serial.println(loraSignedValue4);

  Serial.print("  Nano ESC unsigned: ");
  Serial.print("E1=");
  Serial.print(loraUnsignedValue1);
  Serial.print(" E2=");
  Serial.println(loraUnsignedValue2);

  Serial.print("  LoRa: ");
  Serial.println(loraOk ? "OK" : "FAILED");
  Serial.println();
}