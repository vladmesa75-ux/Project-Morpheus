/*
  Project Morpheus - yaw PIDF bench test

  Purpose:
  - Immediately fully open the wing lock by commanding Due pin 45 to 850 us.
  - Allow selecting the three confirmed lock positions from Serial:
      OPEN = 850 us, PARTIAL = 1300 us (landing failsafe), LOCK = 1950 us.
  - Read fuselage yaw rate from the MPU-6050 and integrate it into a relative yaw angle.
  - Keep the relative yaw angle at 0 degrees with the bidirectional axle DC motor.
  - Measure wing/shaft rpm with the AS5600 encoder, accounting for the
    1.34375:1 encoder-to-wing gearing.
  - Add a feedforward motor term based on measured rotor rpm:

        F = F0 + KF * rotorRpm

  - Allow all controller values to be changed from the Serial Monitor.

  This is deliberately a simple, standalone bench-test sketch.
  It does not run the receiver, LoRa, airplane mixer, flaperons, ESCs, or mode switch.

  Current Project Morpheus Due wiring used here:
  - MPU-6050: Wire / I2C0, address 0x68
  - AS5600: Wire1 / I2C1, address 0x36
  - Wing lock servo: pin 45 (850 open / 1300 partial / 1950 locked)
  - Axle motor H-bridge: pins 4 and 5

  Serial Monitor:
  - Baud: 115200
  - Line ending: Newline

  Start with KP, KI, KD, KF and F0 at zero. The motor therefore stays stopped
  until you enter controller values.
*/

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// ============================================================
// Hardware configuration from the current Due setup
// ============================================================

const uint32_t USB_BAUD = 115200;
const uint32_t I2C0_SPEED_HZ = 400000;
const uint32_t I2C1_SPEED_HZ = 400000;

const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t AS5600_ADDRESS = 0x36;
const uint8_t AS5600_RAW_ANGLE_HIGH_REGISTER = 0x0C;

const uint8_t WING_UNLOCK_SERVO_PIN = 45;
// Calibrated physical positions for the wing lock servo on pin 45.
const int16_t WING_LOCK_OPEN_US = 850;          // Fully open for rotor testing.
const int16_t WING_LOCK_PARTIAL_US = 1300;     // Partial: landing failsafe.
const int16_t WING_LOCK_LOCKED_US = 1950;      // Fully locked.

// Encoder rotates 1.34375 turns for each actual wing/rotor turn.
const float ENCODER_TURNS_PER_WING_TURN = 1.34375f;

const uint8_t MOTOR_PIN_A = 4;
const uint8_t MOTOR_PIN_B = 5;
const int16_t HARD_MOTOR_LIMIT = 255;
const int16_t MOTOR_DEADBAND = 4;

// ============================================================
// Test timing and filtering
// ============================================================

// 5000 us = 200 Hz controller update rate.
const uint32_t CONTROL_PERIOD_US = 5000;

// Print controller data at 10 Hz.
const uint32_t TELEMETRY_PERIOD_MS = 100;

// Number of stationary gyro samples taken at startup.
const uint16_t GYRO_CALIBRATION_SAMPLES = 1000;

// Ignore extremely small residual yaw rates after calibration.
const float YAW_RATE_DEADBAND_DEG_PER_SEC = 0.05f;
const float RAD_TO_DEG_FLOAT = 57.2957795f;

// Low-pass filter for measured encoder rpm.
// 0 = never changes, 1 = no filtering.
const float RPM_FILTER_ALPHA = 0.25f;

// ============================================================
// Controller settings - all can be changed from Serial Monitor
// ============================================================

float kp = 0.0f;
float ki = 0.0f;
float kd = 0.0f;

// Feedforward equation:
// feedForwardTerm = f0 + kf * rotorRpm
float kf = 0.0f;
float f0 = 0.0f;

// Limits are expressed in motor command units, where 255 is full PWM.
float integralLimit = 100.0f;
int16_t outputLimit = 100;

// Direction switches. Use only +1 or -1.
int8_t motorDirection = 1;
int8_t yawDirection = 1;
int8_t encoderDirection = 1;

// ============================================================
// Hardware objects and controller state
// ============================================================

Adafruit_MPU6050 mpu;
Servo wingUnlockServo;
int16_t wingLockCommandUs = WING_LOCK_OPEN_US;

bool imuOk = false;
bool encoderOk = false;

float gyroZBiasRadPerSec = 0.0f;

float yawDeg = 0.0f;
float yawRateDegPerSec = 0.0f;
float targetYawDeg = 0.0f;

float rotorAngleDeg = 0.0f;
float rotorRpm = 0.0f;
float previousRotorAngleDeg = 0.0f;
bool rotorSpeedInitialized = false;

float yawErrorDeg = 0.0f;
float proportionalTerm = 0.0f;
float integralTerm = 0.0f;
float derivativeTerm = 0.0f;
float feedForwardTerm = 0.0f;
float controllerOutput = 0.0f;
int16_t motorCommand = 0;

uint32_t previousControlUs = 0;
uint32_t previousTelemetryMs = 0;

// Serial input buffer.
const uint8_t SERIAL_LINE_LENGTH = 96;
char serialLine[SERIAL_LINE_LENGTH];
uint8_t serialLinePosition = 0;

// Function declarations. Arduino usually generates these automatically,
// but listing them explicitly keeps the sketch clear and portable.
bool setupImu();
void calibrateGyroBias();
bool updateRelativeYaw(float dtSeconds);
bool readAs5600Angle(float &angleDeg);
void updateRotorRpm(float dtSeconds);
void updateController(float dtSeconds);
void zeroYawController();
void stopYawController();
void setWingLockPosition(int16_t pulseUs);
void setupMotor();
void setMotorPower(int16_t power);
void readSerialInput();
void processSerialCommand(char *line);
bool readFloatArgument(float &value);
bool validDirection(float value);
void makeUppercase(char *text);
void printHelp();
void printSettings();
void printTelemetry();
float wrapAngle180(float angleDeg);
float constrainFloat(float value, float minimum, float maximum);

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(USB_BAUD);
  Serial.setTimeout(50);

  analogWriteResolution(8);
  setupMotor();

  // Keep the original bench-test behavior: start with wing lock fully open.
  // All PIDF gains still start at zero, so the axle motor stays stopped.
  wingUnlockServo.attach(WING_UNLOCK_SERVO_PIN);
  wingUnlockServo.writeMicroseconds(wingLockCommandUs);

  Wire.begin();
  Wire.setClock(I2C0_SPEED_HZ);

  Wire1.begin();
  Wire1.setClock(I2C1_SPEED_HZ);

  imuOk = setupImu();

  if (imuOk) {
    calibrateGyroBias();
  }

  // The integrated MPU yaw is relative. Zero means the heading at startup.
  zeroYawController();

  float firstAngle;
  encoderOk = readAs5600Angle(firstAngle);

  if (encoderOk) {
    rotorAngleDeg = firstAngle;
    previousRotorAngleDeg = firstAngle;
    rotorSpeedInitialized = true;
  }

  previousControlUs = micros();

  Serial.println();
  Serial.println("Project Morpheus yaw PIDF bench test ready.");
  Serial.print("Wing lock servo pin 45: OPEN, ");
  Serial.print(wingLockCommandUs);
  Serial.println(" us (PARTIAL=1300, LOCK=1950)");
  Serial.println(imuOk ? "MPU-6050: OK" : "MPU-6050: FAILED");
  Serial.println(encoderOk ? "AS5600: OK" : "AS5600: NOT DETECTED");
  Serial.println("All gains start at zero, so the axle motor starts stopped.");
  printHelp();
  printSettings();
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  readSerialInput();

  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - previousControlUs) >= CONTROL_PERIOD_US) {
    float dtSeconds = (nowUs - previousControlUs) * 0.000001f;
    previousControlUs = nowUs;

    updateController(dtSeconds);
  }

  uint32_t nowMs = millis();

  if ((uint32_t)(nowMs - previousTelemetryMs) >= TELEMETRY_PERIOD_MS) {
    previousTelemetryMs = nowMs;
    printTelemetry();
  }
}

// ============================================================
// MPU-6050 and relative yaw
// ============================================================

bool setupImu() {
  if (!mpu.begin(MPU6050_ADDRESS, &Wire)) {
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  return true;
}

void calibrateGyroBias() {
  Serial.println("Keep the fuselage completely still: calibrating gyro...");

  sensors_event_t acceleration;
  sensors_event_t gyro;
  sensors_event_t temperature;

  double sumZ = 0.0;
  uint16_t goodSamples = 0;

  for (uint16_t i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    mpu.getEvent(&acceleration, &gyro, &temperature);
    sumZ += gyro.gyro.z;
    goodSamples++;
    delay(2);
  }

  if (goodSamples > 0) {
    gyroZBiasRadPerSec = (float)(sumZ / goodSamples);
  }

  Serial.print("Gyro Z bias: ");
  Serial.print(gyroZBiasRadPerSec, 7);
  Serial.println(" rad/s");
}

bool updateRelativeYaw(float dtSeconds) {
  if (!imuOk || dtSeconds <= 0.0f || dtSeconds > 0.1f) {
    return false;
  }

  sensors_event_t acceleration;
  sensors_event_t gyro;
  sensors_event_t temperature;
  mpu.getEvent(&acceleration, &gyro, &temperature);

  yawRateDegPerSec =
    (gyro.gyro.z - gyroZBiasRadPerSec) * RAD_TO_DEG_FLOAT * yawDirection;

  if (yawRateDegPerSec > -YAW_RATE_DEADBAND_DEG_PER_SEC &&
      yawRateDegPerSec < YAW_RATE_DEADBAND_DEG_PER_SEC) {
    yawRateDegPerSec = 0.0f;
  }

  yawDeg += yawRateDegPerSec * dtSeconds;
  yawDeg = wrapAngle180(yawDeg);

  return true;
}

// ============================================================
// AS5600 angle and rotor rpm
// ============================================================

bool readAs5600Angle(float &angleDeg) {
  Wire1.beginTransmission(AS5600_ADDRESS);
  Wire1.write(AS5600_RAW_ANGLE_HIGH_REGISTER);

  if (Wire1.endTransmission(false) != 0) {
    return false;
  }

  uint8_t bytesRead = Wire1.requestFrom(AS5600_ADDRESS, (uint8_t)2);

  if (bytesRead != 2) {
    return false;
  }

  uint8_t highByte = Wire1.read();
  uint8_t lowByte = Wire1.read();

  uint16_t rawAngle =
    ((uint16_t)(highByte & 0x0F) << 8) | lowByte;

  angleDeg = rawAngle * (360.0f / 4096.0f);
  return true;
}

void updateRotorRpm(float dtSeconds) {
  float newAngleDeg;
  encoderOk = readAs5600Angle(newAngleDeg);

  if (!encoderOk || dtSeconds <= 0.0f || dtSeconds > 0.1f) {
    // Do not keep applying feedforward from an old rpm value.
    // Re-initialize the angle delta when the encoder resumes so a gap
    // cannot produce one spurious very high RPM reading.
    rotorRpm = 0.0f;
    rotorSpeedInitialized = false;
    return;
  }

  rotorAngleDeg = newAngleDeg;

  if (!rotorSpeedInitialized) {
    previousRotorAngleDeg = newAngleDeg;
    rotorSpeedInitialized = true;
    rotorRpm = 0.0f;
    return;
  }

  float angleChangeDeg = newAngleDeg - previousRotorAngleDeg;
  previousRotorAngleDeg = newAngleDeg;

  // Correct the 359 -> 0 and 0 -> 359 wraparound.
  if (angleChangeDeg > 180.0f) {
    angleChangeDeg -= 360.0f;
  } else if (angleChangeDeg < -180.0f) {
    angleChangeDeg += 360.0f;
  }

  // AS5600 reads encoder-shaft speed; divide to obtain actual wing RPM.
  float newRpm =
    encoderDirection * (angleChangeDeg / 360.0f) *
    (60.0f / dtSeconds) / ENCODER_TURNS_PER_WING_TURN;

  rotorRpm += RPM_FILTER_ALPHA * (newRpm - rotorRpm);
}

// ============================================================
// PIDF controller
// ============================================================

void updateController(float dtSeconds) {
  if (!updateRelativeYaw(dtSeconds)) {
    setMotorPower(0);
    motorCommand = 0;
    return;
  }

  updateRotorRpm(dtSeconds);

  // Hardware interlock for this bench sketch: axle motor cannot be driven
  // while the wing lock is commanded PARTIAL or LOCKED.
  if (wingLockCommandUs != WING_LOCK_OPEN_US) {
    integralTerm = 0.0f;
    controllerOutput = 0.0f;
    motorCommand = 0;
    setMotorPower(0);
    return;
  }

  // Target is always zero for this test.
  yawErrorDeg = wrapAngle180(targetYawDeg - yawDeg);

  proportionalTerm = kp * yawErrorDeg;

  integralTerm += ki * yawErrorDeg * dtSeconds;
  integralTerm = constrainFloat(
    integralTerm,
    -integralLimit,
    integralLimit
  );

  // Because target yaw is fixed, derivative of error is the negative yaw rate.
  // Using the gyro rate directly is cleaner than differentiating the angle.
  derivativeTerm = -kd * yawRateDegPerSec;

  // Requested velocity-based feedforward.
  feedForwardTerm = f0 + kf * rotorRpm;

  controllerOutput =
    proportionalTerm +
    integralTerm +
    derivativeTerm +
    feedForwardTerm;

  controllerOutput *= motorDirection;
  controllerOutput = constrainFloat(
    controllerOutput,
    -(float)outputLimit,
    (float)outputLimit
  );

  motorCommand = (int16_t)round(controllerOutput);
  setMotorPower(motorCommand);
}

void zeroYawController() {
  yawDeg = 0.0f;
  targetYawDeg = 0.0f;
  yawErrorDeg = 0.0f;
  integralTerm = 0.0f;
  proportionalTerm = 0.0f;
  derivativeTerm = 0.0f;
  feedForwardTerm = 0.0f;
  controllerOutput = 0.0f;
}

void stopYawController() {
  kp = 0.0f;
  ki = 0.0f;
  kd = 0.0f;
  kf = 0.0f;
  f0 = 0.0f;
  zeroYawController();
  motorCommand = 0;
  setMotorPower(0);
}

void setWingLockPosition(int16_t pulseUs) {
  // Safety: moving the lock stops the motor and clears controller settings.
  // This prevents a motor command resuming unexpectedly after reopening.
  stopYawController();
  wingUnlockServo.writeMicroseconds(pulseUs);
  wingLockCommandUs = pulseUs;
}

// ============================================================
// Bidirectional H-bridge motor output
// ============================================================

void setupMotor() {
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);

  analogWrite(MOTOR_PIN_A, 0);
  analogWrite(MOTOR_PIN_B, 0);
}

void setMotorPower(int16_t power) {
  power = constrain(
    power,
    (int16_t)-HARD_MOTOR_LIMIT,
    (int16_t)HARD_MOTOR_LIMIT
  );

  if (power > -MOTOR_DEADBAND && power < MOTOR_DEADBAND) {
    power = 0;
  }

  if (power > 0) {
    analogWrite(MOTOR_PIN_A, power);
    analogWrite(MOTOR_PIN_B, 0);
  } else if (power < 0) {
    analogWrite(MOTOR_PIN_A, 0);
    analogWrite(MOTOR_PIN_B, -power);
  } else {
    analogWrite(MOTOR_PIN_A, 0);
    analogWrite(MOTOR_PIN_B, 0);
  }
}

// ============================================================
// Serial command handling
// ============================================================

void readSerialInput() {
  while (Serial.available() > 0) {
    char character = (char)Serial.read();

    if (character == '\r') {
      continue;
    }

    if (character == '\n') {
      serialLine[serialLinePosition] = '\0';
      processSerialCommand(serialLine);
      serialLinePosition = 0;
      continue;
    }

    if (serialLinePosition < SERIAL_LINE_LENGTH - 1) {
      serialLine[serialLinePosition++] = character;
    }
  }
}

void processSerialCommand(char *line) {
  char *command = strtok(line, " ,\t");

  if (command == NULL) {
    return;
  }

  makeUppercase(command);

  if (strcmp(command, "HELP") == 0 || strcmp(command, "?") == 0) {
    printHelp();
    return;
  }

  if (strcmp(command, "P") == 0 || strcmp(command, "PRINT") == 0) {
    printSettings();
    return;
  }

  if (strcmp(command, "ZERO") == 0) {
    zeroYawController();
    Serial.println("OK: current fuselage heading is now yaw 0");
    return;
  }

  if (strcmp(command, "STOP") == 0) {
    stopYawController();
    Serial.println("OK: all PIDF gains set to zero and motor stopped");
    return;
  }

  if (strcmp(command, "OPEN") == 0 ||
      strcmp(command, "PARTIAL") == 0 ||
      strcmp(command, "LOCK") == 0) {
    int16_t pulseUs = WING_LOCK_OPEN_US;
    if (strcmp(command, "PARTIAL") == 0) pulseUs = WING_LOCK_PARTIAL_US;
    if (strcmp(command, "LOCK") == 0) pulseUs = WING_LOCK_LOCKED_US;
    setWingLockPosition(pulseUs);
    Serial.print("OK: wing lock commanded to ");
    Serial.print(pulseUs);
    Serial.println(" us. Axle motor stopped; re-enter PIDF to restart.");
    return;
  }

  if (strcmp(command, "PIDF") == 0) {
    float newKp;
    float newKi;
    float newKd;
    float newKf;
    float newF0;

    if (!readFloatArgument(newKp) ||
        !readFloatArgument(newKi) ||
        !readFloatArgument(newKd) ||
        !readFloatArgument(newKf) ||
        !readFloatArgument(newF0)) {
      Serial.println("ERROR: use PIDF <KP> <KI> <KD> <KF> <F0>");
      return;
    }

    kp = newKp;
    ki = newKi;
    kd = newKd;
    kf = newKf;
    f0 = newF0;
    integralTerm = 0.0f;

    Serial.println("OK: PIDF values updated");
    printSettings();
    return;
  }

  float newValue;

  if (strcmp(command, "KP") == 0) {
    if (!readFloatArgument(newValue)) {
      Serial.println("ERROR: use KP <value>");
      return;
    }
    kp = newValue;
  } else if (strcmp(command, "KI") == 0) {
    if (!readFloatArgument(newValue)) {
      Serial.println("ERROR: use KI <value>");
      return;
    }
    ki = newValue;
    integralTerm = 0.0f;
  } else if (strcmp(command, "KD") == 0) {
    if (!readFloatArgument(newValue)) {
      Serial.println("ERROR: use KD <value>");
      return;
    }
    kd = newValue;
  } else if (strcmp(command, "KF") == 0) {
    if (!readFloatArgument(newValue)) {
      Serial.println("ERROR: use KF <value>");
      return;
    }
    kf = newValue;
  } else if (strcmp(command, "F0") == 0) {
    if (!readFloatArgument(newValue)) {
      Serial.println("ERROR: use F0 <value>");
      return;
    }
    f0 = newValue;
  } else if (strcmp(command, "ILIM") == 0) {
    if (!readFloatArgument(newValue)) {
      Serial.println("ERROR: use ILIM <0 to 255>");
      return;
    }
    integralLimit = constrainFloat(newValue, 0.0f, 255.0f);
    integralTerm = constrainFloat(
      integralTerm,
      -integralLimit,
      integralLimit
    );
  } else if (strcmp(command, "MAX") == 0) {
    if (!readFloatArgument(newValue)) {
      Serial.println("ERROR: use MAX <0 to 255>");
      return;
    }
    outputLimit = (int16_t)constrain((int)round(newValue), 0, 255);
  } else if (strcmp(command, "MDIR") == 0) {
    if (!readFloatArgument(newValue) || !validDirection(newValue)) {
      Serial.println("ERROR: use MDIR 1 or MDIR -1");
      return;
    }
    motorDirection = (int8_t)newValue;
  } else if (strcmp(command, "YDIR") == 0) {
    if (!readFloatArgument(newValue) || !validDirection(newValue)) {
      Serial.println("ERROR: use YDIR 1 or YDIR -1");
      return;
    }
    yawDirection = (int8_t)newValue;
    zeroYawController();
  } else if (strcmp(command, "EDIR") == 0) {
    if (!readFloatArgument(newValue) || !validDirection(newValue)) {
      Serial.println("ERROR: use EDIR 1 or EDIR -1");
      return;
    }
    encoderDirection = (int8_t)newValue;
    rotorRpm = 0.0f;
  } else {
    Serial.println("ERROR: unknown command. Type HELP");
    return;
  }

  Serial.println("OK");
  printSettings();
}

bool readFloatArgument(float &value) {
  char *argument = strtok(NULL, " ,\t");

  if (argument == NULL) {
    return false;
  }

  char *endPointer;
  value = strtof(argument, &endPointer);

  return endPointer != argument && *endPointer == '\0';
}

bool validDirection(float value) {
  return value == 1.0f || value == -1.0f;
}

void makeUppercase(char *text) {
  while (*text != '\0') {
    *text = (char)toupper(*text);
    text++;
  }
}

// ============================================================
// Serial printing
// ============================================================

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  PIDF <KP> <KI> <KD> <KF> <F0>   Set all controller values");
  Serial.println("  KP <value>                       Set proportional gain");
  Serial.println("  KI <value>                       Set integral gain");
  Serial.println("  KD <value>                       Set derivative gain");
  Serial.println("  KF <value>                       Set rpm feedforward gain");
  Serial.println("  F0 <value>                       Set constant feedforward offset");
  Serial.println("  MAX <0-255>                      Set absolute motor output limit");
  Serial.println("  ILIM <0-255>                     Set integral-term limit");
  Serial.println("  MDIR 1 or -1                     Reverse complete motor output");
  Serial.println("  YDIR 1 or -1                     Reverse MPU yaw direction");
  Serial.println("  EDIR 1 or -1                     Reverse encoder/rpm direction");
  Serial.println("  ZERO                              Set current heading to yaw 0");
  Serial.println("  OPEN                              Wing lock fully open (850 us)");
  Serial.println("  PARTIAL                           Landing failsafe (1300 us)");
  Serial.println("  LOCK                              Wing fully locked (1950 us)");
  Serial.println("                                    Lock commands also stop the motor");
  Serial.println("                                    and reset PIDF to zero");
  Serial.println("  STOP                              Zero every gain and stop motor");
  Serial.println("  P                                 Print current settings");
  Serial.println("  HELP or ?                         Print this command list");
  Serial.println();
  Serial.println("Feedforward equation: F = F0 + KF * rotorRpm");
  Serial.println();
}

void printSettings() {
  Serial.print("SETTINGS: KP=");
  Serial.print(kp, 5);
  Serial.print(" KI=");
  Serial.print(ki, 5);
  Serial.print(" KD=");
  Serial.print(kd, 5);
  Serial.print(" KF=");
  Serial.print(kf, 5);
  Serial.print(" F0=");
  Serial.print(f0, 3);
  Serial.print(" MAX=");
  Serial.print(outputLimit);
  Serial.print(" ILIM=");
  Serial.print(integralLimit, 1);
  Serial.print(" MDIR=");
  Serial.print(motorDirection);
  Serial.print(" YDIR=");
  Serial.print(yawDirection);
  Serial.print(" EDIR=");
  Serial.print(encoderDirection);
  Serial.print(" WING_LOCK_US=");
  Serial.println(wingLockCommandUs);
}

void printTelemetry() {
  Serial.print("yaw=");
  Serial.print(yawDeg, 2);
  Serial.print(",rate=");
  Serial.print(yawRateDegPerSec, 2);
  Serial.print(",rpm=");
  Serial.print(rotorRpm, 2);
  Serial.print(",error=");
  Serial.print(yawErrorDeg, 2);
  Serial.print(",P=");
  Serial.print(proportionalTerm, 2);
  Serial.print(",I=");
  Serial.print(integralTerm, 2);
  Serial.print(",D=");
  Serial.print(derivativeTerm, 2);
  Serial.print(",F=");
  Serial.print(feedForwardTerm, 2);
  Serial.print(",out=");
  Serial.print(motorCommand);
  Serial.print(",enc=");
  Serial.print(encoderOk ? "OK" : "ERR");
  Serial.print(",wingLockUs=");
  Serial.println(wingLockCommandUs);
}

// ============================================================
// Small helpers
// ============================================================

float wrapAngle180(float angleDeg) {
  while (angleDeg > 180.0f) {
    angleDeg -= 360.0f;
  }

  while (angleDeg < -180.0f) {
    angleDeg += 360.0f;
  }

  return angleDeg;
}

float constrainFloat(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }

  if (value > maximum) {
    return maximum;
  }

  return value;
}