/*
  Project Morpheus - manual helicopter / collective test V3 - 6-byte LoRa packet

  Purpose
  -------
  - Unlock the rotating wing.
  - Control BOTH wing motors directly from the normal throttle stick.
  - Control BOTH flaperons as collective blade pitch from the right stick up/down.
  - Measure and print the REAL WING RPM from the geared AS5600 encoder.
  - Gather throttle / collective / RPM data for a future RPM feed-forward term.

  Hardware used
  -------------
  Arduino Due:
    CH2 / right-stick vertical receiver signal -> pin 46
    CH3 / throttle receiver signal            -> pin 44
    Wing unlock servo                         -> pin 45, 850 us
    AS5600                                    -> Wire1 / I2C1
    SX1278 LoRa                               -> existing Due wiring

  Nano, through the compact 6-byte LoRa packet:
    byte 0 -> left flaperon
    byte 1 -> right flaperon
    byte 2 -> left wing ESC
    byte 3 -> right wing ESC
    byte 4 -> special/status value
    byte 5 -> XOR checksum of bytes 0...4

  Important RPM correction
  ------------------------
  The AS5600 is NOT 1:1 with the wing.
  The encoder rotates 1.34375 turns for every 1 wing turn.

      wing RPM = encoder RPM / 1.34375

  Safety behaviour
  ----------------
  1. Motor command ALWAYS starts at zero.
  2. Motor control follows the same proven path as the final V1 airplane code:
       fresh CH3 -> 0...255 throttle command -> the two motor bytes.
  3. If CH2 or CH3 is lost for more than 100 ms:
       - both motor commands immediately become zero
       - collective returns to neutral
       - special byte 4 is sent as 255 (receiver-failsafe marker)
  4. RPM measurement NEVER controls throttle. It is measurement only.
  5. There is no PID, feedforward, autonomous throttle, or extra arming gate.
  6. LoRa packets are repeated at 50 Hz.
  7. The existing Nano code also has its own lost-LoRa-packet ESC failsafe.

  Serial Monitor
  --------------
  115200 baud.

  Output at 10 Hz:
    ms,wing_rpm,throttle_input,throttle_pct,motor_cmd,
    flap_cmd,flap_pct,ch2_us,ch3_us,rc_fresh,encoder_ok

  "flap_cmd" is the collective COMMAND in the existing -127...+127 scale.
  It is not an angle in degrees, because there is no flap-position sensor
  or calibrated flap-angle conversion in the current hardware.
*/

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>
#include <math.h>

// ============================================================
// Basic hardware configuration
// ============================================================

const uint32_t USB_BAUD = 115200;

// Only the two receiver channels needed by this test.
const uint8_t RC_PITCH_PIN    = 46;  // CH2, right stick up/down
const uint8_t RC_THROTTLE_PIN = 44;  // CH3, throttle

const uint16_t RC_MIN_VALID_US = 800;
const uint16_t RC_MAX_VALID_US = 2200;
const uint32_t RC_TIMEOUT_US   = 100000; // 100 ms

// Same calibration / direction as the normal aircraft code.
const uint16_t RC_PITCH_MIN_US    = 1000;
const uint16_t RC_PITCH_CENTER_US = 1500;
const uint16_t RC_PITCH_MAX_US    = 2000;

// Normal aircraft throttle is reversed:
// 1000 us = full throttle
// 2000 us = throttle off
const uint16_t RC_THROTTLE_FULL_US = 1000;
const uint16_t RC_THROTTLE_OFF_US  = 2000;

// Small center deadband for collective.
const int16_t COLLECTIVE_DEADBAND_BYTE = 1;

// If collective moves the wrong overall direction, change 1 to -1.
// This reverses the collective input without changing the mirrored servo setup.
const int8_t COLLECTIVE_INPUT_DIRECTION = 1;

// Helicopter-mode collective directions.
//
// Airplane mode uses opposite final signs because the two servos are mirrored.
// In helicopter mode the LEFT wing is physically flipped 180 degrees,
// so the left flaperon gets one additional reversal.
// For collective pitch this makes both LoRa flap commands use the same sign.
const int8_t LEFT_FLAPERON_DIRECTION  = -1;
const int8_t RIGHT_FLAPERON_DIRECTION = -1;

// Wing unlock servo, same value used in the previous rotor tests.
const uint8_t WING_UNLOCK_SERVO_PIN = 45;
const uint16_t WING_UNLOCK_SERVO_US = 850;

// ============================================================
// AS5600 and RPM measurement
// ============================================================

const uint32_t I2C1_SPEED_HZ = 400000;
const uint8_t AS5600_ADDRESS = 0x36;
const uint8_t AS5600_RAW_ANGLE_HIGH_REGISTER = 0x0C;

// NEW mechanical ratio:
// encoder revolutions / wing revolutions
const float ENCODER_TO_WING_RATIO = 1.34375f;

// Read the encoder rapidly, but calculate RPM over a much longer window.
// This is intentionally smooth because the wing has high rotational inertia.
const uint32_t ENCODER_SAMPLE_PERIOD_US = 5000;   // 200 Hz angle sampling
const uint32_t RPM_WINDOW_US            = 100000; // 100 ms RPM measurement window

// Additional low-pass filter on the 100 ms RPM measurements.
// Lower = smoother/slower. 0.25 is deliberately calm for this test.
const float RPM_FILTER_ALPHA = 0.25f;

// Ignore tiny residual motion/noise near zero.
const float RPM_ZERO_DEADBAND = 0.5f;

// A 5 ms encoder step larger than this is implausible for the intended test
// and is treated as a bad sample instead of creating an RPM spike.
const float MAX_VALID_SAMPLE_DELTA_DEG = 90.0f;

// ============================================================
// LoRa configuration - same as the existing aircraft system
// ============================================================

const long LORA_FREQUENCY_HZ = 433000000L;
const uint8_t LORA_SS_PIN   = 10;
const uint8_t LORA_RST_PIN  = 9;
const uint8_t LORA_DIO0_PIN = 2;

const uint8_t LORA_TX_POWER_DBM = 17;
const uint8_t LORA_SPREADING_FACTOR = 7;
const long LORA_BANDWIDTH_HZ = 500000L;
const uint8_t LORA_CODING_RATE_DENOMINATOR = 5;

const uint8_t LORA_PACKET_BYTES = 6;
const uint32_t LORA_SEND_PERIOD_US = 20000; // 50 Hz
const uint32_t LORA_TX_TIMEOUT_US  = 250000;

// ============================================================
// Serial timing
// ============================================================

const uint32_t SERIAL_PERIOD_MS = 100; // 10 Hz

// ============================================================
// RC interrupt state
// ============================================================

volatile uint32_t pitchRiseUs = 0;
volatile uint16_t pitchPulseUs = 1500;
volatile uint32_t pitchLastValidUs = 0;

volatile uint32_t throttleRiseUs = 0;
volatile uint16_t throttlePulseUs = 2000;
volatile uint32_t throttleLastValidUs = 0;

// Snapshot used by the main loop.
uint16_t currentPitchUs = 1500;
uint16_t currentThrottleUs = 2000;
bool rcFresh = false;

// ============================================================
// Command state
// ============================================================

Servo wingUnlockServo;

uint8_t throttleInputCommand = 0; // requested by stick, 0...255
uint8_t motorCommand = 0;         // actually sent to both ESCs

int16_t collectiveCommand = 0;    // -127...+127
int8_t leftFlaperonCommand = 0;
int8_t rightFlaperonCommand = 0;

// ============================================================
// Encoder / RPM state
// ============================================================

bool encoderOk = false;
bool rpmInitialized = false;
bool rpmValid = false;

float previousEncoderAngleDeg = 0.0f;
float accumulatedEncoderDeg = 0.0f;
uint32_t rpmWindowStartUs = 0;

float rawWingRpm = 0.0f;
float filteredWingRpm = 0.0f;

uint32_t previousEncoderSampleUs = 0;

// ============================================================
// LoRa state
// ============================================================

bool loraOk = false;
volatile bool loraTxBusy = false;
uint32_t lastLoraTxStartUs = 0;
uint32_t previousLoraSendUs = 0;

uint8_t loraPacket[LORA_PACKET_BYTES];

// ============================================================
// Serial state
// ============================================================

uint32_t previousSerialMs = 0;

// ============================================================
// Function declarations
// ============================================================

void pitchReceiverInterrupt();
void throttleReceiverInterrupt();
void updateRcSnapshot(uint32_t nowUs);
void updateManualCommands(uint32_t nowUs);

uint8_t rcThrottleToUnsignedCommand(uint16_t pulseUs);
int16_t rcPitchToCollectiveCommand(uint16_t pulseUs);

bool readAs5600Angle(float &angleDeg);
void serviceRpmMeasurement(uint32_t nowUs);
void resetRpmMeasurement(uint32_t nowUs);

bool setupLoRaRadio();
void onLoRaTxDone();
void serviceLoRaTxTimeout();
void buildLoRaPacket(uint8_t packet[LORA_PACKET_BYTES]);
bool sendLoRaPacket(const uint8_t packet[LORA_PACKET_BYTES]);
uint8_t xorChecksum(const uint8_t *buffer, uint8_t length);

void printTelemetry();

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(USB_BAUD);

  // ----------------------------------------------------------
  // First priority: safe motor state.
  // ----------------------------------------------------------
  throttleInputCommand = 0;
  motorCommand = 0;
  collectiveCommand = 0;
  leftFlaperonCommand = 0;
  rightFlaperonCommand = 0;

  // ----------------------------------------------------------
  // Receiver inputs.
  // ----------------------------------------------------------
  pinMode(RC_PITCH_PIN, INPUT);
  pinMode(RC_THROTTLE_PIN, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(RC_PITCH_PIN),
    pitchReceiverInterrupt,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(RC_THROTTLE_PIN),
    throttleReceiverInterrupt,
    CHANGE
  );

  // ----------------------------------------------------------
  // Unlock the rotating wing.
  // ----------------------------------------------------------
  wingUnlockServo.attach(WING_UNLOCK_SERVO_PIN);
  wingUnlockServo.writeMicroseconds(WING_UNLOCK_SERVO_US);

  // ----------------------------------------------------------
  // AS5600.
  // ----------------------------------------------------------
  Wire1.begin();
  Wire1.setClock(I2C1_SPEED_HZ);

  float firstAngleDeg = 0.0f;
  encoderOk = readAs5600Angle(firstAngleDeg);

  uint32_t nowUs = micros();

  if (encoderOk) {
    previousEncoderAngleDeg = firstAngleDeg;
    rpmInitialized = true;
    rpmWindowStartUs = nowUs;
  } else {
    resetRpmMeasurement(nowUs);
  }

  previousEncoderSampleUs = nowUs;

  // ----------------------------------------------------------
  // LoRa.
  // ----------------------------------------------------------
  loraOk = setupLoRaRadio();

  // Send an explicit zero-power packet immediately if the radio is ready.
  // rcFresh is false here, so special byte 4 is also the failsafe marker.
  if (loraOk) {
    buildLoRaPacket(loraPacket);
    sendLoRaPacket(loraPacket);
  }

  previousLoraSendUs = micros();

  // ----------------------------------------------------------
  // Startup information.
  // ----------------------------------------------------------
  Serial.println();
  Serial.println("Project Morpheus manual helicopter test V3 - 6-byte packet");
  Serial.println("---------------------------------------");
  Serial.println("CH3 throttle: 2000 us = OFF, 1000 us = FULL");
  Serial.println("CH2 right stick: collective flaperon command");
  Serial.println("AS5600 ratio: 1.34375 encoder rev / wing rev");
  Serial.println("Motor output starts at 0.");
  Serial.println("With fresh RC, CH3 directly controls both motors as in airplane V1.");
  Serial.println("Any RC loss immediately sets both motor commands to 0.");
  Serial.print("AS5600: ");
  Serial.println(encoderOk ? "OK" : "NOT DETECTED");
  Serial.print("LoRa: ");
  Serial.println(loraOk ? "OK" : "FAILED");
  Serial.println();
  Serial.println(
    "ms,wing_rpm,throttle_input,throttle_pct,motor_cmd,"
    "flap_cmd,flap_pct,ch2_us,ch3_us,rc_fresh,encoder_ok"
  );
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  uint32_t nowUs = micros();

  // Read a safe snapshot of the two receiver channels.
  updateRcSnapshot(nowUs);

  // Derive motor and collective commands.
  // This function is the ONLY place that can produce a non-zero motor command.
  updateManualCommands(nowUs);

  // RPM measurement is observational only. It never changes motor command.
  if ((uint32_t)(nowUs - previousEncoderSampleUs) >= ENCODER_SAMPLE_PERIOD_US) {
    previousEncoderSampleUs = nowUs;
    serviceRpmMeasurement(nowUs);
  }

  // Keep the radio state machine healthy.
  serviceLoRaTxTimeout();

  // Re-send the current commands at 50 Hz.
  // This includes repeated zero-power packets whenever the RC failsafe is active.
  nowUs = micros();

  if (loraOk &&
      !loraTxBusy &&
      (uint32_t)(nowUs - previousLoraSendUs) >= LORA_SEND_PERIOD_US) {

    previousLoraSendUs = nowUs;
    buildLoRaPacket(loraPacket);
    sendLoRaPacket(loraPacket);
  }

  // Human-readable / loggable telemetry.
  uint32_t nowMs = millis();

  if ((uint32_t)(nowMs - previousSerialMs) >= SERIAL_PERIOD_MS) {
    previousSerialMs = nowMs;
    printTelemetry();
  }
}

// ============================================================
// RC receiver interrupts
// ============================================================

void pitchReceiverInterrupt() {
  uint32_t nowUs = micros();

  if (digitalRead(RC_PITCH_PIN)) {
    pitchRiseUs = nowUs;
  } else {
    uint32_t pulseWidthUs = nowUs - pitchRiseUs;

    if (pulseWidthUs >= RC_MIN_VALID_US &&
        pulseWidthUs <= RC_MAX_VALID_US) {
      pitchPulseUs = (uint16_t)pulseWidthUs;
      pitchLastValidUs = nowUs;
    }
  }
}

void throttleReceiverInterrupt() {
  uint32_t nowUs = micros();

  if (digitalRead(RC_THROTTLE_PIN)) {
    throttleRiseUs = nowUs;
  } else {
    uint32_t pulseWidthUs = nowUs - throttleRiseUs;

    if (pulseWidthUs >= RC_MIN_VALID_US &&
        pulseWidthUs <= RC_MAX_VALID_US) {
      throttlePulseUs = (uint16_t)pulseWidthUs;
      throttleLastValidUs = nowUs;
    }
  }
}

// ============================================================
// RC snapshot and manual command generation
// ============================================================

void updateRcSnapshot(uint32_t nowUs) {
  uint16_t pitchUs;
  uint16_t throttleUs;
  uint32_t pitchAgeReferenceUs;
  uint32_t throttleAgeReferenceUs;

  // Copy multi-byte ISR variables atomically.
  noInterrupts();
  pitchUs = pitchPulseUs;
  throttleUs = throttlePulseUs;
  pitchAgeReferenceUs = pitchLastValidUs;
  throttleAgeReferenceUs = throttleLastValidUs;
  interrupts();

  bool pitchFresh =
    pitchAgeReferenceUs != 0 &&
    (uint32_t)(nowUs - pitchAgeReferenceUs) <= RC_TIMEOUT_US;

  bool throttleFresh =
    throttleAgeReferenceUs != 0 &&
    (uint32_t)(nowUs - throttleAgeReferenceUs) <= RC_TIMEOUT_US;

  currentPitchUs = pitchUs;
  currentThrottleUs = throttleUs;
  rcFresh = pitchFresh && throttleFresh;
}

void updateManualCommands(uint32_t nowUs) {
  (void)nowUs;

  // Receiver loss: immediate hard motor stop.
  if (!rcFresh) {
    throttleInputCommand = 0;
    motorCommand = 0;

    collectiveCommand = 0;
    leftFlaperonCommand = 0;
    rightFlaperonCommand = 0;
    return;
  }

  // Throttle path intentionally matches the final airplane V1 principle:
  // CH3 2000 us -> 0
  // CH3 1000 us -> 255
  //
  // No PID, no ramp, no feedforward and no extra arming gate.
  throttleInputCommand =
    rcThrottleToUnsignedCommand(currentThrottleUs);

  motorCommand = throttleInputCommand;

  // Collective blade pitch from CH2.
  collectiveCommand =
    rcPitchToCollectiveCommand(currentPitchUs);

  int16_t left =
    collectiveCommand * LEFT_FLAPERON_DIRECTION;

  int16_t right =
    collectiveCommand * RIGHT_FLAPERON_DIRECTION;

  if (left < -127) left = -127;
  if (left > 127) left = 127;
  if (right < -127) right = -127;
  if (right > 127) right = 127;

  leftFlaperonCommand = (int8_t)left;
  rightFlaperonCommand = (int8_t)right;
}

uint8_t rcThrottleToUnsignedCommand(uint16_t pulseUs) {
  // Same mapping used in the normal aircraft code:
  // 1000 us -> 255
  // 2000 us -> 0

  pulseUs = (uint16_t)constrain(
    pulseUs,
    RC_THROTTLE_FULL_US,
    RC_THROTTLE_OFF_US
  );

  int32_t inputRange =
    (int32_t)RC_THROTTLE_OFF_US -
    (int32_t)RC_THROTTLE_FULL_US;

  if (inputRange <= 0) {
    return 0;
  }

  int32_t commandValue =
    ((int32_t)RC_THROTTLE_OFF_US - pulseUs) *
    255L /
    inputRange;

  if (commandValue < 0) commandValue = 0;
  if (commandValue > 255) commandValue = 255;

  return (uint8_t)commandValue;
}

int16_t rcPitchToCollectiveCommand(uint16_t pulseUs) {
  // Same signed-stick convention as the aircraft mixer:
  // 1000 us -> -127
  // 1500 us -> 0
  // 2000 us -> +127

  pulseUs = (uint16_t)constrain(
    pulseUs,
    RC_PITCH_MIN_US,
    RC_PITCH_MAX_US
  );

  int32_t commandValue = 0;

  if (pulseUs <= RC_PITCH_CENTER_US) {
    int32_t inputRange =
      (int32_t)RC_PITCH_CENTER_US -
      (int32_t)RC_PITCH_MIN_US;

    if (inputRange > 0) {
      commandValue =
        -127L +
        ((int32_t)(pulseUs - RC_PITCH_MIN_US) * 127L) /
        inputRange;
    }
  } else {
    int32_t inputRange =
      (int32_t)RC_PITCH_MAX_US -
      (int32_t)RC_PITCH_CENTER_US;

    if (inputRange > 0) {
      commandValue =
        ((int32_t)(pulseUs - RC_PITCH_CENTER_US) * 127L) /
        inputRange;
    }
  }

  commandValue *= COLLECTIVE_INPUT_DIRECTION;

  if (commandValue < -127) commandValue = -127;
  if (commandValue > 127) commandValue = 127;

  if (commandValue >= -COLLECTIVE_DEADBAND_BYTE &&
      commandValue <= COLLECTIVE_DEADBAND_BYTE) {
    commandValue = 0;
  }

  return (int16_t)commandValue;
}

// ============================================================
// AS5600 and deliberately smooth wing RPM
// ============================================================

bool readAs5600Angle(float &angleDeg) {
  Wire1.beginTransmission(AS5600_ADDRESS);
  Wire1.write(AS5600_RAW_ANGLE_HIGH_REGISTER);

  if (Wire1.endTransmission(false) != 0) {
    return false;
  }

  uint8_t bytesRead =
    Wire1.requestFrom(AS5600_ADDRESS, (uint8_t)2);

  if (bytesRead != 2) {
    return false;
  }

  uint8_t highByte = Wire1.read();
  uint8_t lowByte = Wire1.read();

  uint16_t rawAngle =
    ((uint16_t)(highByte & 0x0F) << 8) |
    lowByte;

  angleDeg =
    rawAngle * (360.0f / 4096.0f);

  return true;
}

void serviceRpmMeasurement(uint32_t nowUs) {
  float newAngleDeg = 0.0f;

  if (!readAs5600Angle(newAngleDeg)) {
    encoderOk = false;
    resetRpmMeasurement(nowUs);
    return;
  }

  encoderOk = true;

  if (!rpmInitialized) {
    previousEncoderAngleDeg = newAngleDeg;
    rpmWindowStartUs = nowUs;
    accumulatedEncoderDeg = 0.0f;
    rpmInitialized = true;
    return;
  }

  float deltaDeg =
    newAngleDeg - previousEncoderAngleDeg;

  previousEncoderAngleDeg = newAngleDeg;

  // Unwrap the AS5600's 0...360 degree output.
  if (deltaDeg > 180.0f) {
    deltaDeg -= 360.0f;
  } else if (deltaDeg < -180.0f) {
    deltaDeg += 360.0f;
  }

  // Drop an implausible sample rather than letting it create a visible spike.
  if (fabsf(deltaDeg) > MAX_VALID_SAMPLE_DELTA_DEG) {
    accumulatedEncoderDeg = 0.0f;
    rpmWindowStartUs = nowUs;
    rpmValid = false;
    return;
  }

  accumulatedEncoderDeg += deltaDeg;

  uint32_t windowUs =
    nowUs - rpmWindowStartUs;

  if (windowUs < RPM_WINDOW_US) {
    return;
  }

  float windowSeconds =
    windowUs * 0.000001f;

  // Encoder speed over the entire measurement window.
  float encoderRpm =
    (accumulatedEncoderDeg / 360.0f) *
    (60.0f / windowSeconds);

  // IMPORTANT: convert encoder RPM to actual wing RPM using
  // the new 1.34375:1 mechanical ratio.
  rawWingRpm =
    fabsf(encoderRpm) /
    ENCODER_TO_WING_RATIO;

  if (rawWingRpm < RPM_ZERO_DEADBAND) {
    rawWingRpm = 0.0f;
  }

  if (!rpmValid) {
    filteredWingRpm = rawWingRpm;
    rpmValid = true;
  } else {
    filteredWingRpm +=
      RPM_FILTER_ALPHA *
      (rawWingRpm - filteredWingRpm);
  }

  // Start a new 100 ms measurement window.
  accumulatedEncoderDeg = 0.0f;
  rpmWindowStartUs = nowUs;
}

void resetRpmMeasurement(uint32_t nowUs) {
  rpmInitialized = false;
  rpmValid = false;
  accumulatedEncoderDeg = 0.0f;
  rpmWindowStartUs = nowUs;
  rawWingRpm = 0.0f;
  filteredWingRpm = 0.0f;
}

// ============================================================
// LoRa transmitter
// ============================================================

bool setupLoRaRadio() {
  LoRa.setPins(
    LORA_SS_PIN,
    LORA_RST_PIN,
    LORA_DIO0_PIN
  );

  if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
    return false;
  }

  LoRa.setTxPower(LORA_TX_POWER_DBM);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH_HZ);
  LoRa.setCodingRate4(LORA_CODING_RATE_DENOMINATOR);
  LoRa.enableCrc();

  LoRa.onTxDone(onLoRaTxDone);

  return true;
}

void onLoRaTxDone() {
  loraTxBusy = false;
}

void serviceLoRaTxTimeout() {
  if (!loraTxBusy) {
    return;
  }

  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastLoraTxStartUs) >
      LORA_TX_TIMEOUT_US) {

    loraTxBusy = false;
    LoRa.idle();

    // Send a zero command after a transport timeout.
    // The next valid RC loop can immediately restore the live throttle command.
    motorCommand = 0;
  }
}

void buildLoRaPacket(uint8_t packet[LORA_PACKET_BYTES]) {
  // Compact command packet - 6 bytes total:
  //
  // byte 0 = left flaperon, signed -127...+127 stored as raw int8_t byte
  // byte 1 = right flaperon, signed -127...+127 stored as raw int8_t byte
  // byte 2 = left motor, 0...255
  // byte 3 = right motor, 0...255
  // byte 4 = special/status byte: 0 normal, 255 receiver failsafe
  // byte 5 = XOR checksum of bytes 0...4

  if (rcFresh) {
    packet[0] = (uint8_t)leftFlaperonCommand;
    packet[1] = (uint8_t)rightFlaperonCommand;
  } else {
    packet[0] = 0;
    packet[1] = 0;
  }

  // Both wing motors receive the same direct throttle-stick command.
  packet[2] = motorCommand;
  packet[3] = motorCommand;

  // Preserve the existing special/failsafe convention.
  packet[4] = rcFresh ? 0 : 255;

  // Simple packet-level safety check, in addition to LoRa hardware CRC.
  packet[5] = xorChecksum(packet, 5);
}

bool sendLoRaPacket(const uint8_t packet[LORA_PACKET_BYTES]) {
  if (!loraOk || loraTxBusy) {
    return false;
  }

  LoRa.beginPacket();
  LoRa.write(packet, LORA_PACKET_BYTES);

  loraTxBusy = true;
  lastLoraTxStartUs = micros();

  int result =
    LoRa.endPacket(true);

  if (result == 0) {
    loraTxBusy = false;

    // Treat a transmit-start failure conservatively for this iteration.
    // The next valid RC loop can immediately restore the live throttle command.
    motorCommand = 0;
    return false;
  }

  return true;
}

uint8_t xorChecksum(
  const uint8_t *buffer,
  uint8_t length
) {
  uint8_t checksum = 0;

  for (uint8_t i = 0; i < length; i++) {
    checksum ^= buffer[i];
  }

  return checksum;
}

// ============================================================
// Serial telemetry
// ============================================================

void printTelemetry() {
  float throttlePercent =
    throttleInputCommand * (100.0f / 255.0f);

  float flapPercent =
    collectiveCommand * (100.0f / 127.0f);

  Serial.print(millis());
  Serial.print(',');

  if (rpmValid && encoderOk) {
    Serial.print(filteredWingRpm, 2);
  } else {
    Serial.print("nan");
  }

  Serial.print(',');
  Serial.print(throttleInputCommand);

  Serial.print(',');
  Serial.print(throttlePercent, 1);

  Serial.print(',');
  Serial.print(motorCommand);

  Serial.print(',');
  Serial.print(collectiveCommand);

  Serial.print(',');
  Serial.print(flapPercent, 1);

  Serial.print(',');
  Serial.print(currentPitchUs);

  Serial.print(',');
  Serial.print(currentThrottleUs);

  Serial.print(',');
  Serial.print(rcFresh ? 1 : 0);

  Serial.print(',');
  Serial.println(encoderOk ? 1 : 0);
}