#ifndef TYPES_AND_GLOBALS_H
#define TYPES_AND_GLOBALS_H

/*
  TypesAndGlobals.h

  This tab contains:
  - small data structures used by the flight computer
  - global variables shared between tabs

  Global variables are values that several tabs need to access.
*/

struct RcFrame {
  uint16_t us[RC_CHANNEL_COUNT];
  bool fresh;
  uint32_t maxAgeUs;
};

struct ImuData {
  // Accelerometer values in m/s^2.
  float ax;
  float ay;
  float az;

  // Gyro values in rad/s after bias correction.
  float gx;
  float gy;
  float gz;

  float tempC;
};

struct AttitudeData {
  // Roll, pitch, and yaw in degrees.
  float rollDeg;
  float pitchDeg;
  float yawDeg;
};

struct EncoderData {
  uint16_t raw;
  float deg;
};

struct FlightCommand {
  // These are the actuator values you can change in userFlightLogic().
  int16_t servoUs[SERVO_COUNT];
  int16_t motorPower;
};

// Hardware library objects.
Adafruit_MPU6050 mpu;
Servo servos[SERVO_COUNT];

// Receiver values measured inside interrupts.
volatile uint32_t rcRiseUs[RC_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0};
volatile uint16_t rcPulseUs[RC_CHANNEL_COUNT] = {1500, 1500, 1500, 1500, 1500, 1500};
volatile uint32_t rcLastEdgeUs[RC_CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0};

// Main shared data.
RcFrame rc;
ImuData imu;
AttitudeData attitude;
EncoderData encoder;
FlightCommand command;

// Health flags.
bool imuOk = false;
bool encoderOk = false;
bool loraOk = false;

bool lastImuReadOk = false;
bool lastEncoderReadOk = false;

// Gyro calibration values.
float gyroXBiasRad = 0.0f;
float gyroYBiasRad = 0.0f;
float gyroZBiasRad = 0.0f;
bool gyroCalibrated = false;

// Attitude filter state.
bool attitudeInitialized = false;
uint32_t lastImuUpdateUs = 0;

// Custom variables for the LoRa packet.
// You will write these later in userFlightLogic().
// Keep signed values between -127 and +127.
int8_t loraSignedValue1 = 0;
int8_t loraSignedValue2 = 0;
int8_t loraSignedValue3 = 0;
int8_t loraSignedValue4 = 0;

// Keep unsigned values between 0 and 255.
uint8_t loraUnsignedValue1 = 0;
uint8_t loraUnsignedValue2 = 0;

// Extra free byte for arbitrary communication between the two boards.
// You can set this later in userFlightLogic().
uint8_t loraArbitraryByte = 0;

// LoRa TX state.
volatile bool loraTxBusy = false;
uint32_t lastLoraTxStartUs = 0;
uint8_t telemetryPacket[LORA_PACKET_BYTES];
uint32_t packetsSentThisSecond = 0;

// Loop timing.
uint32_t previousLoopStartUs = 0;
uint32_t loopTimeUs = 0;
float loopHz = 0.0f;

// Timers for no-delay logging/debug.
uint32_t lastOpenLogUs = 0;
uint32_t lastUsbDebugUs = 0;
uint32_t lastPacketRateCountMs = 0;
uint32_t lastPacketRatePrintMs = 0;
uint32_t lastPacketRateValue = 0;

#endif