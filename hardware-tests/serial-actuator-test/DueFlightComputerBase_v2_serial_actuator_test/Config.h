#ifndef CONFIG_H
#define CONFIG_H

/*
  Config.h

  This tab contains the values you are most likely to change:
  - pin numbers
  - baud rates
  - LoRa settings
  - logging settings
  - aircraft control tuning
*/

// ------------------------------------------------------------
// Serial speeds
// ------------------------------------------------------------

const uint32_t USB_BAUD = 115200;
const uint32_t OPENLOG_BAUD = 115200;

// ------------------------------------------------------------
// I2C speeds
// ------------------------------------------------------------

const uint32_t I2C0_SPEED_HZ = 400000;  // MPU-6050 on Wire
const uint32_t I2C1_SPEED_HZ = 400000;  // AS5600 on Wire1

// ------------------------------------------------------------
// MPU-6050 settings
// ------------------------------------------------------------

const uint8_t MPU6050_ADDRESS = 0x68;

// Set true to measure gyro drift at startup.
// Keep the aircraft perfectly still while the board boots.
const bool ENABLE_GYRO_CALIBRATION = true;
const uint16_t GYRO_CALIBRATION_SAMPLES = 1000;

// Manual yaw trim in degrees per second.
// If yaw still drifts after calibration, adjust this value.
const float GYRO_Z_BIAS_TRIM_DEG_PER_SEC = 0.0f;

// Small gyro yaw values below this are ignored.
// This helps stop tiny still-aircraft yaw drift.
const float YAW_RATE_DEADBAND_DEG_PER_SEC = 0.05f;

// Complementary filter amount.
// Higher value trusts the gyro more; lower value trusts the accelerometer more.
const float ATTITUDE_ALPHA = 0.98f;
const float RAD_TO_DEG_FLOAT = 57.2957795f;

// ------------------------------------------------------------
// AS5600 settings
// ------------------------------------------------------------

const uint8_t AS5600_ADDRESS = 0x36;
const uint8_t AS5600_RAW_ANGLE_HIGH_REGISTER = 0x0C;

// ------------------------------------------------------------
// LoRa SX1278 settings
// ------------------------------------------------------------

const long LORA_FREQUENCY_HZ = 433000000L;

// Standard wiring for the Sandeep Mistry LoRa library.
const uint8_t LORA_SS_PIN = 10;
const uint8_t LORA_RST_PIN = 9;
const uint8_t LORA_DIO0_PIN = 2;

// Radio settings are intentionally UNCHANGED.
const uint8_t LORA_TX_POWER_DBM = 17;
const uint8_t LORA_SPREADING_FACTOR = 7;
const long LORA_BANDWIDTH_HZ = 500E3;
const uint8_t LORA_CODING_RATE_DENOMINATOR = 5;

// Compact LoRa command packet:
// byte 0 = left flaperon, signed -127...+127
// byte 1 = right flaperon, signed -127...+127
// byte 2 = left motor, 0...255
// byte 3 = right motor, 0...255
// byte 4 = special/status byte; CH5 position normally, 255 = RC failsafe
// byte 5 = XOR checksum of bytes 0...4
const uint8_t LORA_PACKET_BYTES = 6;

// If DIO0 or the TX done interrupt fails, this prevents LoRa from staying busy forever.
// Intentionally unchanged.
const uint32_t LORA_TX_TIMEOUT_US = 250000;

// ------------------------------------------------------------
// Motor driver pins
// ------------------------------------------------------------

const uint8_t MOTOR_PIN_A = 4;
const uint8_t MOTOR_PIN_B = 5;

const int16_t MOTOR_MAX_POWER = 255;
const int16_t MOTOR_DEADBAND = 4;

// ------------------------------------------------------------
// Servo pins
// ------------------------------------------------------------

// Pin 41 is deliberately omitted: the elevator is not Arduino-controlled.
const uint8_t SERVO_COUNT = 6;
const uint8_t SERVO_PINS[SERVO_COUNT] = {39, 43, 45, 47, 49, 51};

const uint16_t SERVO_MIN_US = 1000;
const uint16_t SERVO_CENTER_US = 1500;
const uint16_t SERVO_MAX_US = 2000;

// ------------------------------------------------------------
// RC receiver pins
// ------------------------------------------------------------

const uint8_t RC_CHANNEL_COUNT = 6;

// CH1 -> 48, CH2 -> 46, CH3 -> 44, CH4 -> 42,
// CH5 -> 40, CH6 -> 38.
const uint8_t RC_PINS[RC_CHANNEL_COUNT] = {48, 46, 44, 42, 40, 38};

const uint16_t RC_MIN_VALID_US = 800;
const uint16_t RC_MAX_VALID_US = 2200;
const uint32_t RC_TIMEOUT_US = 100000;

// ------------------------------------------------------------
// H-tail / aircraft RC mixer settings
// ------------------------------------------------------------

/*
  RC channel meaning:

  CH1 / rc.us[0]: roll stick       1000 left  -> 2000 right
  CH2 / rc.us[1]: pitch stick      1000 up    -> 2000 down / pull back
  CH3 / rc.us[2]: throttle stick   1000 up    -> 2000 down / throttle off
  CH4 / rc.us[3]: yaw stick        1000 left  -> 2000 right
  CH5 / rc.us[4]: 3-position flap switch, about 1000 / 1500 / 2000 us
  CH6 / rc.us[5]: 3-position switch used as a two-position control

  Current H-tail arrangement:
  - The elevator is NOT controlled by the Arduino.
  - Pin 39 is the tail unlocking servo and switches between
    two independently adjustable positions from CH6.
  - Pin 41 is not attached or driven by the Servo library.
  - Pin 45 keeps its existing CH6 two-position behavior.
  - CH4 yaw controls only the differential thrust.
*/

// RC input calibration. Adjust these if Serial Monitor readings are not exact.
const uint16_t RC_ROLL_MIN_US      = 1000;
const uint16_t RC_ROLL_CENTER_US   = 1500;
const uint16_t RC_ROLL_MAX_US      = 2000;

// Throttle is intentionally reversed: 2000 us = off, 1000 us = full.
const uint16_t RC_THROTTLE_OFF_US  = 2000;
const uint16_t RC_THROTTLE_FULL_US = 1000;

const uint16_t RC_YAW_MIN_US       = 1000;
const uint16_t RC_YAW_CENTER_US    = 1500;
const uint16_t RC_YAW_MAX_US       = 2000;

// CH5 switch detection thresholds.
// Below LOW_MAX is position 1, above HIGH_MIN is position 3, otherwise position 2.
const uint16_t RC_CH5_LOW_MAX_US   = 1250;
const uint16_t RC_CH5_HIGH_MIN_US  = 1750;

// CH6 is a 3-step switch used as a 2-position switch.
// Low and middle use position 1. High uses position 2.
const uint16_t CH6_POSITION_2_MIN_US = 1750;

// Stick direction tuning.
// Change a value from 1 to -1 if that control moves the wrong way.
const int8_t ROLL_INPUT_DIRECTION  = 1;
const int8_t YAW_INPUT_DIRECTION   = 1;

// Small deadband around stick center, expressed in signed-byte units.
const int16_t CONTROL_DEADBAND_BYTE = 1;

// ------------------------------------------------------------
// Tail unlocking servo on pin 39
// ------------------------------------------------------------

// SERVO_PINS[0] is physical pin 39.
const uint8_t TAIL_UNLOCK_SERVO_INDEX = 0;

// Change these two values to set the two tail-lock positions.
// CH6 low or middle -> position 1.
// CH6 high          -> position 2.
const int16_t TAIL_UNLOCK_POSITION_1_US = 1800;
const int16_t TAIL_UNLOCK_POSITION_2_US = 1200;

// Position used if the receiver signal is lost.
const int16_t TAIL_UNLOCK_FAILSAFE_US = TAIL_UNLOCK_POSITION_1_US;

// ------------------------------------------------------------
// Existing CH6 two-position servo on pin 45
// ------------------------------------------------------------

// SERVO_PINS[2] is physical pin 45.
const uint8_t CH6_SERVO_INDEX = 2;

// Existing values are kept unchanged.
const int16_t CH6_SERVO_POSITION_1_US = 1300;
const int16_t CH6_SERVO_POSITION_2_US = 1950;
const int16_t CH6_SERVO_FAILSAFE_US = CH6_SERVO_POSITION_1_US;

// ------------------------------------------------------------
// Flaperons sent over LoRa
// ------------------------------------------------------------

// loraSignedValue1 = left flaperon command
// loraSignedValue2 = right flaperon command
const int16_t FLAPERON_ROLL_GAIN_PERCENT = 50;

// Flap settings from CH5.
const int16_t FLAP_SETTING_LOW_BYTE  = 0;
const int16_t FLAP_SETTING_MID_BYTE  = 25;
const int16_t FLAP_SETTING_HIGH_BYTE = 50;

// Flaperon direction tuning before values are sent to the Nano.
const int8_t FLAPERON_LEFT_ROLL_SIGN  = 1;
const int8_t FLAPERON_RIGHT_ROLL_SIGN = -1;

// Reverses the final left/right flaperon command if needed.
const int8_t FLAPERON_LEFT_DIRECTION  = 1;
const int8_t FLAPERON_RIGHT_DIRECTION = -1;

// ------------------------------------------------------------
// Differential thrust
// ------------------------------------------------------------

// loraUnsignedValue1 = left motor throttle
// loraUnsignedValue2 = right motor throttle
const int16_t DIFFERENTIAL_THRUST_YAW_GAIN_PERCENT = 10;

// If yaw makes the wrong motor speed up, change this from 1 to -1.
const int8_t DIFFERENTIAL_THRUST_DIRECTION = 1;

// Motor trim and scaling.
const int16_t LEFT_MOTOR_OFFSET_BYTE = 0;
const int16_t RIGHT_MOTOR_OFFSET_BYTE = 0;
const int16_t LEFT_MOTOR_SCALE_PERCENT = 100;
const int16_t RIGHT_MOTOR_SCALE_PERCENT = 100;

// Main byte ranges used by the LoRa command packet.
const int16_t SIGNED_COMMAND_MIN = -127;
const int16_t SIGNED_COMMAND_MAX = 127;
const int16_t UNSIGNED_COMMAND_MIN = 0;
const int16_t UNSIGNED_COMMAND_MAX = 255;

// ------------------------------------------------------------
// Which RC channels are saved to OpenLog
// ------------------------------------------------------------

const uint8_t OPENLOG_RC_CHANNEL_COUNT = 6;
const uint8_t OPENLOG_RC_CHANNEL_INDEXES[OPENLOG_RC_CHANNEL_COUNT] = {0, 1, 2, 3, 4, 5};

// ------------------------------------------------------------
// Legacy LoRa RC selection constants
// ------------------------------------------------------------
//
// These are retained so no unrelated code/tuning is disturbed.
// The compact 6-byte packet no longer transmits RC channel bytes.

const uint8_t LORA_RC_CHANNEL_COUNT = 4;
const uint8_t LORA_RC_CHANNEL_INDEXES[LORA_RC_CHANNEL_COUNT] = {0, 1, 2, 3};

// ------------------------------------------------------------
// Logging and debug
// ------------------------------------------------------------

const bool ENABLE_OPENLOG = true;
const bool ENABLE_USB_DEBUG = false;

// 20000 = 50 Hz.
const uint32_t OPENLOG_INTERVAL_US = 20000;
const uint32_t USB_DEBUG_INTERVAL_US = 1000000;

#endif