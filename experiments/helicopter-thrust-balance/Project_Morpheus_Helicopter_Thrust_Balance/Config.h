#ifndef HELICOPTER_THRUST_BALANCE_CONFIG_H
#define HELICOPTER_THRUST_BALANCE_CONFIG_H

#include <Arduino.h>

// Standalone Due experiment. These values do not change airplane mode.
const uint32_t USB_BAUD = 115200;

// RC CH3: current receiver wiring and reversed airplane-mode throttle.
const uint8_t RC_THROTTLE_PIN = 44;
const uint16_t RC_MIN_VALID_US = 800;
const uint16_t RC_MAX_VALID_US = 2200;
const uint16_t RC_THROTTLE_FULL_US = 1000;
const uint16_t RC_THROTTLE_OFF_US = 2000;
const uint32_t RC_TIMEOUT_US = 100000UL;
const uint16_t THROTTLE_OFF_ARM_THRESHOLD_US = 1950;
const uint32_t SERVO_SETTLE_TIME_MS = 3000UL;

// Due servos: all bounds are allowed to cover 500..2500 us.
const uint16_t SERVO_MIN_US = 500;
const uint16_t SERVO_MAX_US = 2500;
const uint8_t TAIL_UNLOCK_PIN = 39;
const uint8_t HTAIL_FOLD_PIN = 43;
const uint8_t WING_LOCK_PIN = 45;
const uint8_t OTHER_SERVO_PINS[] = {47, 49, 51};

// Keep the tail locked in helicopter operation; unlock only for landing
// in a separate landing-mode controller.
const uint16_t TAIL_LOCKED_US = 1000;
const uint16_t HTAIL_FOLDED_US = 1000;
// Existing helicopter experiments use 900 us for fully open, not the
// airplane-mode 1300 us partial-unlock position. Verify on the real mechanism.
const uint16_t WING_UNLOCK_US = 900;
const uint16_t OTHER_SERVO_CENTER_US = 1500;

// Current six-byte Nano receiver, no Nano firmware changes required.
const long LORA_FREQUENCY_HZ = 433000000L;
const uint8_t LORA_SS_PIN = 10;
const uint8_t LORA_RST_PIN = 9;
const uint8_t LORA_DIO0_PIN = 2;
const uint8_t LORA_TX_POWER_DBM = 17;
const uint8_t LORA_SPREADING_FACTOR = 7;
const long LORA_BANDWIDTH_HZ = 500000L;
const uint8_t LORA_CODING_RATE_DENOMINATOR = 5;
const uint8_t LORA_PACKET_BYTES = 6;
const uint32_t LORA_SEND_PERIOD_US = 20000UL;
const uint32_t LORA_TX_TIMEOUT_US = 250000UL;

// Helicopter-only motor trim: LEFT_D8 - RIGHT_D7, in 0..255 command units.
// Positive means more command to the left ESC (Nano D8).
// After finding the final value, set it here and apply that same value to
// helicopter-mode mixers ONLY; do not change airplane-mode motor scaling.
const int16_t INITIAL_HELICOPTER_MOTOR_TRIM_UNITS = 0;
const int16_t MAX_ABS_HELICOPTER_MOTOR_TRIM_UNITS = 100;

#endif
