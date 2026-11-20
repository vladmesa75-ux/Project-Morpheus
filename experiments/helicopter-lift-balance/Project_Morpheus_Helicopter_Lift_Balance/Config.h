#ifndef MORPHEUS_HELICOPTER_LIFT_BALANCE_CONFIG_H
#define MORPHEUS_HELICOPTER_LIFT_BALANCE_CONFIG_H

#include <Arduino.h>

// This file belongs only to the standalone helicopter lift-balance experiment.
const uint32_t USB_BAUD = 115200;

// The existing receiver: CH2 = pitch/collective; CH3 = throttle.
const uint8_t RC_PITCH_PIN = 46;
const uint8_t RC_THROTTLE_PIN = 44;
const uint16_t RC_MIN_VALID_US = 800;
const uint16_t RC_MAX_VALID_US = 2200;
const uint32_t RC_TIMEOUT_US = 100000UL;

const uint16_t RC_PITCH_MIN_US = 1000;
const uint16_t RC_PITCH_CENTER_US = 1500;
const uint16_t RC_PITCH_MAX_US = 2000;
const int8_t COLLECTIVE_INPUT_DIRECTION = 1;
const int16_t COLLECTIVE_DEADBAND_BYTE = 1;

// As in the existing helicopter V3 sketch: the left wing is physically
// flipped 180 degrees, so BOTH signed flaperon commands use direction -1.
// Changing these signs requires checking actual deflection on the hardware.
const int8_t LEFT_HELI_FLAPERON_DIRECTION = -1;
const int8_t RIGHT_HELI_FLAPERON_DIRECTION = -1;

// CH3 has reversed throttle (1000 us = full, 2000 us = stopped).
// Set RC_THROTTLE_OFF_US to the actual RC value observed at throttle-off.
// The arming threshold automatically follows this calibration.
const uint16_t RC_THROTTLE_FULL_US = 1000;
const uint16_t RC_THROTTLE_OFF_US = 2000;
const uint16_t THROTTLE_OFF_ARM_MARGIN_US = 15;
const uint16_t THROTTLE_OFF_ARM_THRESHOLD_US =
  RC_THROTTLE_OFF_US - THROTTLE_OFF_ARM_MARGIN_US;
const uint32_t SERVO_SETTLE_TIME_MS = 3000UL;

// Due servos: wing unlocked, tail LOCKED, H-tail folded.
const uint16_t SERVO_MIN_US = 500;
const uint16_t SERVO_MAX_US = 2500;
const uint8_t TAIL_LOCK_SERVO_PIN = 39;
const uint16_t TAIL_LOCKED_US = 1000;
const uint8_t HTAIL_FOLD_SERVO_PIN = 43;
const uint16_t HTAIL_FOLDED_US = 1000;
const uint8_t WING_UNLOCK_SERVO_PIN = 45;
const uint16_t WING_UNLOCK_US = 850;
const uint8_t OTHER_SERVO_PINS[] = {47, 49, 51};
const uint16_t OTHER_SERVO_CENTER_US = 1500;

// Original six-byte LoRa protocol; Nano remains completely unchanged.
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

// Difference in transmitted signed (-127..+127) commands, LEFT - RIGHT.
// Start at zero, then adjust from the DUE Serial Monitor while testing.
// The final value is only for helicopter-mode collective mixing.
const int16_t INITIAL_HELI_FLAPERON_TRIM_UNITS = 0;
const int16_t MAX_ABS_HELI_FLAPERON_TRIM_UNITS = 40;

#endif
