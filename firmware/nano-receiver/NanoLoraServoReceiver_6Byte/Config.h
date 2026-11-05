#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// Config.h
//
// This tab is the main place where you should change settings.
// Most people should only need to edit this file.
// ============================================================

// -------------------- LoRa wiring --------------------
//
// Arduino Nano + SX1278 usual wiring:
//
// SX1278 NSS / CS  -> Nano D10
// SX1278 RST       -> Nano D9
// SX1278 DIO0      -> Nano D2
// SX1278 MOSI      -> Nano D11
// SX1278 MISO      -> Nano D12
// SX1278 SCK       -> Nano D13
// SX1278 3.3 V     -> 3.3 V supply, not 5 V
// SX1278 GND       -> GND
//
// Important:
// The SX1278 is a 3.3 V module. Many breakout boards are not 5 V safe.
// If your module does not have level shifting, do not feed Nano 5 V logic
// directly into the LoRa module.

const uint8_t LORA_CS_PIN = 10;
const uint8_t LORA_RST_PIN = 9;
const uint8_t LORA_DIO0_PIN = 2;

// -------------------- LoRa radio settings --------------------
//
// These must match the Arduino Due transmitter exactly.
// If one of these is different between the Due and the Nano, reception can fail.

const long LORA_FREQUENCY_HZ = 433000000L;
const uint8_t LORA_SPREADING_FACTOR = 7;
const long LORA_BANDWIDTH_HZ = 500000L;
const uint8_t LORA_CODING_RATE_DENOMINATOR = 5;

// This must match the Due code.
// true  = the radio chip also checks the LoRa hardware CRC.
// false = no LoRa hardware CRC.
//
// This is separate from the manual XOR checksum in byte 5.
// The manual XOR checksum is still checked by this sketch.

const bool USE_LORA_CRC = true;

// -------------------- Packet settings --------------------
//
// This sketch expects the compact 6-byte packet from the Due.
//
// Packet layout:
// Byte 0: signed value 1, left flapperon, -127 to +127
// Byte 1: signed value 2, right flapperon, -127 to +127
// Byte 2: unsigned value 1, ESC 1
// Byte 3: unsigned value 2, ESC 2
// Byte 4: special / arbitrary info byte
// Byte 5: XOR checksum of bytes 0 to 4
//
// The former signed values 3 and 4 are no longer transmitted.
// PacketReceiver.ino sets them locally to zero so D3 and D5 remain centered,
// exactly as they did when the Due transmitted zero in those spare fields.

const uint8_t PACKET_SIZE = 6;

// -------------------- Output pins --------------------
//
// LoRa already uses D2, D9, D10, D11, D12, and D13.
// The servo and ESC outputs therefore use D3, D4, D5, D6, D7, and D8.
//
// Your current physical setup:
// D4 = flapperon servo
// D6 = flapperon servo
// D3 = normal servo controlled by one of the other signed values
// D5 = normal servo controlled by one of the other signed values
// D8 = ESC
// D7 = ESC

const uint8_t SERVO_COUNT = 4;
const uint8_t ESC_COUNT = 2;

const uint8_t SERVO_PINS[SERVO_COUNT] = {
  3,  // Servo output 1, normal servo
  4,  // Servo output 2, flapperon
  5,  // Servo output 3, normal servo
  6   // Servo output 4, flapperon
};

const uint8_t ESC_PINS[ESC_COUNT] = {
  8,  // ESC output 1
  7   // ESC output 2
};

// -------------------- Which packet value controls each servo --------------------
//
// The compact packet transmits only two signed values: bytes 0 and 1.
// They are the two flapperons.
//
// To preserve the original output code and pin mapping:
// - signed value 1, packet byte 0, goes to D4
// - signed value 2, packet byte 1, goes to D6
// - signed value 3 is generated locally as zero and goes to D3
// - signed value 4 is generated locally as zero and goes to D5
//
// The array order follows the physical pins in SERVO_PINS[]:
// servo 0 = D3
// servo 1 = D4, flapperon
// servo 2 = D5
// servo 3 = D6, flapperon
//
// So this means:
// D3 uses local signed value 3 = 0.
// D4 uses signed value 1, packet byte 0.
// D5 uses local signed value 4 = 0.
// D6 uses signed value 2, packet byte 1.

const uint8_t SERVO_SIGNED_VALUE_INDEX[SERVO_COUNT] = {
  2,  // D3 uses local signed value 3, held at zero
  0,  // D4 uses signed packet byte 0, the first flapperon value
  3,  // D5 uses local signed value 4, held at zero
  1   // D6 uses signed packet byte 1, the second flapperon value
};

// -------------------- Servo endpoints --------------------
//
// Each servo maps a signed input from -127...+127 to a PWM output.
//
// 1000 us is usually one end.
// 1500 us is usually center.
// 2000 us is usually the other end.
//
// To reverse one servo, swap its min and max values.
// Example:
// For D4 reverse, change its min from 1000 to 2000 and max from 2000 to 1000.

int servoMinUs[SERVO_COUNT] = {
  1000,  // D3
  1950,  // D4, flapperon
  1000,  // D5
  1800   // D6, flapperon
};

int servoCenterUs[SERVO_COUNT] = {
  1500,  // D3
  1430,  // D4, flapperon
  1500,  // D5
  1470   // D6, flapperon
};

int servoMaxUs[SERVO_COUNT] = {
  2000,  // D3
  1100,  // D4, flapperon
  2000,  // D5
  900   // D6, flapperon
};

// -------------------- ESC endpoints --------------------
//
// Standard RC ESC signal range:
// 1000 us = minimum throttle
// 2000 us = full throttle
//
// For first tests, remove propellers.
// You can also temporarily lower escMaxUs to 1300 or 1500.

int escMinUs[ESC_COUNT] = {
  1000,  // D8 ESC
  1000   // D7 ESC
};

int escMaxUs[ESC_COUNT] = {
  2000,  // D8 ESC
  2000   // D7 ESC
};

// The current Due transmitter sends unsigned values from 0 to 254.
// If the Due is later changed to send true 0 to 255, change this to 255.

const int UNSIGNED_INPUT_MAX = 254;

// -------------------- Failsafe --------------------
//
// If no good packet is received for this many milliseconds:
// - all servos go to their center positions
// - both ESCs go to minimum throttle

const unsigned long FAILSAFE_TIMEOUT_MS = 500;

// -------------------- Serial monitor --------------------
//
// The serial monitor prints one line per second.
// Format:
// Hz,D4_flapperon_us,D6_flapperon_us,D3_us,D5_us,E8_us,E7_us

const unsigned long SERIAL_PRINT_INTERVAL_MS = 1000;

#endif