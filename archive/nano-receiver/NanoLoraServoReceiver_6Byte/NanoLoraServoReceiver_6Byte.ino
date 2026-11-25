#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>

#include "Config.h"
#include "FunctionPrototypes.h"

// ============================================================
// NanoLoraServoReceiver_6Byte.ino
//
// Main tab.
//
// What this sketch does:
// 1. Receives 6-byte LoRa packets from the Arduino Due.
// 2. Checks that the packet size is correct.
// 3. Checks the manual XOR checksum in byte 5.
// 4. Uses two signed values for the two flapperons.
// 5. Uses two unsigned values for the two ESC throttle signals.
// 6. Keeps the two former spare servo values at zero / center.
// 7. Goes to failsafe if packets stop arriving.
//
// Physical outputs in your setup:
// D3 = servo controlled by signed packet value
// D4 = flapperon controlled by signed packet value
// D5 = servo controlled by signed packet value
// D6 = flapperon controlled by signed packet value
// D8 = ESC controlled by unsigned packet value 1
// D7 = ESC controlled by unsigned packet value 2
// ============================================================

// -------------------- Servo and ESC objects --------------------
//
// The Servo library is also used for ESCs because RC ESCs understand
// the same kind of 1000 to 2000 microsecond signal as servos.

Servo servos[SERVO_COUNT];
Servo escs[ESC_COUNT];

// -------------------- Received values --------------------
//
// signedValues[0] and [1] store packet bytes 0 and 1.
// These are the two flapperon commands, from -127 to +127.
//
// signedValues[2] and [3] are kept at zero by PacketReceiver.ino.
// This preserves the original centered D3/D5 behavior without transmitting
// two unnecessary spare bytes.
//
// unsignedValues[] stores packet bytes 2 and 3.
// These are used for the two ESC throttle outputs.

int8_t signedValues[4] = {0, 0, 0, 0};
uint8_t unsignedValues[2] = {0, 0};

// Legacy variables retained so the rest of the original sketch structure
// stays unchanged. The compact packet no longer transmits attitude data.

int16_t pitchTenthsDeg = 0;
int16_t rollTenthsDeg = 0;
int16_t yawTenthsDeg = 0;

// Special/status byte from the Due. Not used directly by the output code.

uint8_t arbitraryByte = 0;

// -------------------- Current output values --------------------
//
// These variables remember what is currently being sent to the servos and ESCs.
// They are also printed to Serial Monitor once per second.

int servoOutputUs[SERVO_COUNT] = {1500, 1500, 1500, 1500};
int escOutputUs[ESC_COUNT] = {1000, 1000};

// -------------------- Failsafe state --------------------
//
// lastGoodPacketMs remembers the time of the last valid packet.
// failsafeActive remembers whether the board is currently in failsafe.

unsigned long lastGoodPacketMs = 0;
bool failsafeActive = true;

// -------------------- Receive frequency display --------------------
//
// validPacketCount counts only packets that passed the checksum test.
// The Serial Monitor frequency is calculated from this value.

unsigned long lastPrintMs = 0;
unsigned long validPacketCount = 0;
unsigned long previousValidPacketCount = 0;

void setup() {
  // Start Serial Monitor.
  // The code does not depend on Serial being connected.
  // It will run from battery power without the USB cable.
  Serial.begin(115200);
  delay(500);

  // Attach servos and ESCs, then immediately send safe outputs.
  setupOutputs();
  applyFailsafe();

  // Start LoRa radio.
  // If the LoRa module is not detected, stay in failsafe forever.
  if (!setupLoraRadio()) {
    Serial.println("LoRa failed");

    while (1) {
      applyFailsafe();
      delay(1000);
    }
  }

  lastGoodPacketMs = millis();
  lastPrintMs = millis();

  printOutputHeader();
}

void loop() {
  // Try to receive and process one LoRa packet.
  // This function returns true only when a full valid packet was received.
  if (tryReceiveOnePacket()) {
    applyOutputs();
    lastGoodPacketMs = millis();
    validPacketCount++;
  }

  // Failsafe check.
  // If packets stop, servos center and ESCs go to minimum throttle.
  if (millis() - lastGoodPacketMs > FAILSAFE_TIMEOUT_MS) {
    if (!failsafeActive) {
      applyFailsafe();
    }
  }

  // Print one compact line every second.
  if (millis() - lastPrintMs >= SERIAL_PRINT_INTERVAL_MS) {
    printOutputLine();
  }
}