#include "Calibration.h"
#include "NanoOutputs.h"
#include "Radio.h"

bool ready, tracking;
uint32_t lastPacketMs;

uint16_t throttleUs(uint8_t value) {
  // Matches the baseline receiver: 254 and 255 both give full scale.
  return 1000 + (uint32_t)min(value, (uint8_t)254) * 1000 / 254;
}

void setup() {
  Serial.begin(115200);
  outputs::begin();
  bool calibrated = validCalibration(leftPulseUs) && validCalibration(rightPulseUs);
  if (calibrated) outputs::setFlaps(leftPulseUs[4], rightPulseUs[4]);
  outputs::service();
  bool radioOk = beginRadio();
  if (!calibrated) Serial.println(F("Fill both measured rows in Calibration.h"));
  if (!radioOk) Serial.println(F("LoRa initialization failed"));
  ready = calibrated && radioOk;
}

void loop() {
  uint8_t packet[6];
  if (ready && receivePacket(packet)) {
    outputs::setFlaps(interpolate((int8_t)packet[0], leftPulseUs),
                      interpolate((int8_t)packet[1], rightPulseUs));
    outputs::service();
    outputs::setMotors(throttleUs(packet[2]), throttleUs(packet[3]));
    lastPacketMs = millis();
    tracking = true;
  }
  if (tracking && millis() - lastPacketMs >= 500UL) {
    outputs::setMotors(1000, 1000);
    outputs::setFlaps(leftPulseUs[4], rightPulseUs[4]);
    tracking = false;
  }
  outputs::service();
}
