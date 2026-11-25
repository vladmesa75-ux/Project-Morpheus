#include "NanoOutputs.h"
#include "Radio.h"
#include <stdlib.h>
#include <string.h>

bool radioOk;
uint32_t periodStart, packets;
char line[24];
uint8_t used;
bool overflowed;

void applyLine() {
  char *side = strtok(line, " \t");
  char *value = strtok(NULL, " \t");
  if (!side || !value || strtok(NULL, " \t")) {
    Serial.println(F("Use L 1430 or R 1470"));
    return;
  }
  char *end;
  long us = strtol(value, &end, 10);
  bool left = !strcmp(side, "L") || !strcmp(side, "l");
  bool right = !strcmp(side, "R") || !strcmp(side, "r");
  if ((!left && !right) || *end || us < 500 || us > 2500) {
    Serial.println(F("Use L or R and 500..2500 us"));
    return;
  }
  outputs::setFlaps(left ? us : outputs::leftUs, right ? us : outputs::rightUs);
  outputs::service();
}

void readConsole() {
  // Bound console work per pass so reception continues during serial input.
  for (uint8_t n = 0; n < 8 && Serial.available(); ++n) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (overflowed) Serial.println(F("Command too long"));
      else if (used) {
        line[used] = 0;
        applyLine();
      }
      used = 0;
      overflowed = false;
    } else if (used < sizeof(line)-1) {
      line[used++] = c;
    } else {
      overflowed = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  outputs::begin();
  outputs::service();
  radioOk = beginRadio();
  if (!radioOk) Serial.println(F("LoRa unavailable; manual calibration active"));
  Serial.println(F("Use L 1430 or R 1470, then Enter"));
  periodStart = millis();
}

void loop() {
  uint8_t packet[6];
  if (radioOk && receivePacket(packet)) {
    ++packets;
    outputs::setFlaps(outputs::leftUs, outputs::rightUs);
  }
  outputs::service();
  readConsole();
  uint32_t now = millis();
  uint32_t elapsed = now - periodStart;
  if (elapsed >= 1000) {
    Serial.print(F("RX "));
    Serial.print(1000.0f * packets / elapsed, 1);
    Serial.print(F(" Hz | D4 "));
    Serial.print(outputs::leftUs);
    Serial.print(F(" us | D6 "));
    Serial.print(outputs::rightUs);
    Serial.println(F(" us"));
    packets = 0;
    periodStart = now;
  }
}
