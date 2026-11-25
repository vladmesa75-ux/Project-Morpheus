#pragma once
#include <SPI.h>
#include <LoRa.h>

inline bool beginRadio() {
  LoRa.setPins(10, 9, 2);
  LoRa.setSPIFrequency(8000000);
  if (!LoRa.begin(433000000L)) return false;
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(500000L);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  return true;
}

inline bool receivePacket(uint8_t (&packet)[6]) {
  int length = LoRa.parsePacket();
  if (!length) return false;
  if (length != 6) {
    while (LoRa.available()) LoRa.read();
    return false;
  }
  uint8_t checksum = 0;
  for (uint8_t i = 0; i < 6; ++i) {
    int value = LoRa.read();
    if (value < 0) return false;
    packet[i] = value;
    checksum ^= packet[i];
  }
  return checksum == 0;
}
