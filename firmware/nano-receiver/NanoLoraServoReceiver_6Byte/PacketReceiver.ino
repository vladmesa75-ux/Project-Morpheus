// ============================================================
// PacketReceiver.ino
//
// This tab receives packets from LoRa and decodes the compact 6-byte format.
// ============================================================

uint8_t calculateChecksum(const uint8_t *packet) {
  // The Due puts an XOR checksum in the last byte of the packet.
  // To calculate it, we XOR bytes 0 to 4.
  // The result must equal byte 5.

  uint8_t checksum = 0;

  for (uint8_t i = 0; i < PACKET_SIZE - 1; i++) {
    checksum ^= packet[i];
  }

  return checksum;
}

int16_t readInt16LittleEndian(uint8_t lowByte, uint8_t highByte) {
  // The Due sends int16_t values as low byte first, high byte second.
  // This function puts the two bytes back together.

  return (int16_t)((uint16_t)lowByte | ((uint16_t)highByte << 8));
}

bool tryReceiveOnePacket() {
  // Ask the LoRa library whether a packet has arrived.
  // If no packet is waiting, packetSize will be 0.

  int packetSize = LoRa.parsePacket();

  if (packetSize <= 0) {
    return false;
  }

  // This sketch only accepts the compact 6-byte packet.
  // If a packet with the wrong length arrives, we throw it away.

  if (packetSize != PACKET_SIZE) {
    while (LoRa.available()) {
      LoRa.read();
    }

    return false;
  }

  uint8_t packet[PACKET_SIZE];

  // Read the full packet into the packet[] array.
  for (uint8_t i = 0; i < PACKET_SIZE; i++) {
    if (LoRa.available()) {
      packet[i] = (uint8_t)LoRa.read();
    } else {
      // This should rarely happen because packetSize said 6 bytes arrived.
      // If it does happen, reject the packet.
      return false;
    }
  }

  // Clear any extra bytes just in case.
  while (LoRa.available()) {
    LoRa.read();
  }

  // Check the manual XOR checksum.
  uint8_t receivedChecksum = packet[PACKET_SIZE - 1];
  uint8_t calculatedChecksum = calculateChecksum(packet);

  if (receivedChecksum != calculatedChecksum) {
    return false;
  }

  // If we got here, the packet is valid.
  // Decode it into the global variables used by the output tab.
  readPacketData(packet);

  return true;
}

void readPacketData(const uint8_t *packet) {
  // Bytes 0 and 1 are the two signed flapperon commands.
  signedValues[0] = (int8_t)packet[0];
  signedValues[1] = (int8_t)packet[1];

  // The old packet also carried two spare signed servo values.
  // The Due sent those as zero. Keep the exact same effective behavior
  // locally without spending two LoRa bytes on them.
  signedValues[2] = 0;
  signedValues[3] = 0;

  // Bytes 2 and 3 are the two ESC throttle commands.
  unsignedValues[0] = packet[2];
  unsignedValues[1] = packet[3];

  // Byte 4 is the existing special / arbitrary info byte.
  arbitraryByte = packet[4];

  // Attitude and receiver-channel fields were removed from the compact packet.
  // Their legacy globals are intentionally left untouched because they are not
  // used anywhere in this Nano's output path.
}