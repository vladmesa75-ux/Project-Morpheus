/*
  LoraTelemetry.ino

  Project Morpheus - Final V1 airplane mode
  Compact 6-byte LoRa command packet.

  IMPORTANT:
  Only packet packing has been reduced.
  Frequency, spreading factor, bandwidth, coding rate, CRC,
  asynchronous TX and timeout behaviour are unchanged.

  Packet:
    byte 0 = left flaperon command, int8_t raw byte
    byte 1 = right flaperon command, int8_t raw byte
    byte 2 = left motor command, uint8_t
    byte 3 = right motor command, uint8_t
    byte 4 = special/status byte
    byte 5 = XOR checksum of bytes 0...4
*/

bool setupLoRaRadio() {
  LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

  if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
    return false;
  }

  LoRa.setTxPower(LORA_TX_POWER_DBM);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH_HZ);
  LoRa.setCodingRate4(LORA_CODING_RATE_DENOMINATOR);
  LoRa.enableCrc();

  LoRa.onTxDone(onLoRaTxDone);

  return true;
}

void onLoRaTxDone() {
  loraTxBusy = false;
}

void serviceLoRaTxTimeout() {
  if (!loraTxBusy) {
    return;
  }

  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastLoraTxStartUs) > LORA_TX_TIMEOUT_US) {
    loraTxBusy = false;
    LoRa.idle();
  }
}

bool sendLoRaPacket(const uint8_t packet[LORA_PACKET_BYTES]) {
  if (!loraOk || loraTxBusy) {
    return false;
  }

  LoRa.beginPacket();
  LoRa.write(packet, LORA_PACKET_BYTES);

  loraTxBusy = true;
  lastLoraTxStartUs = micros();

  int result = LoRa.endPacket(true);

  if (result == 0) {
    loraTxBusy = false;
    return false;
  }

  return true;
}

void buildLoRaPacket(uint8_t packet[LORA_PACKET_BYTES]) {
  packet[0] = (uint8_t)loraSignedValue1;
  packet[1] = (uint8_t)loraSignedValue2;
  packet[2] = loraUnsignedValue1;
  packet[3] = loraUnsignedValue2;
  packet[4] = loraArbitraryByte;
  packet[5] = xorChecksum(packet, 5);
}

uint8_t xorChecksum(const uint8_t *buffer, uint8_t length) {
  uint8_t checksum = 0;

  for (uint8_t i = 0; i < length; i++) {
    checksum ^= buffer[i];
  }

  return checksum;
}

// -----------------------------------------------------------------
// Legacy helper functions retained so this tab remains drop-in
// compatible with the existing final V1 declarations.
// They are no longer used by the compact packet builder.
// -----------------------------------------------------------------

void writeInt16LittleEndian(uint8_t *buffer, uint8_t startIndex, int16_t value) {
  uint16_t raw = (uint16_t)value;
  buffer[startIndex] = (uint8_t)(raw & 0xFF);
  buffer[startIndex + 1] = (uint8_t)((raw >> 8) & 0xFF);
}

uint8_t rcUsToByte(uint16_t pulseUs) {
  if (pulseUs <= 1000) {
    return 0;
  }

  if (pulseUs >= 2000) {
    return 254;
  }

  return (uint8_t)(((uint32_t)(pulseUs - 1000) * 254UL) / 1000UL);
}

int16_t angleToTenthsDegInt16(float angleDeg) {
  float scaled = angleDeg * 10.0f;

  if (scaled > 32767.0f) {
    scaled = 32767.0f;
  } else if (scaled < -32768.0f) {
    scaled = -32768.0f;
  }

  return (int16_t)scaled;
}

int8_t constrainSignedByte(int16_t value) {
  if (value < -127) {
    value = -127;
  } else if (value > 127) {
    value = 127;
  }

  return (int8_t)value;
}

uint8_t constrainUnsignedByte(int16_t value) {
  if (value < 0) {
    value = 0;
  } else if (value > 255) {
    value = 255;
  }

  return (uint8_t)value;
}