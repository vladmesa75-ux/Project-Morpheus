// ============================================================
// LoRaSetup.ino
//
// This tab starts the SX1278 LoRa radio and applies the radio settings.
// These settings must match the Due transmitter.
// ============================================================

bool setupLoraRadio() {
  // Tell the LoRa library which Nano pins are connected to the SX1278.
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

  // 8 MHz SPI is usually fine on a Nano with short wires.
  // If you use long messy wires and reception is unreliable, try 4000000.
  LoRa.setSPIFrequency(8000000);

  // Start the radio at the selected frequency.
  // Returns false if the module does not respond.
  if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
    return false;
  }

  // Apply the same LoRa settings used by the Due transmitter.
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH_HZ);
  LoRa.setCodingRate4(LORA_CODING_RATE_DENOMINATOR);

  // Hardware CRC must match the Due.
  if (USE_LORA_CRC) {
    LoRa.enableCrc();
  } else {
    LoRa.disableCrc();
  }

  return true;
}