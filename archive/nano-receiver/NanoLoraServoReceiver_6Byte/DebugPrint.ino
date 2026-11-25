// ============================================================
// DebugPrint.ino
//
// This tab prints useful values to the Serial Monitor.
// ============================================================

void printOutputHeader() {
  // This header is printed once when the Nano starts.
  // Use 115200 baud in Serial Monitor.

  Serial.println("Hz,D4_flapperon_us,D6_flapperon_us,D3_us,D5_us,E8_us,E7_us");
}

void printOutputLine() {
  unsigned long now = millis();
  float seconds = (now - lastPrintMs) / 1000.0;

  // Avoid division by zero, just in case.
  if (seconds <= 0.0) {
    return;
  }

  // Count how many valid packets arrived since the last print.
  unsigned long packetsThisPeriod = validPacketCount - previousValidPacketCount;

  // Convert packets per time period into packets per second.
  // This is the receive frequency in Hz.
  float receiveHz = packetsThisPeriod / seconds;

  previousValidPacketCount = validPacketCount;
  lastPrintMs = now;

  // Compact comma-separated output.
  // This is easy to read and also easy to paste into a spreadsheet later.

  Serial.print(receiveHz, 1);
  Serial.print(",");
  Serial.print(servoOutputUs[1]);  // D4 flapperon, first signed value from Due
  Serial.print(",");
  Serial.print(servoOutputUs[3]);  // D6 flapperon, second signed value from Due
  Serial.print(",");
  Serial.print(servoOutputUs[0]);  // D3 servo, third signed value from Due
  Serial.print(",");
  Serial.print(servoOutputUs[2]);  // D5 servo, fourth signed value from Due
  Serial.print(",");
  Serial.print(escOutputUs[0]);    // D8 ESC
  Serial.print(",");
  Serial.println(escOutputUs[1]);  // D7 ESC
}