/*
  LoggingDebug.ino

  This tab handles:
  - loop timing
  - OpenLog CSV logging
  - optional USB debug printing

  OpenLog data is written as readable text.
*/

void updateLoopTiming() {
  uint32_t now = micros();

  loopTimeUs = now - previousLoopStartUs;
  previousLoopStartUs = now;

  if (loopTimeUs > 0) {
    loopHz = 1000000.0f / loopTimeUs;
  }
}

void writeOpenLogHeader() {
  /*
    Writes one CSV header line to the OpenLog file.

    Logged data:
    - timestamp in microseconds
    - 6 selected receiver channels
    - roll, pitch, yaw
  */

  Serial2.print("time_us");

  for (uint8_t i = 0; i < OPENLOG_RC_CHANNEL_COUNT; i++) {
    Serial2.print(",rc");
    Serial2.print(OPENLOG_RC_CHANNEL_INDEXES[i] + 1);
  }

  Serial2.println(",roll_deg,pitch_deg,yaw_deg");
}

void logOpenLogIfReady() {
  /*
    Logs to OpenLog without using delay().

    The interval is controlled by OPENLOG_INTERVAL_US in Config.h.
    If you set OPENLOG_INTERVAL_US to 0, this tries to log every loop.

    availableForWrite() helps avoid blocking if the serial buffer is full.
    If the buffer is too full, this skips a log line instead of stopping flight code.
  */

  uint32_t now = micros();

  if (OPENLOG_INTERVAL_US > 0 && now - lastOpenLogUs < OPENLOG_INTERVAL_US) {
    return;
  }

  lastOpenLogUs = now;

  // A CSV line is not huge, but we still avoid writing if the buffer is nearly full.
  if (Serial2.availableForWrite() < 80) {
    return;
  }

  Serial2.print(now);

  for (uint8_t i = 0; i < OPENLOG_RC_CHANNEL_COUNT; i++) {
    uint8_t channelIndex = OPENLOG_RC_CHANNEL_INDEXES[i];

    Serial2.print(',');

    if (channelIndex < RC_CHANNEL_COUNT) {
      Serial2.print(rc.us[channelIndex]);
    } else {
      Serial2.print(0);
    }
  }

  Serial2.print(',');
  Serial2.print(attitude.rollDeg, 2);

  Serial2.print(',');
  Serial2.print(attitude.pitchDeg, 2);

  Serial2.print(',');
  Serial2.println(attitude.yawDeg, 2);
}

void printUsbDebugIfReady() {
  /*
    Optional slow debug message for Serial Monitor.

    This is intentionally low rate because Serial printing can slow the loop.
  */

  uint32_t nowMs = millis();

  if (nowMs - lastPacketRateCountMs >= 1000) {
    lastPacketRateCountMs = nowMs;
    lastPacketRateValue = packetsSentThisSecond;
    packetsSentThisSecond = 0;
  }

  uint32_t nowUs = micros();

  if (nowUs - lastUsbDebugUs < USB_DEBUG_INTERVAL_US) {
    return;
  }

  lastUsbDebugUs = nowUs;

  Serial.print("loop_hz:");
  Serial.print(loopHz, 1);
  Serial.print('\t');

  Serial.print("lora_hz:");
  Serial.print(lastPacketRateValue);
  Serial.print('\t');

  Serial.print("rc_fresh:");
  Serial.print(rc.fresh ? 1 : 0);
  Serial.print('\t');

  Serial.print("pitch:");
  Serial.print(attitude.pitchDeg, 1);
  Serial.print('\t');

  Serial.print("roll:");
  Serial.print(attitude.rollDeg, 1);
  Serial.print('\t');

  Serial.print("yaw:");
  Serial.print(attitude.yawDeg, 1);
  Serial.print('\t');

  Serial.print("encoder:");
  Serial.print(encoder.raw);
  Serial.print('\t');

  Serial.print("lora_busy:");
  Serial.println(loraTxBusy ? 1 : 0);
}