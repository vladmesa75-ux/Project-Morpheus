/*
  EncoderAs5600.ino

  This tab reads the AS5600 magnetic encoder on Wire1 / I2C1.

  The AS5600 gives a 12-bit angle:
  - 0 means 0 degrees
  - 4095 is almost 360 degrees
*/

bool setupAs5600() {
  return readAs5600();
}

bool readAs5600() {
  Wire1.beginTransmission(AS5600_ADDRESS);
  Wire1.write(AS5600_RAW_ANGLE_HIGH_REGISTER);

  if (Wire1.endTransmission(false) != 0) {
    return false;
  }

  uint8_t bytesRead = Wire1.requestFrom(AS5600_ADDRESS, (uint8_t)2);

  if (bytesRead != 2) {
    return false;
  }

  uint8_t highByte = Wire1.read();
  uint8_t lowByte = Wire1.read();

  encoder.raw = ((uint16_t)(highByte & 0x0F) << 8) | lowByte;
  encoder.deg = encoder.raw * (360.0f / 4096.0f);

  return true;
}