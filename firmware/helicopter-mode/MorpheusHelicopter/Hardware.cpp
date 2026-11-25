#include "Hardware.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>

namespace heli {
namespace {
Servo servos[6];
Adafruit_MPU6050 mpu;
volatile uint32_t rise[6], edge[6];
volatile uint16_t width[6];
volatile bool risingSeen[6], received[6];
bool imuReady = false, attitudeStarted = false;
float bias[3] = {}, accelScale = 0, gyroScale = 0;
uint32_t lastImu = 0;
bool radioInit = false, transmitting = false, completed = false;
uint32_t txStart = 0, lastComplete = 0, countStart = 0, count = 0;
float rate = 0;
volatile bool txDone = false;
void onTxDone() {
  txDone = true;
}
template <unsigned N> void rcChange() {
  uint32_t now = micros();
  if (digitalRead(cfg::rcPins[N])) {
    rise[N] = now;
    risingSeen[N] = true;
    return;
  }
  uint32_t pulse = now - rise[N];
  if (risingSeen[N] && pulse >= unsigned(cfg::rcMinValidUs) && pulse <= unsigned(cfg::rcMaxValidUs)) {
    width[N] = pulse;
    edge[N] = now;
    received[N] = true;
  }
  risingSeen[N] = false;
}
bool registers(TwoWire &bus, uint8_t address, uint8_t reg, uint8_t *bytes, uint8_t n) {
  bus.beginTransmission(address);
  bus.write(reg);
  if (bus.endTransmission(false) != 0 || bus.requestFrom(address, n) != n)
    return false;
  for (uint8_t i = 0; i < n; i++)
    bytes[i] = bus.read();
  return true;
}
int16_t word(const uint8_t *p) {
  return int16_t(uint16_t(p[0]) << 8 | p[1]);
}
bool initImu() {
  if (!mpu.begin(cfg::imuAddress, &Wire))
    return false;
  mpu.setAccelerometerRange(MORPHEUS_ACCEL_RANGE);
  mpu.setGyroRange(MORPHEUS_GYRO_RANGE);
  mpu.setFilterBandwidth(MORPHEUS_IMU_FILTER);
  Wire.setClock(cfg::imuBusHz);
  accelScale = (2U << unsigned(MORPHEUS_ACCEL_RANGE)) / 32768.0f;
  gyroScale = (250U << unsigned(MORPHEUS_GYRO_RANGE)) / 32768.0f;
  if (cfg::calibrateGyro) {
    uint8_t b[6];
    for (unsigned i = 0; i < cfg::gyroSamples; i++) {
      if (!registers(Wire, cfg::imuAddress, 0x43, b, 6))
        return false;
      for (unsigned axis = 0; axis < 3; axis++)
        bias[axis] += word(b + 2 * axis) * gyroScale;
      delay(2);
    }
    for (float &v : bias)
      v /= cfg::gyroSamples;
  }
  bias[2] += cfg::yawGyroTrimDps;
  return true;
}
} // namespace
void beginHardware() {
  analogWriteResolution(8);
  pinMode(cfg::axlePinA, OUTPUT);
  pinMode(cfg::axlePinB, OUTPUT);
  driveAxle(0);
  for (unsigned i = 0; i < 6; i++) {
    // Preload the first pulse before the timer can mark this channel active.
    servos[i].writeMicroseconds(cfg::mechanismUs[i]);
    servos[i].attach(cfg::servoPins[i], cfg::servoMinUs, cfg::servoMaxUs);
    servos[i].writeMicroseconds(cfg::mechanismUs[i]);
    pinMode(cfg::rcPins[i], INPUT);
  }
  void (*const handlers[])() = {rcChange<0>, rcChange<1>, rcChange<2>,
                                rcChange<3>, rcChange<4>, rcChange<5>};
  for (unsigned i = 0; i < 6; i++)
    attachInterrupt(digitalPinToInterrupt(cfg::rcPins[i]), handlers[i], CHANGE);
  Wire.begin();
  Wire.setClock(cfg::imuBusHz);
  Wire1.begin();
  Wire1.setClock(cfg::encoderBusHz);
  imuReady = initImu();
  LoRa.setPins(cfg::radioCs, cfg::radioReset, cfg::radioDio);
  radioInit = LoRa.begin(cfg::radioHz);
  if (radioInit) {
    LoRa.setTxPower(cfg::radioDbm);
    LoRa.setSpreadingFactor(cfg::radioSf);
    LoRa.setSignalBandwidth(cfg::radioBandwidthHz);
    LoRa.setCodingRate4(cfg::radioCr);
    LoRa.enableCrc();
    LoRa.onTxDone(onTxDone);
  }
}
Inputs receiver() {
  Inputs rc;
  rc.fresh = true;
  noInterrupts();
  uint32_t now = micros();
  for (unsigned i = 0; i < 6; i++) {
    rc.us[i] = width[i];
    uint32_t age = now - edge[i];
    // Latch expired channels. micros() wrap must not resurrect an old pulse.
    if (age > cfg::rcTimeoutUs)
      received[i] = false;
    if (!received[i])
      rc.fresh = false;
    if (age > rc.ageUs)
      rc.ageUs = age;
  }
  interrupts();
  return rc;
}
bool readEncoder(uint16_t &raw) {
  uint8_t b[3];
  if (!registers(Wire1, cfg::encoderAddress, 0x0B, b, 3) || !(b[0] & 0x20))
    return false;
  raw = (uint16_t(b[1] & 15) << 8) | b[2];
  return true;
}
bool sampleImu(Attitude &a, uint32_t now) {
  uint8_t b[14];
  if (!imuReady || !registers(Wire, cfg::imuAddress, 0x3B, b, 14))
    return false;
  float ax = word(b) * accelScale, ay = word(b + 2) * accelScale, az = word(b + 4) * accelScale;
  a.gx = word(b + 8) * gyroScale - bias[0];
  a.gy = word(b + 10) * gyroScale - bias[1];
  a.gz = cfg::yawGyroSign * (word(b + 12) * gyroScale - bias[2]);
  float roll = atan2f(ay, az) * 57.2957795f;
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;
  uint32_t interval = now - lastImu;
  if (!attitudeStarted || interval >= cfg::imuStaleUs) {
    a.roll = roll;
    a.pitch = pitch;
  } else {
    float dt = seconds(interval), blend = dt / (cfg::attitudeTimeConstantS + dt);
    a.roll = wrap180(a.roll + a.gx * dt);
    a.pitch = wrap180(a.pitch + a.gy * dt);
    a.roll = wrap180(a.roll + blend * wrap180(roll - a.roll));
    a.pitch = wrap180(a.pitch + blend * wrap180(pitch - a.pitch));
    if (fabsf(a.gz) > cfg::yawDeadbandDps)
      a.yaw = wrap180(a.yaw + a.gz * dt);
  }
  attitudeStarted = true;
  lastImu = now;
  return true;
}
void driveAxle(int value) {
  // De-energize the opposite bridge input first, including on reversals.
  if (value > 0) {
    analogWrite(cfg::axlePinB, 0);
    analogWrite(cfg::axlePinA, value);
  } else {
    analogWrite(cfg::axlePinA, 0);
    analogWrite(cfg::axlePinB, -value);
  }
}
void pollRadio(uint32_t now) {
  if (!radioInit)
    return;
  if (transmitting) {
    if (elapsed(now, txStart, cfg::txTimeoutUs)) {
      LoRa.idle();
      transmitting = completed = false;
    } else if (txDone) {
      transmitting = false;
      completed = true;
      lastComplete = now;
      ++count;
    }
  }
  if (elapsed(now, countStart, 1000000)) {
    rate = count / seconds(now - countStart);
    count = 0;
    countStart = now;
  }
}
bool radioHealthy(uint32_t now) {
  return completed && now - lastComplete < cfg::txTimeoutUs;
}
bool radioReady(uint32_t now) {
  return radioInit && !transmitting && elapsed(now, txStart, cfg::txSpacingUs);
}
void transmit(const Outputs &o, bool fault, uint32_t now) {
  uint8_t packet[6];
  pack(o, fault, packet);
  txDone = false;
  if (!LoRa.beginPacket() || LoRa.write(packet, 6) != 6 || !LoRa.endPacket(true)) {
    completed = false;
    return;
  }
  txStart = now;
  transmitting = true;
}
float txRate() {
  return rate;
}
} // namespace heli
