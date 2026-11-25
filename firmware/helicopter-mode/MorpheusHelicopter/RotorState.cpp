#include "Control.h"
namespace heli {
void Rotor::index(uint16_t raw, uint32_t now) {
  *this = Rotor();
  indexed = true;
  previousRaw = raw;
  lastUs = windowStartUs = now;
  phase = wrap360(cfg::phaseZeroDeg);
}
void Rotor::sample(uint16_t raw, uint32_t now) {
  if (!healthy(now)) {
    lost = true;
    return;
  }
  int counts = int(raw) - int(previousRaw);
  if (counts > 2048)
    counts -= 4096;
  if (counts < -2048)
    counts += 4096;
  float encoderDeg = counts * (360.0f / 4096.0f);
  if (fabsf(encoderDeg) > cfg::maxEncoderDps * seconds(now - lastUs) + cfg::encoderSlackDeg) {
    // The rejected sample is not a new reference. An ambiguous gap latches lost.
    return;
  }
  float delta = cfg::encoderSign * encoderDeg / cfg::encoderTurnsPerRotorTurn;
  phase = wrap360(phase + delta);
  windowDeg += delta;
  previousRaw = raw;
  lastUs = now;
  uint32_t duration = now - windowStartUs;
  if (duration >= cfg::rpmWindowUs) {
    float measured = windowDeg / (6.0f * seconds(duration));
    if (fabsf(measured) < cfg::rpmZeroBand)
      measured = 0;
    windowRpm = measured;
    rpm += (speedReady ? cfg::rpmAlpha : 1.0f) * (measured - rpm);
    if (measured == 0 && fabsf(rpm) < cfg::rpmZeroBand)
      rpm = 0;
    speedReady = true;
    windowStartUs = now;
    windowDeg = 0;
  }
}
} // namespace heli
