#include "Control.h"
namespace heli {
void Yaw::start(float heading, float gyro) {
  *this = Yaw();
  target = wrap180(heading);
  previousGyro = gyro;
}
void Yaw::step(YawMode mode, float stick, float heading, float gyro, float rpm, float dt,
               const Tuning &t) {
  if (dt <= 0)
    return;
  if (mode == YawMode::Off) {
    start(heading, gyro);
    return;
  }
  stickRate = cfg::yawMaxRateDps * ((1 - cfg::yawExpo) * stick + cfg::yawExpo * stick * stick * stick);
  referenceRate = stickRate;
  float error = stickRate - gyro;
  float slope = (gyro - previousGyro) / dt;
  leadLimited = false;
  const bool position = mode == YawMode::Heading;
  if (position) {
    float lead = wrap180(target - heading);
    float desiredStep = stickRate * dt;
    // Restrict only motion in the requested direction. A disturbance must
    // never drag a centered-stick target or make reverse stick jump the target.
    float step =
        limit(desiredStep, fminf(0, -cfg::yawMaxLeadDeg - lead), fmaxf(0, cfg::yawMaxLeadDeg - lead));
    target = wrap180(target + step);
    referenceRate = step / dt;
    leadLimited = fabsf(step - desiredStep) > 1e-6f;
    error = wrap180(target - heading);
    slope = gyro;
  } else {
    target = heading;
  }
  float ff = t.yawBias + t.yawRpmF * rpm + t.yawRateF * referenceRate;
  float slew = cfg::axleSlewPerSecond * dt;
  power = pid.update(position ? t.heading : t.rate, error, slope, ff, dt,
                     fmaxf(-cfg::axleMaxPwm, power - slew), fminf(cfg::axleMaxPwm, power + slew));
  previousGyro = gyro;
}
} // namespace heli
