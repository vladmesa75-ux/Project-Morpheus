#include "Control.h"
namespace heli {
void Governor::start(float rpm) {
  *this = Governor();
  previousRpm = rpm;
}
void Governor::step(float rpm, float collective, float dt, const Tuning &t) {
  if (dt <= 0)
    return;
  target = fminf(cfg::targetRpm, target + cfg::spoolRpmPerSecond * dt);
  float ff = target * (t.rpmF + t.rpmCollectiveF * fabsf(collective - cfg::collectiveCenterDeg));
  float ceiling = fminf(cfg::governorEscMax, thrust + cfg::escRisePerSecond * dt);
  thrust = pid.update(t.rpm, target - rpm, (rpm - previousRpm) / dt, ff, dt, 0, ceiling);
  previousRpm = rpm;
}
} // namespace heli
