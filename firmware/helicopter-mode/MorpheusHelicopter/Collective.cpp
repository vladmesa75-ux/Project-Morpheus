#include "Control.h"
#include <stdlib.h>
namespace heli {
float stickValue(const Stick &s, const Inputs &rc) {
  float delta = int(rc.us[s.channel - 1]) - int(s.centerUs);
  if (s.highUs < s.lowUs)
    delta = -delta;
  float span = delta < 0 ? abs(int(s.lowUs) - int(s.centerUs)) : abs(int(s.highUs) - int(s.centerUs));
  float magnitude = limit((fabsf(delta) - s.deadbandUs) / (span - s.deadbandUs), 0, 1);
  return delta < 0 ? -magnitude : magnitude;
}
float collectiveAngle(CollectiveMode mode, const Inputs &rc) {
  if (mode == CollectiveMode::Fixed)
    return cfg::fixedCollectiveDeg;
  float stick = stickValue(cfg::collective, rc);
  float span = stick < 0 ? cfg::collectiveCenterDeg - cfg::collectiveLowDeg
                         : cfg::collectiveHighDeg - cfg::collectiveCenterDeg;
  return cfg::collectiveCenterDeg + stick * span;
}
float manualThrottle(const Inputs &rc) {
  return 255 * limit((int(rc.us[cfg::collective.channel - 1]) - cfg::throttleOffUs) /
                         float(cfg::throttleFullUs - cfg::throttleOffUs),
                     0, 1);
}
} // namespace heli
