#include "Control.h"
namespace heli {
static float excursion(const Travel &t, float stick) {
  return t.neutral + stick * (stick < 0 ? t.negativeTravel : t.positiveTravel);
}
void mixCyclic(Outputs &o, Modes modes, const Inputs &rc, const Rotor &rotor, bool enabled,
               uint32_t now) {
  o.roll = enabled && modes.rollCyclic ? excursion(cfg::rollCyclic, stickValue(cfg::roll, rc)) : 0;
  o.pitch = enabled && modes.pitchCyclic ? excursion(cfg::pitchCyclic, stickValue(cfg::pitch, rc)) : 0;
  // Phase at packet construction plus adjustable measured command-to-motion lag.
  float delay = seconds(now - rotor.lastUs) + cfg::commandDelayMs * 0.001f;
  o.mixPhase = wrap360(rotor.phase + cfg::phaseOffsetDeg + 6 * rotor.rpm * delay);
  float radians = o.mixPhase * 0.01745329252f;
  setSurfaces(o, o.roll * cosf(radians) + o.pitch * sinf(radians));
}
} // namespace heli
