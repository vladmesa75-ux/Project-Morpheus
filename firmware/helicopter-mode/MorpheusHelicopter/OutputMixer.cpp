#include "Control.h"
namespace heli {
static Wing surface(float demand, const Surface &s) {
  Wing w;
  w.requested = demand;
  w.angle = limit(demand, s.minDeg, s.maxDeg);
  // The Nano's nine calibration knots use round(degrees * 127 / 20).
  w.command = int8_t(lroundf(s.signToNano * w.angle * (127.0f / 20.0f)));
  return w;
}
void setSurfaces(Outputs &o, float cyclic) {
  o.left = surface(o.collective + cyclic + cfg::leftSurface.trimDeg, cfg::leftSurface);
  o.right = surface(o.collective - cyclic + cfg::rightSurface.trimDeg, cfg::rightSurface);
}
void stopOutputs(Outputs &o, bool fault) {
  o = Outputs();
  o.left = surface(fault ? cfg::leftSurface.faultDeg : cfg::leftSurface.stoppedDeg, cfg::leftSurface);
  o.right =
      surface(fault ? cfg::rightSurface.faultDeg : cfg::rightSurface.stoppedDeg, cfg::rightSurface);
}
static uint8_t esc(float thrust, const Motor &m, float cap) {
  return thrust <= 0 ? 0 : uint8_t(lroundf(limit(thrust * m.scale + m.offset, 0, cap)));
}
void setEscs(Outputs &o, float thrust, float cap) {
  o.leftEsc = esc(thrust, cfg::leftEsc, cap);
  o.rightEsc = esc(thrust, cfg::rightEsc, cap);
}
void pack(const Outputs &o, bool fault, uint8_t (&p)[6]) {
  p[0] = uint8_t(o.left.command);
  p[1] = uint8_t(o.right.command);
  p[2] = o.leftEsc;
  p[3] = o.rightEsc;
  p[4] = fault ? 255 : 0;
  p[5] = p[0] ^ p[1] ^ p[2] ^ p[3] ^ p[4];
}
} // namespace heli
