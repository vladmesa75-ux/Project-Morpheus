// Build/run instructions are in ../README.md. These exercise the flight tabs directly.
#include "Control.h"
#include <cassert>
#include <cmath>
#include <cstdio>
using namespace heli;
static void near(float a, float b, float tol = 0.002f) {
  assert(std::fabs(a - b) < tol);
}

static void rotorGeometry() {
  Rotor r;
  // A 90-degree rotor turn is exactly 1,376 encoder counts at 1.34375:1.
  uint32_t t = UINT32_MAX - 40000;
  int raw = 3600;
  r.index(raw, t);
  for (int quarter = 1; quarter <= 12; quarter++) {
    for (int i = 0; i < 32; i++) {
      raw = (raw + 43) & 4095;
      t += 5000;
      r.sample(raw, t);
    }
    near(r.phase, float((90 * quarter) % 360));
  }
  near(r.rpm, 93.75f);
  assert(r.speedReady && r.healthy(t));
  for (int i = 0; i < 32; i++) {
    raw = (raw - 43) & 4095;
    t += 5000;
    r.sample(raw, t);
  }
  near(r.phase, 270);
  r.sample((raw + 2048) & 4095, t + 5000);
  near(r.phase, 270);
  r.sample(raw, t + cfg::encoderStaleUs + 1);
  assert(r.lost && !r.healthy(t + cfg::encoderStaleUs + 1));
  r.sample(raw, t + cfg::encoderStaleUs + 2);
  assert(r.lost);
}
static void angleMixing() {
  Inputs in;
  in.us[cfg::collective.channel - 1] = 1500;
  near(collectiveAngle(CollectiveMode::Stick, in), 5);
  in.us[cfg::collective.channel - 1] = 2000;
  near(collectiveAngle(CollectiveMode::Stick, in), cfg::collectiveLowDeg);
  near(manualThrottle(in), 0);
  in.us[cfg::collective.channel - 1] = 1000;
  near(collectiveAngle(CollectiveMode::Stick, in), cfg::collectiveHighDeg);
  near(manualThrottle(in), 255);
  near(collectiveAngle(CollectiveMode::Fixed, in), cfg::fixedCollectiveDeg);
  Outputs o;
  o.collective = 10;
  setSurfaces(o, 15);
  near(o.left.requested, 25);
  near(o.left.angle, 20);
  near(o.right.angle, -5);
  assert(o.left.command == cfg::leftSurface.signToNano * 127);
  assert(o.right.command == cfg::rightSurface.signToNano * -32);
  setEscs(o, 90, 64);
  assert(o.leftEsc == 64 && o.rightEsc == 64);
  setEscs(o, 0, 64);
  assert(!o.leftEsc && !o.rightEsc);
  uint8_t p[6];
  pack(o, true, p);
  assert(p[0] == uint8_t(o.left.command) && p[4] == 255);
  assert((p[0] ^ p[1] ^ p[2] ^ p[3] ^ p[4] ^ p[5]) == 0);
  const int knots[] = {-127, -95, -64, -32, 0, 32, 64, 95, 127};
  for (int i = 0; i < 9; i++) {
    o.collective = -20 + 5 * i;
    setSurfaces(o, 0);
    assert(o.left.command == cfg::leftSurface.signToNano * knots[i]);
  }
  Rotor r;
  r.phase = 0;
  r.lastUs = 100;
  r.rpm = 0;
  Modes m = modesFor(Profile::Flight);
  m.rollCyclic = m.pitchCyclic = true;
  in.us[cfg::roll.channel - 1] = cfg::roll.highUs;
  in.us[cfg::pitch.channel - 1] = cfg::pitch.centerUs;
  o.collective = 5;
  mixCyclic(o, m, in, r, true, 100);
  near(o.left.requested, 10);
  near(o.right.requested, 0);
  r.phase = 180;
  mixCyclic(o, m, in, r, true, 100);
  near(o.left.requested, 0);
  near(o.right.requested, 10);
  in.us[cfg::roll.channel - 1] = cfg::roll.centerUs;
  in.us[cfg::pitch.channel - 1] = cfg::pitch.highUs;
  r.phase = 90;
  mixCyclic(o, m, in, r, true, 100);
  near(o.left.requested, 10);
  mixCyclic(o, m, in, r, false, 100);
  near(o.left.angle, 5);
  near(o.right.angle, 5);
}
static void yawTracking() {
  Tuning tune;
  tune.heading = {2, 0, 0.2f, 40, 0};
  tune.yawRateF = 0.1f;
  Yaw y;
  y.start(179, 0);
  for (int i = 0; i < 100; i++)
    y.step(YawMode::Heading, 1, 179, 0, 0, 0.01f, tune);
  near(wrap180(y.target - 179), 3);
  near(y.referenceRate, 0);
  near(y.pid.f, 0);
  assert(y.leadLimited);
  y.step(YawMode::Heading, 1, -179, 0, 0, 0.01f, tune);
  near(y.target, -177.7f);
  near(y.referenceRate, 30);
  float last = y.target;
  y.step(YawMode::Heading, 0, 150, 0, 0, 0.01f, tune);
  near(y.target, last);
  y.step(YawMode::Heading, 1, 150, 0, 0, 0.01f, tune);
  near(y.target, last);
  y.step(YawMode::Heading, -1, 150, 0, 0, 0.01f, tune);
  near(y.target, last - 0.3f);
  y.start(-179, 0);
  for (int i = 0; i < 100; i++)
    y.step(YawMode::Heading, -1, -179, 0, 0, 0.01f, tune);
  near(wrap180(y.target + 179), -3);
  y.step(YawMode::Off, 1, 25, 0, 0, 0.01f, tune);
  near(y.power, 0);
  near(y.target, 25);
  tune.rate = {1, 0, 0, 40, 0};
  y.step(YawMode::Rate, 0.5f, 25, 8, 0, 0.01f, tune);
  near(y.pid.p, 7);
  near(y.pid.f, 1.5f);
}
static void pidAndGovernor() {
  Pidf pid;
  Gains g = {1, 10, 1, 10, 0};
  for (int i = 0; i < 1000; i++)
    pid.update(g, 20, 0, 2, 0.01f, 0, 10);
  near(pid.output, 10);
  near(pid.i, 0);
  assert(pid.limited);
  pid.update(g, 1, 2, 0, 0.01f, 0, 10);
  near(pid.d, -2);
  Governor gov;
  Tuning tune;
  tune.rpmF = 2;
  gov.start(0);
  gov.step(0, 5, 0.1f, tune);
  near(gov.target, 1);
  near(gov.pid.f, 2);
  near(gov.thrust, 2);
  for (int i = 0; i < 100; i++)
    gov.step(0, 5, 0.1f, tune);
  near(gov.target, 50);
  near(gov.thrust, cfg::governorEscMax);
  gov.start(50);
  near(gov.target, 0);
  near(gov.thrust, 0);
  near(gov.pid.i, 0);
}
static void interlocks() {
  const Modes governor = modesFor(Profile::RpmTuning), manual = modesFor(Profile::YawTuning);
  assert(manual.rotor == RotorMode::Manual && manual.collective == CollectiveMode::Fixed &&
         !manual.rollCyclic);
  assert(governor.yaw == YawMode::Off);
  Health ok = {true, true, true, true, true, true, true};
  Gate g;
  g.step(governor, ok, 2000, 1500, 0, 0);
  assert(!g.active());
  g.step(governor, ok, 1000, 1500, 0, 1);
  g.step(governor, ok, 2000, 2000, 0, 2);
  assert(g.reason == Fault::StartStick && !g.active());
  g.step(governor, ok, 2000, 1500, 0, 3);
  assert(!g.active());
  g.step(governor, ok, 1000, 1500, 0, 4);
  g.step(governor, ok, 2000, 1500, 0, 5);
  assert(g.state == State::Starting);
  g.step(governor, ok, 1660, 1500, 25, 6);
  assert(g.state == State::Running);
  g.step(governor, ok, 2000, 1500, 0, 7);
  g.step(governor, ok, 2000, 1500, 0, 7 + cfg::lowRpmTimeoutMs);
  assert(g.reason == Fault::LowRpm);
  g.step(manual, ok, 1000, 2000, 0, 3000);
  g.step(manual, ok, 2000, 2000, 0, 3001);
  assert(g.active());
  ok.rc = false;
  g.step(manual, ok, 2000, 1500, 0, 3100);
  assert(!g.active());
  ok.rc = true;
  g.step(manual, ok, 2000, 2000, 0, 3101);
  assert(!g.active());
  g.step(manual, ok, 1000, 2000, 0, 3102);
  g.step(manual, ok, 2000, 2000, 0, 3103);
  ok.imu = false;
  g.step(manual, ok, 2000, 1500, 0, 3104);
  assert(g.reason == Fault::Imu && !g.active());
  // Independent RPM test does not require the yaw IMU.
  g.step(governor, ok, 1000, 1500, 0, 4000);
  g.step(governor, ok, 2000, 1500, 0, 4001);
  assert(g.active());
  g.step(governor, ok, 2000, 1500, 0, 4001 + cfg::spoolDeadlineMs);
  assert(g.reason == Fault::Runup && !g.active());
  ok.imu = true;
  for (int i = 0; i < 5; i++) {
    Health bad = ok;
    g.step(governor, ok, 1000, 1500, 0, 25000);
    g.step(governor, ok, 2000, 1500, 0, 25001);
    if (i == 0)
      bad.encoder = false;
    if (i == 1)
      bad.radio = false;
    if (i == 2)
      bad.timing = false;
    if (i == 3)
      bad.tuned = false;
    g.step(governor, bad, 2000, 1500, i == 4 ? cfg::overspeedRpm + 1 : 0, 25002);
    assert(!g.active());
    g.step(governor, ok, 2000, 1500, 0, 25003);
    assert(!g.active());
  }
}
int main() {
  rotorGeometry();
  angleMixing();
  yawTracking();
  pidAndGovernor();
  interlocks();
  puts("Passed: geared phase, wraps, calibration knots, independent clipping, packet, cyclic, yaw "
       "lead, PIDF and rotor interlocks.");
}
