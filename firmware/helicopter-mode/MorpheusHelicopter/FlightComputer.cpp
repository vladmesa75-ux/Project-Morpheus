#include "MorpheusHelicopter.h"
#include "FlightData.h"
#include "Hardware.h"
#include <Arduino.h>

namespace heli {
namespace {
FlightData f;
uint32_t lastCycle = 0, lastSensor = 0, lastImuGood = 0, bootMs = 0;
uint32_t lastGovernor = 0, lastYaw = 0;
bool imuSeen = false, settled = false;
bool gainsSet(const Gains &g) {
  return g.kp != 0 || g.ki != 0 || g.kd != 0;
}
bool tuningPresent() {
  bool rpmReady = f.modes.rotor != RotorMode::Governor || gainsSet(f.tuning.rpm) ||
                  f.tuning.rpmF != 0 || f.tuning.rpmCollectiveF != 0;
  const Gains &y = f.modes.yaw == YawMode::Heading ? f.tuning.heading : f.tuning.rate;
  bool yawReady = f.modes.yaw == YawMode::Off || gainsSet(y) || f.tuning.yawBias != 0 ||
                  f.tuning.yawRpmF != 0 || f.tuning.yawRateF != 0;
  return rpmReady && yawReady;
}
void sampleSensors(uint32_t now) {
  uint16_t raw;
  if (readEncoder(raw))
    f.rotor.sample(raw, now);
  if (sampleImu(f.attitude, now)) {
    lastImuGood = now;
    imuSeen = true;
  }
}
void stepControllers(uint32_t now) {
  f.out.collective = collectiveAngle(f.modes.collective, f.rc);
  if (f.modes.rotor == RotorMode::Governor) {
    if (elapsed(now, lastGovernor, cfg::governorIntervalUs)) {
      f.governor.step(f.rotor.rpm, f.out.collective, seconds(now - lastGovernor), f.tuning);
      lastGovernor = now;
    }
    setEscs(f.out, f.governor.thrust, cfg::governorEscMax);
  } else {
    setEscs(f.out, manualThrottle(f.rc), cfg::manualEscMax);
  }
  if (elapsed(now, lastYaw, cfg::yawIntervalUs)) {
    f.yaw.step(f.modes.yaw, stickValue(cfg::yaw, f.rc), f.attitude.yaw, f.attitude.gz, f.rotor.rpm,
               seconds(now - lastYaw), f.tuning);
    lastYaw = now;
  }
  int axle = int(lroundf(f.yaw.power)) * cfg::axleSign;
  f.out.axle = abs(axle) < cfg::axleDeadband ? 0 : axle;
  if (f.rotor.rpm >= cfg::cyclicEngageRpm)
    f.cyclic = true;
  if (f.rotor.rpm < cfg::cyclicDropoutRpm)
    f.cyclic = false;
}
} // namespace
void begin(Profile profile) {
  f.profile = profile;
  f.modes = modesFor(profile);
  startTelemetry(profile);
  stopOutputs(f.out, false);
  beginHardware();
  uint16_t raw;
  // An unknown starting azimuth cannot be reconstructed from a geared sensor.
  if (readEncoder(raw))
    f.rotor.index(raw, micros());
  bootMs = millis();
  lastCycle = micros();
}
void service() {
  uint32_t now = micros();
  bool imuGap = imuSeen && elapsed(now, lastImuGood, cfg::imuStaleUs);
  if (imuGap)
    imuSeen = false;
  if (f.rotor.indexed && !f.rotor.healthy(now))
    f.rotor.lost = true;
  if (elapsed(now, lastSensor, cfg::sampleUs)) {
    lastSensor = now;
    sampleSensors(now);
  }
  f.rc = receiver();
  now = micros();
  f.loopUs = now - lastCycle;
  lastCycle = now;
  f.imuAgeUs = now - lastImuGood;
  f.imuFresh = imuSeen && f.imuAgeUs < cfg::imuStaleUs && !imuGap;
  f.encoderFresh = f.rotor.healthy(now) && f.rotor.speedReady;
  pollRadio(now);
  f.linkFresh = radioHealthy(now);
  uint32_t nowMs = millis();
  settled = settled || elapsed(nowMs, bootMs, cfg::mechanismSettleMs);
  Health health = {
      f.rc.fresh,      f.linkFresh, f.encoderFresh, f.imuFresh, f.loopUs < cfg::controlGapUs,
      tuningPresent(), settled};
  bool wasActive = f.gate.active();
  f.gate.step(f.modes, health, f.rc.us[cfg::rotorSwitchChannel - 1],
              f.rc.us[cfg::collective.channel - 1], f.rotor.rpm, nowMs, f.rotor.windowRpm);
  if (wasActive != f.gate.active()) {
    f.governor.start(f.rotor.rpm);
    f.yaw.start(f.attitude.yaw, f.attitude.gz);
    lastGovernor = lastYaw = now;
    f.cyclic = false;
  }
  bool fault = f.gate.state == State::Faulted || f.gate.state == State::WaitOff;
  if (f.gate.active())
    stepControllers(now);
  else
    stopOutputs(f.out, fault);
  driveAxle(f.out.axle);
  if (radioReady(now)) {
    now = micros();
    if (f.gate.active())
      mixCyclic(f.out, f.modes, f.rc, f.rotor, f.cyclic, now);
    transmit(f.out, fault, now);
  }
  console(f);
  logTelemetry(f, nowMs);
}
} // namespace heli
