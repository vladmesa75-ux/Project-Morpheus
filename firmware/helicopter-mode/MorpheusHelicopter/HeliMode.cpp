#include "Control.h"
#include <stdlib.h>
namespace heli {
Modes modesFor(Profile profile) {
  if (profile == Profile::RpmTuning)
    return {RotorMode::Governor, CollectiveMode::Stick, YawMode::Off, false, false};
  if (profile == Profile::YawTuning)
    return {RotorMode::Manual, CollectiveMode::Fixed,
            cfg::flight.yaw == YawMode::Off ? YawMode::Heading : cfg::flight.yaw, false, false};
  return cfg::flight;
}
static Fault healthFault(Modes m, Health h, float rpm) {
  if (!h.rc)
    return Fault::Receiver;
  if (!h.radio)
    return Fault::Radio;
  if (!h.timing)
    return Fault::Timing;
  if (!h.encoder)
    return Fault::Encoder;
  if (m.yaw != YawMode::Off && !h.imu)
    return Fault::Imu;
  if (fabsf(rpm) > cfg::overspeedRpm)
    return Fault::Overspeed;
  if (!h.tuned)
    return Fault::Untuned;
  return Fault::None;
}
static bool atStartStick(RotorMode mode, int us) {
  if (mode == RotorMode::Manual)
    return abs(us - cfg::throttleOffUs) <= cfg::throttleOffToleranceUs;
  int target = cfg::customStartUs;
  if (cfg::governorStartStick == StartStick::Low)
    target = cfg::collective.lowUs;
  if (cfg::governorStartStick == StartStick::Center)
    target = cfg::collective.centerUs;
  return abs(us - target) <= cfg::governorStartToleranceUs;
}
void Gate::step(Modes modes, Health h, int switchUs, int stickUs, float rpm, uint32_t nowMs,
                float windowRpm) {
  bool definiteOff = cfg::switchHighRuns ? switchUs <= cfg::switchLowUs : switchUs >= cfg::switchHighUs;
  bool definiteOn = cfg::switchHighRuns ? switchUs >= cfg::switchHighUs : switchUs <= cfg::switchLowUs;
  if (!h.rc) {
    switchRun = offSeen = false;
    state = State::Faulted;
    reason = Fault::Receiver;
    return;
  }
  if (definiteOff) {
    switchRun = false;
    offSeen = true;
    state = State::Off;
    lowSpeed = false;
    return;
  }
  bool rising = definiteOn && !switchRun;
  if (definiteOn)
    switchRun = true;
  if (!active()) {
    if (!rising || !offSeen || modes.rotor == RotorMode::Off)
      return;
    offSeen = false;
    reason = healthFault(modes, h, fmaxf(fabsf(rpm), fabsf(windowRpm)));
    if (reason == Fault::None && !h.settled)
      reason = Fault::Settling;
    if (reason == Fault::None && !atStartStick(modes.rotor, stickUs))
      reason = Fault::StartStick;
    if (reason != Fault::None) {
      state = State::Faulted;
      return;
    }
    startMs = nowMs;
    lowSpeed = false;
    state = modes.rotor == RotorMode::Governor ? State::Starting : State::Running;
  }
  Fault fault = healthFault(modes, h, fmaxf(fabsf(rpm), fabsf(windowRpm)));
  if (state == State::Starting && fault == Fault::None) {
    if (rpm >= cfg::minimumRunRpm)
      state = State::Running;
    else if (elapsed(nowMs, startMs, cfg::spoolDeadlineMs))
      fault = Fault::Runup;
  }
  if (state == State::Running && modes.rotor == RotorMode::Governor && rpm < cfg::minimumRunRpm) {
    if (!lowSpeed) {
      lowSinceMs = nowMs;
      lowSpeed = true;
    }
    if (elapsed(nowMs, lowSinceMs, cfg::lowRpmTimeoutMs) && fault == Fault::None)
      fault = Fault::LowRpm;
  } else
    lowSpeed = false;
  if (fault != Fault::None) {
    reason = fault;
    state = State::Faulted;
    offSeen = false;
  }
}
} // namespace heli
