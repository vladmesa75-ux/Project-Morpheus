#pragma once

#include "Config.h"
#include <math.h>
#include <stdint.h>

namespace heli {
inline float limit(float v, float lo, float hi) {
  return fminf(hi, fmaxf(lo, v));
}
inline float wrap180(float deg) {
  float a = fmodf(deg + 180.0f, 360.0f);
  return (a < 0 ? a + 360.0f : a) - 180.0f;
}
inline float wrap360(float deg) {
  float a = fmodf(deg, 360);
  return a < 0 ? a + 360 : a;
}
inline float seconds(uint32_t us) {
  return us * 1e-6f;
}
inline bool elapsed(uint32_t now, uint32_t then, uint32_t period) {
  return now - then >= period;
}

struct Pidf {
  float p = 0, i = 0, d = 0, f = 0, slope = 0, output = 0;
  bool limited = false;
  float update(const Gains &, float error, float measuredSlope, float feedforward, float dt, float low,
               float high);
};
struct Rotor {
  bool indexed = false, lost = false, speedReady = false;
  uint16_t previousRaw = 0;
  uint32_t lastUs = 0, windowStartUs = 0;
  float phase = 0, rpm = 0, windowRpm = 0, windowDeg = 0;
  void index(uint16_t raw, uint32_t now);
  void sample(uint16_t raw, uint32_t now);
  bool healthy(uint32_t now) const {
    return indexed && !lost && now - lastUs < cfg::encoderStaleUs;
  }
};
struct Tuning {
  Gains rpm = cfg::rpmGains, heading = cfg::headingGains, rate = cfg::rateGains;
  float rpmF = cfg::rpmFfPerRpm, rpmCollectiveF = cfg::rpmFfPerRpmDegree;
  float yawBias = cfg::yawFfBias, yawRpmF = cfg::yawFfPerRpm, yawRateF = cfg::yawFfPerTargetDps;
};
struct Governor {
  Pidf pid;
  float target = 0, previousRpm = 0, thrust = 0;
  void start(float rpm);
  void step(float rpm, float collective, float dt, const Tuning &);
};
struct Yaw {
  Pidf pid;
  float target = 0, stickRate = 0, referenceRate = 0, previousGyro = 0, power = 0;
  bool leadLimited = false;
  void start(float heading, float gyro);
  void step(YawMode mode, float stick, float heading, float gyro, float rpm, float dt, const Tuning &);
};
struct Inputs {
  uint16_t us[6] = {};
  bool fresh = false;
  uint32_t ageUs = 0;
};
struct Attitude {
  float roll = 0, pitch = 0, yaw = 0, gx = 0, gy = 0, gz = 0;
};
struct Wing {
  float requested = 0, angle = 0;
  int8_t command = 0;
};
struct Outputs {
  Wing left, right;
  float collective = 0, roll = 0, pitch = 0, mixPhase = 0;
  uint8_t leftEsc = 0, rightEsc = 0;
  int axle = 0;
};
enum class Fault {
  None,
  Receiver,
  Radio,
  Encoder,
  Imu,
  Timing,
  Overspeed,
  Runup,
  LowRpm,
  StartStick,
  Untuned,
  Settling
};
enum class State { WaitOff, Off, Starting, Running, Faulted };
struct Health {
  bool rc, radio, encoder, imu, timing, tuned, settled;
};
struct Gate {
  State state = State::WaitOff;
  Fault reason = Fault::None;
  bool switchRun = false, offSeen = false, lowSpeed = false;
  uint32_t startMs = 0, lowSinceMs = 0;
  bool active() const {
    return state == State::Starting || state == State::Running;
  }
  void step(Modes modes, Health h, int switchUs, int stickUs, float rpm, uint32_t nowMs,
            float windowRpm = 0);
};

Modes modesFor(Profile profile);
float stickValue(const Stick &, const Inputs &);
float collectiveAngle(CollectiveMode, const Inputs &);
float manualThrottle(const Inputs &);
void mixCyclic(Outputs &, Modes, const Inputs &, const Rotor &, bool enabled, uint32_t now);
void setSurfaces(Outputs &, float cyclic);
void stopOutputs(Outputs &, bool fault);
void setEscs(Outputs &, float thrust, float cap);
void pack(const Outputs &, bool failsafe, uint8_t (&packet)[6]);
} // namespace heli
