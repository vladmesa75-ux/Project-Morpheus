#include "MorpheusHelicopter.h"
#include "Hardware.h"
#include "FlightData.h"
#include <cassert>
#include <cstdio>
#include <cstring>
using namespace heli;
namespace {
uint32_t tick = 0;
Inputs rc;
bool encoderOk = true, radioOk = true, setGains = false;
int axle = 0;
Outputs packet;
FlightData snapshot;
uint32_t txTick = 0;
} // namespace
uint32_t micros() {
  return tick;
}
uint32_t millis() {
  return tick / 1000;
}
namespace heli {
void beginHardware() {}
Inputs receiver() {
  return rc;
}
bool readEncoder(uint16_t &raw) {
  raw = 0;
  return encoderOk;
}
bool sampleImu(Attitude &a, uint32_t) {
  a.yaw = 0;
  a.gz = 0;
  return true;
}
void driveAxle(int pwm) {
  axle = pwm;
}
void pollRadio(uint32_t) {}
bool radioHealthy(uint32_t) {
  return radioOk;
}
bool radioReady(uint32_t now) {
  return elapsed(now, txTick, 12500);
}
void transmit(const Outputs &out, bool, uint32_t now) {
  packet = out;
  txTick = now;
}
float txRate() {
  return 80;
}
void startTelemetry(Profile) {}
void console(FlightData &f) {
  if (setGains) {
    f.tuning.rpm.kp = 1;
    f.tuning.heading.kp = 10;
    f.tuning.rate.kp = 1;
  }
}
void logTelemetry(const FlightData &f, uint32_t) {
  snapshot = f;
}
} // namespace heli
static void run(unsigned ms) {
  for (unsigned i = 0; i < ms; i++) {
    tick += 1000;
    service();
  }
}
int main(int argc, char **argv) {
  assert(argc == 2);
  bool yawTest = !strcmp(argv[1], "yaw"), rpmTest = !strcmp(argv[1], "rpm");
  Profile profile = yawTest ? Profile::YawTuning : rpmTest ? Profile::RpmTuning : Profile::Flight;
  for (auto &pulse : rc.us)
    pulse = 1500;
  rc.fresh = true;
  rc.us[5] = 2000;
  rc.us[2] = yawTest ? 2000 : 1500;
  begin(profile);
  run(3300);
  assert(!snapshot.gate.active() && !packet.leftEsc && axle == 0);
  rc.us[5] = 1000;
  run(50);
  rc.us[5] = 2000;
  run(50);
  assert(snapshot.gate.reason == Fault::Untuned && !snapshot.gate.active());
  rc.us[5] = 1000;
  setGains = true;
  run(50);
  rc.us[5] = 2000;
  run(50);
  assert(snapshot.gate.active());
  rc.us[3] = 2000;
  if (yawTest)
    rc.us[2] = 1500;
  run(1500);
  assert(packet.leftEsc > 0 && packet.rightEsc == packet.leftEsc);
  if (rpmTest)
    assert(!axle);
  else {
    assert(axle > 0);
    assert(snapshot.yaw.target <= cfg::yawMaxLeadDeg + 0.001f);
  }
  assert(!snapshot.cyclic);
  if (yawTest)
    assert(snapshot.out.collective == cfg::fixedCollectiveDeg);
  rc.us[5] = 1000;
  run(20);
  assert(!packet.leftEsc && !packet.rightEsc && axle == 0 && !snapshot.gate.active());
  rc.us[2] = yawTest ? 2000 : 1500;
  rc.us[5] = 2000;
  run(50);
  assert(snapshot.gate.active());
  if (yawTest) {
    radioOk = false;
    run(20);
    assert(snapshot.gate.reason == Fault::Radio);
  } else {
    encoderOk = false;
    run(60);
    assert(snapshot.gate.reason == Fault::Encoder);
  }
  assert(!packet.leftEsc && !packet.rightEsc && axle == 0);
  radioOk = encoderOk = true;
  run(100);
  assert(!snapshot.gate.active());
  if (!yawTest)
    assert(snapshot.rotor.lost);
  std::printf("Passed full setup/service path: %s\n", argv[1]);
}
