#include "FlightData.h"
#include "Hardware.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

namespace heli {
namespace {
// Each UART drains its own completed row only into free TX buffer space.
struct Row : Print {
  char bytes[1400];
  size_t size = 0, sent = 0;
  bool overflow = false;
  size_t write(uint8_t b) override {
    if (size >= sizeof(bytes)) {
      overflow = true;
      return 0;
    }
    bytes[size++] = char(b);
    return 1;
  }
  void clear() {
    size = sent = 0;
    overflow = false;
  }
  void drain(UARTClass &port) {
    if (!size)
      return;
    size_t room = port.availableForWrite();
    size_t count = size - sent < room ? size - sent : room;
    if (count)
      sent += port.write((const uint8_t *)bytes + sent, count);
    if (sent == size)
      clear();
  }
};
struct Sink {
  Row row;
  View view = View::Status;
  bool needsHeader = true;
  uint32_t lastMs = 0, skipped = 0;
};
Sink usb, logSink;
Row reply;
char command[60];
size_t used = 0;
bool discard = false;
const char *stateName(State s) {
  const char *names[] = {"wait_off", "off", "spooling", "running", "faulted"};
  return names[unsigned(s)];
}
const char *faultName(Fault s) {
  const char *names[] = {"none",      "receiver", "radio",   "encoder",     "imu",     "timing",
                         "overspeed", "runup",    "low_rpm", "start_stick", "untuned", "settling"};
  return names[unsigned(s)];
}
struct Fields {
  Row &row;
  bool csv, header, first;
  const char *selected;
  bool include(const char *name) const {
    if (!*selected)
      return true;
    size_t n = strlen(name);
    for (const char *p = selected; *p;) {
      const char *end = strchr(p, ',');
      size_t length = end ? size_t(end - p) : strlen(p);
      if (n == length && !strncmp(p, name, n))
        return true;
      if (!end)
        break;
      p = end + 1;
    }
    return false;
  }
  void add(const char *name, float value, int digits = 2) {
    if (!include(name))
      return;
    if (!first)
      row.print(csv ? ',' : '\t');
    first = false;
    if (header) {
      row.print(name);
      return;
    }
    if (!csv) {
      row.print(name);
      row.print(':');
    }
    row.print(value, digits);
  }
  void integer(const char *name, uint32_t value) {
    if (!include(name))
      return;
    if (!first)
      row.print(csv ? ',' : '\t');
    first = false;
    if (header) {
      row.print(name);
      return;
    }
    if (!csv) {
      row.print(name);
      row.print(':');
    }
    row.print(value);
  }
  void terms(const Pidf &pid, bool rpm) {
    add(rpm ? "rpm_p" : "yaw_p", pid.p);
    add(rpm ? "rpm_i" : "yaw_i", pid.i);
    add(rpm ? "rpm_d" : "yaw_d", pid.d);
    add(rpm ? "rpm_f" : "yaw_f", pid.f);
    integer(rpm ? "rpm_limited" : "yaw_limited", pid.limited);
  }
};
void render(Sink &sink, const FlightData &f, uint32_t ms, bool csv, bool header) {
  Row &row = sink.row;
  View v = sink.view;
  bool all = v == View::All;
  Fields x = {row, csv, header, true, csv ? cfg::openLogFields : cfg::usbFields};
  if (csv || all || v == View::Status)
    x.integer("ms", ms);
  if (v == View::Status || all) {
    x.integer("profile", unsigned(f.profile));
    x.integer("state", unsigned(f.gate.state));
    x.integer("last_reason", unsigned(f.gate.reason));
    x.integer("ch6_run", f.gate.switchRun);
    x.integer("rotor_mode", unsigned(f.modes.rotor));
    x.integer("yaw_mode", unsigned(f.modes.yaw));
    x.integer("collective_mode", unsigned(f.modes.collective));
    x.integer("roll_on", f.modes.rollCyclic);
    x.integer("pitch_on", f.modes.pitchCyclic);
    x.integer("rc_ok", f.rc.fresh);
    x.integer("rc_age_us", f.rc.ageUs);
    x.integer("encoder_ok", f.encoderFresh);
    x.integer("imu_ok", f.imuFresh);
    x.integer("imu_age_us", f.imuAgeUs);
    x.integer("radio_tx_ok", f.linkFresh);
    x.add("tx_hz", txRate(), 1);
    x.integer("loop_us", f.loopUs);
    x.integer("log_skips", sink.skipped);
    const char *channels[] = {"ch1_us", "ch2_us", "ch3_us", "ch4_us", "ch5_us", "ch6_us"};
    for (unsigned i = 0; i < 6; i++)
      x.integer(channels[i], f.rc.us[i]);
  }
  if (all || v == View::Rpm || v == View::Status || v == View::Yaw)
    x.add("rpm", f.rotor.rpm);
  if (all || v == View::Rpm) {
    x.add("rpm_window", f.rotor.windowRpm);
    x.add("rpm_set", cfg::targetRpm);
    x.add("rpm_ramp", f.governor.target);
    x.add("governor_esc", f.governor.thrust);
    x.terms(f.governor.pid, true);
  }
  if (all || v == View::Rpm || v == View::Yaw || v == View::Status) {
    x.integer("esc_l", f.out.leftEsc);
    x.integer("esc_r", f.out.rightEsc);
    x.add("axle_pwm", f.out.axle, 0);
  }
  if (all || v == View::Yaw || v == View::Status) {
    x.add("yaw_deg", f.attitude.yaw);
    x.add("roll_deg", f.attitude.roll);
    x.add("pitch_deg", f.attitude.pitch);
  }
  if (all || v == View::Yaw) {
    x.add("yaw_target", f.yaw.target);
    x.add("yaw_error", wrap180(f.yaw.target - f.attitude.yaw));
    x.add("yaw_stick_dps", f.yaw.stickRate);
    x.add("yaw_ref_dps", f.yaw.referenceRate);
    x.add("yaw_dps", f.attitude.gz);
    x.add("roll_dps", f.attitude.gx);
    x.add("pitch_dps", f.attitude.gy);
    x.integer("lead_limit", f.yaw.leadLimited);
    x.terms(f.yaw.pid, false);
  }
  if (all || v == View::Cyclic || v == View::Rpm)
    x.add("collective_deg", f.out.collective);
  if (all || v == View::Cyclic) {
    x.add("rotor_deg", f.rotor.phase);
    x.add("mix_phase", f.out.mixPhase);
    x.integer("cyclic_ready", f.cyclic);
    x.add("roll_cyclic", f.out.roll);
    x.add("pitch_cyclic", f.out.pitch);
    x.add("l_req", f.out.left.requested);
    x.add("l_deg", f.out.left.angle);
    x.add("l_byte", f.out.left.command, 0);
    x.add("r_req", f.out.right.requested);
    x.add("r_deg", f.out.right.angle);
    x.add("r_byte", f.out.right.command, 0);
  }
  if (!csv && v == View::Status) {
    row.print("\tmode:");
    row.print(stateName(f.gate.state));
    row.print("\treason:");
    row.print(faultName(f.gate.reason));
  }
  row.println();
  if (row.overflow) {
    row.clear();
    ++sink.skipped;
  }
}
void emit(Sink &sink, UARTClass &port, const FlightData &f, uint32_t ms, uint32_t interval, bool csv) {
  sink.row.drain(port);
  if (csv && sink.needsHeader && !sink.row.size) {
    render(sink, f, ms, true, true);
    sink.needsHeader = false;
    return;
  }
  if (!elapsed(ms, sink.lastMs, interval))
    return;
  sink.lastMs = ms;
  if (sink.row.size) {
    ++sink.skipped;
    return;
  }
  render(sink, f, ms, csv, false);
}
void process(FlightData &f, char *line) {
  char *save = nullptr;
  char *op = strtok_r(line, " \t", &save);
  if (!op)
    return;
  char *key = strtok_r(nullptr, " \t", &save);
  char *val = strtok_r(nullptr, " \t", &save);
  if (!strcmp(op, "VIEW") && key) {
    const char *names[] = {"STATUS", "RPM", "YAW", "CYCLIC", "ALL"};
    for (unsigned i = 0; i < 5; i++)
      if (!strcmp(key, names[i])) {
        usb.view = View(i);
        reply.println("OK");
        return;
      }
  }
  bool rpm = f.profile == Profile::RpmTuning;
  Gains &g = rpm ? f.tuning.rpm : (f.modes.yaw == YawMode::Rate ? f.tuning.rate : f.tuning.heading);
  if (!strcmp(op, "GAINS")) {
    reply.print("P I D F0 FR FC FV: ");
    reply.print(g.kp, 5);
    reply.print(' ');
    reply.print(g.ki, 5);
    reply.print(' ');
    reply.print(g.kd, 5);
    reply.print(' ');
    reply.print(rpm ? 0 : f.tuning.yawBias, 5);
    reply.print(' ');
    reply.print(rpm ? f.tuning.rpmF : f.tuning.yawRpmF, 5);
    reply.print(' ');
    reply.print(rpm ? f.tuning.rpmCollectiveF : 0, 5);
    reply.print(' ');
    reply.println(rpm ? 0 : f.tuning.yawRateF, 5);
    return;
  }
  if (!strcmp(op, "SET") && key && val && !strtok_r(nullptr, " \t", &save)) {
    if (f.profile == Profile::Flight) {
      reply.println("Use a tuning sketch for SET");
      return;
    }
    if (f.gate.state != State::Off) {
      reply.println("Set CH6 off before changing gains");
      return;
    }
    char *end = nullptr;
    double n = strtod(val, &end);
    if (*end || !isfinite(n) || fabs(n) > 10000) {
      reply.println("Invalid gain");
      return;
    }
    float *dest = nullptr;
    if (!strcmp(key, "P") && n >= 0)
      dest = &g.kp;
    if (!strcmp(key, "I") && n >= 0)
      dest = &g.ki;
    if (!strcmp(key, "D") && n >= 0)
      dest = &g.kd;
    if (!strcmp(key, "FR"))
      dest = rpm ? &f.tuning.rpmF : &f.tuning.yawRpmF;
    if (!strcmp(key, "FC") && rpm)
      dest = &f.tuning.rpmCollectiveF;
    if (!strcmp(key, "F0") && !rpm)
      dest = &f.tuning.yawBias;
    if (!strcmp(key, "FV") && !rpm)
      dest = &f.tuning.yawRateF;
    if (dest) {
      *dest = float(n);
      reply.print(key);
      reply.print('=');
      reply.println(*dest, 5);
      return;
    }
  }
  reply.println("VIEW STATUS/RPM/YAW/CYCLIC/ALL | GAINS | tuning: SET key value");
}
} // namespace
void startTelemetry(Profile profile) {
  if (cfg::usbEnabled)
    Serial.begin(cfg::usbBaud);
  if (cfg::openLogEnabled)
    Serial2.begin(cfg::openLogBaud);
  usb.view = profile == Profile::RpmTuning   ? View::Rpm
             : profile == Profile::YawTuning ? View::Yaw
                                             : cfg::usbView;
  logSink.view = cfg::openLogView;
}
void console(FlightData &f) {
  if (!cfg::usbEnabled || reply.size)
    return;
  for (unsigned n = 0; n < 16 && Serial.available(); n++) {
    char c = char(Serial.read());
    if (c == '\r')
      continue;
    if (c == '\n') {
      command[used] = 0;
      if (discard)
        reply.println("Command too long");
      else
        process(f, command);
      used = 0;
      discard = false;
      return;
    }
    if (used + 1 < sizeof(command))
      command[used++] = char(toupper((unsigned char)c));
    else
      discard = true;
  }
}
void logTelemetry(const FlightData &f, uint32_t now) {
  if (cfg::usbEnabled) {
    if (reply.size) {
      usb.row.drain(Serial);
      if (!usb.row.size)
        reply.drain(Serial);
    } else
      emit(usb, Serial, f, now, cfg::usbIntervalMs, false);
  }
  if (cfg::openLogEnabled)
    emit(logSink, Serial2, f, now, cfg::openLogIntervalMs, true);
}
} // namespace heli
