#include "RpmTuningConsole.h"
#include "Hardware.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

namespace heli {
const char *stateName(State s) {
  const char *names[] = {"wait_off", "off", "spooling", "running", "faulted"};
  return names[unsigned(s)];
}

const char *faultName(Fault s) {
  const char *names[] = {"none",      "receiver", "radio",   "encoder",     "imu",     "timing",
                         "overspeed", "runup",    "low_rpm", "start_stick", "untuned", "settling"};
  return names[unsigned(s)];
}

void RpmConsole::help(Print &p) const {
  p.println("Morpheus | RPM PIDF tuning | CH6 rotor enable, CH3 collective. Yaw and cyclic off.");
  p.println("HELP       command list       STATUS   inputs, state and current output");
  p.println("GAINS      active settings    EXPORT   Config.h gain declarations");
  p.println("REPORT     current or last run duration, peak RPM, peak ESC and stop reason");
  p.println("MONITOR    readable RPM stream; VIEW RPM selects labeled Serial Plotter output");
  p.println("STREAM OFF / STREAM ON    pause/resume USB rows; OpenLog continues");
  p.println("RATE hz    set USB stream to 1..20 Hz");
  p.println("With CH6 off: SET P n | SET I n | SET D n | SET FR n | SET FC n");
  p.println("FR = ESC units per target RPM; FC = additional ESC units per RPM per degree from center.");
  p.println("FFEST esc rpm    estimate FR from a measured steady manual point at center collective");
  p.println("SET TARGET rpm | SET CAP esc_byte   change the next run in RAM; GAINS shows bounds.");
  p.println("Changes last until reset. Use CH6 to start or stop. EXPORT prints gains to keep.");
}

void RpmConsole::observe(const FlightData &f, uint32_t nowMs) {
  if (f.gate.active()) {
    if (!active) {
      hadRun = true;
      startedMs = nowMs;
      runTarget = f.tuning.rpmSetpoint;
      peakEsc = 0;
      peakRpm = f.rotor.rpm;
      stopReason = Fault::None;
    }
    peakRpm = fmaxf(peakRpm, f.rotor.rpm);
    peakEsc = fmaxf(peakEsc, fmaxf(f.out.leftEsc, f.out.rightEsc));
  }
  if (active || f.gate.active())
    durationMs = nowMs - startedMs;
  if (active && !f.gate.active()) {
    stopReason = f.gate.state == State::Faulted ? f.gate.reason : Fault::None;
    if (f.encoderFresh)
      peakRpm = fmaxf(peakRpm, f.rotor.rpm);
  }
  active = f.gate.active();
}

void RpmConsole::report(Print &p) const {
  if (!hadRun) {
    p.println("No run recorded since power-up.");
    return;
  }
  p.print("Run ");
  p.print(active ? "active" : "finished");
  p.print(" | target RPM ");
  p.print(runTarget);
  p.print(" | seconds ");
  p.print(durationMs * 0.001f);
  p.print(" | peak filtered RPM ");
  p.print(peakRpm);
  p.print(" | peak ESC byte ");
  p.print(peakEsc, 0);
  if (!active) {
    p.print(" | stop ");
    p.print(stopReason == Fault::None ? "CH6 off" : faultName(stopReason));
  }
  p.println();
}

// Nine significant decimal digits round-trip an IEEE-754 single, even for tiny D gains.
static void printCoefficient(Print &p, float value, bool literal = false) {
  char text[24];
  snprintf(text, sizeof(text), literal ? "%#.9g" : "%.9g", double(value));
  p.print(text);
  if (literal)
    p.print('f');
}

static void settings(const Tuning &t, Print &p, bool asCode) {
  if (asCode) {
    p.println("// Replace the matching declarations in Config.h");
    p.print("constexpr Gains rpmGains = {");
    printCoefficient(p, t.rpm.kp, true);
    p.print(", ");
    printCoefficient(p, t.rpm.ki, true);
    p.print(", ");
    printCoefficient(p, t.rpm.kd, true);
    p.print(", ");
    printCoefficient(p, t.rpm.integralLimit, true);
    p.print(", ");
    printCoefficient(p, t.rpm.derivativeFilterSeconds, true);
    p.println("};");
    p.print("constexpr float rpmFfPerRpm = ");
    printCoefficient(p, t.rpmF, true);
    p.print(", rpmFfPerRpmDegree = ");
    printCoefficient(p, t.rpmCollectiveF, true);
    p.println(";");
    p.print("// This run's target RPM = ");
    p.print(t.rpmSetpoint);
    p.print(", ESC cap = ");
    p.println(t.rpmEscCap, 0);
    return;
  }
  p.print("P ");
  p.print(t.rpm.kp, 6);
  p.print("  I ");
  p.print(t.rpm.ki, 6);
  p.print("  D ");
  p.print(t.rpm.kd, 6);
  p.print("  FR ");
  p.print(t.rpmF, 6);
  p.print("  FC ");
  p.println(t.rpmCollectiveF, 6);
  p.print("Target ");
  p.print(t.rpmSetpoint);
  p.print(" RPM (must be > ");
  p.print(cfg::minimumRunRpm);
  p.print(" and < ");
  p.print(cfg::overspeedRpm);
  p.print("). ESC cap ");
  p.print(t.rpmEscCap, 0);
  p.print(" (1..");
  p.print(cfg::governorEscMax, 0);
  p.println(" whole byte units).");
  p.print("Ramp RPM/s ");
  p.print(cfg::spoolRpmPerSecond);
  p.print(" | ESC rise units/s ");
  p.print(cfg::escRisePerSecond);
  p.print(" | governor ms ");
  p.print(cfg::governorIntervalUs / 1000.0f);
  p.print(" | RPM window ms ");
  p.println(cfg::rpmWindowUs / 1000.0f);
}

static void status(const FlightData &f, Print &p) {
  p.print("State ");
  p.print(stateName(f.gate.state));
  p.print(" | last reason ");
  p.println(faultName(f.gate.reason));
  p.print("CH6 us ");
  p.print(f.rc.us[cfg::rotorSwitchChannel - 1]);
  p.print(" | CH3 us ");
  p.print(f.rc.us[cfg::collective.channel - 1]);
  p.print(" | governed start ");
  int startUs = cfg::governorStartStick == StartStick::Center ? cfg::collective.centerUs
                : cfg::governorStartStick == StartStick::Low  ? cfg::collective.lowUs
                                                              : cfg::customStartUs;
  p.print(startUs);
  p.print(" +/- ");
  p.print(cfg::governorStartToleranceUs);
  p.println(" us");
  p.print("RC fresh ");
  p.print(f.rc.fresh);
  p.print(" | encoder fresh ");
  p.print(f.encoderFresh);
  p.print(" | encoder indexed ");
  p.print(f.rotor.indexed);
  p.print(" | phase lost ");
  p.print(f.rotor.lost);
  p.print(" | local radio TX healthy ");
  p.print(f.linkFresh);
  p.print(" | TX Hz ");
  p.println(txRate(), 1);
  p.print("Filtered RPM ");
  p.print(f.rotor.rpm);
  p.print(" | window RPM ");
  p.print(f.rotor.windowRpm);
  p.print(" | ramp target ");
  p.print(f.governor.target);
  p.print(" | ESC L/R ");
  p.print(f.out.leftEsc);
  p.print('/');
  p.print(f.out.rightEsc);
  p.print(" | collective deg ");
  p.println(f.out.collective);
  if (!f.rotor.indexed || f.rotor.lost)
    p.println("Align the right wingtip to aircraft right and reboot to reindex.");
  if (f.tuning.rpm.kp == 0 && f.tuning.rpm.ki == 0 && f.tuning.rpm.kd == 0 && f.tuning.rpmF == 0 &&
      f.tuning.rpmCollectiveF == 0)
    p.println("Coefficients are zero. Enter a bench coefficient with CH6 off before requesting a run.");
}

static bool number(const char *text, float &value) {
  if (!text || !*text)
    return false;
  char *end;
  double n = strtod(text, &end);
  if (*end || !isfinite(n) || fabs(n) > 10000)
    return false;
  value = float(n);
  return true;
}

bool RpmConsole::command(FlightData &f, const char *op, const char *key, const char *value, Print &p) {
  if (!strcmp(op, "HELP") && !key) {
    help(p);
    return true;
  }
  if (!strcmp(op, "STATUS") && !key) {
    status(f, p);
    return true;
  }
  if (!strcmp(op, "GAINS") && !key) {
    settings(f.tuning, p, false);
    return true;
  }
  if (!strcmp(op, "EXPORT") && !key) {
    settings(f.tuning, p, true);
    return true;
  }
  if (!strcmp(op, "REPORT") && !key) {
    report(p);
    return true;
  }
  if (!strcmp(op, "MONITOR") && !key) {
    monitor = stream = true;
    p.println("Readable RPM stream on.");
    return true;
  }
  if (!strcmp(op, "STREAM") && key && !value && (!strcmp(key, "ON") || !strcmp(key, "OFF"))) {
    stream = !strcmp(key, "ON");
    p.println(stream ? "USB stream on." : "USB stream paused. Commands and OpenLog still active.");
    return true;
  }
  float n = 0;
  if (!strcmp(op, "FFEST")) {
    float rpm;
    if (!number(key, n) || !number(value, rpm) || n <= 0 || n > 255 || rpm <= 0) {
      p.println("Use FFEST esc rpm with a positive common ESC byte and measured positive steady RPM.");
      return true;
    }
    float estimate = n / rpm;
    if (!isfinite(estimate) || estimate > 10000) {
      p.println("Point outside the gain range. Check the measured RPM.");
      return true;
    }
    p.print("Estimated FR = ");
    printCoefficient(p, estimate);
    p.println(" ESC units/RPM (center collective). Nothing applied.");
    return true;
  }
  if (!strcmp(op, "RATE") && key && !value) {
    if (!number(key, n) || n < 1 || n > 20 || n != floorf(n))
      p.println("Use RATE 1..20 in whole Hz.");
    else {
      intervalMs = uint32_t(lroundf(1000.0f / n));
      p.print("USB rate Hz = ");
      p.println(n, 0);
    }
    return true;
  }
  if (strcmp(op, "SET") || !key || (strcmp(key, "TARGET") && strcmp(key, "CAP")))
    return false;
  if (f.gate.state != State::Off) {
    p.println("Set CH6 off before changing the run settings.");
    return true;
  }
  bool target = !strcmp(key, "TARGET");
  if (!number(value, n) || (target ? n <= cfg::minimumRunRpm || n >= cfg::overspeedRpm
                                   : n < 1 || n > cfg::governorEscMax || n != floorf(n))) {
    p.println("Outside the tuning range. Type GAINS for the current bounds.");
    return true;
  }
  if (target)
    f.tuning.rpmSetpoint = n;
  else
    f.tuning.rpmEscCap = n;
  p.print(key);
  p.print(" = ");
  p.print(n);
  p.println(" (RAM, next run).");
  return true;
}
} // namespace heli
