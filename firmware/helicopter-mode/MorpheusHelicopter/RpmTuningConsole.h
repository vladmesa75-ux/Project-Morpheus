#pragma once

#include "FlightData.h"
class Print;

namespace heli {
// USB settings affect only the tuning display. All commands use the shared
// logging queue; none of these functions write directly to a UART.
struct RpmConsole {
  bool stream = true, monitor = true;
  uint32_t intervalMs = cfg::usbIntervalMs;
  void observe(const FlightData &, uint32_t nowMs);
  void help(Print &) const;
  bool command(FlightData &, const char *op, const char *key, const char *value, Print &);

private:
  bool active = false, hadRun = false;
  uint32_t startedMs = 0, durationMs = 0;
  float peakRpm = 0, peakEsc = 0, runTarget = 0;
  Fault stopReason = Fault::None;
  void report(Print &) const;
};
const char *stateName(State);
const char *faultName(Fault);
} // namespace heli
