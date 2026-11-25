#pragma once
#include "Control.h"
namespace heli {
struct FlightData {
  Profile profile = Profile::Flight;
  Modes modes = cfg::flight;
  Tuning tuning;
  Inputs rc;
  Rotor rotor;
  Attitude attitude;
  Gate gate;
  Governor governor;
  Yaw yaw;
  Outputs out;
  bool imuFresh = false, encoderFresh = false, linkFresh = false, cyclic = false;
  uint32_t loopUs = 0, imuAgeUs = 0;
};
void startTelemetry(Profile);
void logTelemetry(const FlightData &, uint32_t nowMs);
void console(FlightData &);
} // namespace heli
