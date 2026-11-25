#pragma once
#include "Control.h"
namespace heli {
void beginHardware();
Inputs receiver();
bool readEncoder(uint16_t &raw);
bool sampleImu(Attitude &attitude, uint32_t now);
void driveAxle(int pwm);
void pollRadio(uint32_t now);
bool radioHealthy(uint32_t now);
bool radioReady(uint32_t now);
void transmit(const Outputs &, bool fault, uint32_t now);
float txRate();
} // namespace heli
