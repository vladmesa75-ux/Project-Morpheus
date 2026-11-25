#pragma once
#include <stdint.h>

// Columns in physical degrees: -20, -15, -10, -5, 0, +5, +10, +15, +20.
// Replace BOTH rows with your bench measurements. Zero means unmeasured.
const uint16_t leftPulseUs[9]  = {0, 0, 0, 0, 1430, 0, 0, 0, 0};
const uint16_t rightPulseUs[9] = {0, 0, 0, 0, 1470, 0, 0, 0, 0};
const int8_t commands[9] = {-127, -95, -64, -32, 0, 32, 64, 95, 127};

inline bool validCalibration(const uint16_t (&row)[9]) {
  bool ascending = row[8] > row[0];
  for (uint8_t i = 0; i < 9; ++i) {
    if (row[i] < 500 || row[i] > 2500) return false;
    if (i && (ascending ? row[i] <= row[i-1] : row[i] >= row[i-1])) return false;
  }
  return true;
}

inline uint16_t interpolate(int8_t command, const uint16_t (&row)[9]) {
  if (command <= commands[0]) return row[0];
  uint8_t i = 1;
  while (command > commands[i]) ++i;
  int span = commands[i] - commands[i-1];
  int offset = command - commands[i-1];
  // Weighted sum stays positive even for a reversed linkage.
  return ((int32_t)row[i-1] * (span-offset) + (int32_t)row[i] * offset + span/2) / span;
}
