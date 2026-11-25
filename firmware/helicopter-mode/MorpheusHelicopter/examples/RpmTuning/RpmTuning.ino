#include "MorpheusHelicopter.h"

// Arduino Due programming port, Serial Monitor, 115200 baud, newline.
// Type HELP for commands, STATUS for inputs, or STREAM OFF to pause the display.
// CH6 starts/stops the shared governor. CH3 controls collective. Yaw and cyclic
// stay off. Serial gain edits are temporary; EXPORT prints Config.h declarations.
void setup() {
  heli::begin(heli::Profile::RpmTuning);
}

void loop() {
  heli::service();
}
