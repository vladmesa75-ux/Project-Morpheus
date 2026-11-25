#include <MorpheusHelicopter.h>

// CH3 drives both ESCs directly. Collective is fixed and cyclic is off.
void setup() {
  heli::begin(heli::Profile::YawTuning);
}
void loop() {
  heli::service();
}
