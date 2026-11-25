#include <MorpheusHelicopter.h>

// CH6 ramps the governor, CH3 varies collective. Cyclic and axle motor are off.
void setup() {
  heli::begin(heli::Profile::RpmTuning);
}
void loop() {
  heli::service();
}
