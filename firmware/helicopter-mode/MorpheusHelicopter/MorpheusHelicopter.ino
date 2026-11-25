#include "MorpheusHelicopter.h"

void setup() {
  heli::begin(heli::Profile::Flight);
}
void loop() {
  heli::service();
}
