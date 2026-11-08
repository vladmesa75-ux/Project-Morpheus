/*
  Project Morpheus - emergency local DC motor stop

  Upload this with the DC motor power disconnected.
  Pins 4 and 5 are the two H-bridge PWM inputs.
  Both are repeatedly commanded to zero.
*/

const uint8_t MOTOR_PIN_A = 4;
const uint8_t MOTOR_PIN_B = 5;

void setup() {
  analogWriteResolution(8);

  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);

  analogWrite(MOTOR_PIN_A, 0);
  analogWrite(MOTOR_PIN_B, 0);
}

void loop() {
  analogWrite(MOTOR_PIN_A, 0);
  analogWrite(MOTOR_PIN_B, 0);
}