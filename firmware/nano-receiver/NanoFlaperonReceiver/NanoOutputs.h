#pragma once
#include <Arduino.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error Select a classic 16 MHz ATmega328P Arduino Nano.
#endif

// Timer1 free-runs at 2 ticks/us. Compare A ends the two flaperon pulses.
// Compare B sends the spare servos and ESCs in a separate 20 ms frame.
namespace outputs {
const uint8_t leftMask = _BV(PD4), rightMask = _BV(PD6);
volatile uint8_t firstMask, lastMask;
volatile uint16_t lastEdge;
volatile bool busy;
volatile uint16_t escUs[2] = {1000, 1000};
uint32_t lastStartUs;
uint16_t leftUs = 1430, rightUs = 1470;
bool pending;

void begin() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    PORTD &= ~(_BV(PD3) | leftMask | _BV(PD5) | rightMask | _BV(PD7));
    PORTB &= ~_BV(PB0);
    DDRD |= _BV(PD3) | leftMask | _BV(PD5) | rightMask | _BV(PD7);
    DDRB |= _BV(PB0);
    TCCR1A = TCCR1B = 0;
    TIMSK1 = 0;
    TCNT1 = 0;
    OCR1B = 100;
    TIFR1 = _BV(OCF1A) | _BV(OCF1B);
    TIMSK1 = _BV(OCIE1B);
    TCCR1B = _BV(CS11);
  }
  lastStartUs = micros() - 20000UL;
  pending = true;
}

void setFlaps(uint16_t left, uint16_t right) {
  leftUs = left;
  rightUs = right;
  pending = true;
}

void setMotors(uint16_t left, uint16_t right) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    escUs[0] = left;
    escUs[1] = right;
  }
}

void service() {
  uint32_t now = micros();
  uint32_t elapsed = now - lastStartUs;
  // At the expected ~80 Hz this starts a frame on every packet. Keep the
  // latest command if a burst arrives inside the 5 ms minimum frame period.
  if (busy || elapsed < 5000UL || (!pending && elapsed < 20000UL)) return;
  bool leftFirst = leftUs <= rightUs;
  uint16_t firstTicks = 2 * min(leftUs, rightUs);
  uint16_t lastTicks = 2 * max(leftUs, rightUs);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    firstMask = leftFirst ? leftMask : rightMask;
    lastMask = leftFirst ? rightMask : leftMask;
    if (firstTicks == lastTicks) {
      firstMask |= lastMask;
      lastMask = 0;
    }
    busy = true;
    PORTD |= leftMask | rightMask;
    uint16_t start = TCNT1;
    OCR1A = start + firstTicks;
    lastEdge = start + lastTicks;
    TIFR1 = _BV(OCF1A);
    TIMSK1 |= _BV(OCIE1A);
  }
  pending = false;
  lastStartUs = now;
}
}

ISR(TIMER1_COMPA_vect) {
  using namespace outputs;
  PORTD &= ~firstMask;
  if (lastMask) {
    firstMask = lastMask;
    lastMask = 0;
    // With adjacent widths, the second deadline can pass inside this ISR.
    // Only arm a new interrupt when there is enough time to program it.
    if ((int16_t)(lastEdge - TCNT1) > 24) {
      OCR1A = lastEdge;
      return;
    }
    while ((int16_t)(lastEdge - TCNT1) > 0) {}
    PORTD &= ~firstMask;
  }
  TIMSK1 &= ~_BV(OCIE1A);
  busy = false;
}

ISR(TIMER1_COMPB_vect) {
  static uint8_t phase = 0;
  static uint16_t frameStart, motorLeft, motorRight;
  uint16_t width = 1500;
  switch (phase) {
    case 0:
      frameStart = TCNT1;
      motorLeft = outputs::escUs[0];
      motorRight = outputs::escUs[1];
      PORTD |= _BV(PD3);
      break;
    case 1:
      PORTD &= ~_BV(PD3);
      PORTD |= _BV(PD5);
      break;
    case 2:
      PORTD &= ~_BV(PD5);
      PORTB |= _BV(PB0); // D8
      width = motorLeft;
      break;
    case 3:
      PORTB &= ~_BV(PB0);
      PORTD |= _BV(PD7);
      width = motorRight;
      break;
    default:
      PORTD &= ~_BV(PD7);
      OCR1B = frameStart + 40000U;
      phase = 0;
      return;
  }
  OCR1B = TCNT1 + 2 * width;
  ++phase;
}
