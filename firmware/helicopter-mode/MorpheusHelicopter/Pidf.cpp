#include "Control.h"
namespace heli {
// Derivative on measurement avoids a kick on a setpoint change. The caller
// supplies the achievable output interval including this tick's slew limit.
float Pidf::update(const Gains &g, float error, float measuredSlope, float ff, float dt, float low,
                   float high) {
  if (dt <= 0)
    return output;
  slope += dt / (dt + g.derivativeFilterSeconds) * (measuredSlope - slope);
  p = g.kp * error;
  d = -g.kd * slope;
  f = ff;
  float candidate = limit(i + g.ki * error * dt, -g.integralLimit, g.integralLimit);
  // Allow the integral up to the available headroom and always allow unwinding.
  float base = p + d + f;
  if (candidate > i)
    i = fmaxf(i, fminf(candidate, high - base));
  if (candidate < i)
    i = fminf(i, fmaxf(candidate, low - base));
  float requested = base + i;
  output = limit(requested, low, high);
  limited = fabsf(output - requested) > 0.0001f;
  return output;
}
} // namespace heli
