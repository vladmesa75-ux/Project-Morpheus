#pragma once

#include <stdint.h>

namespace heli {

enum class RotorMode { Off, Manual, Governor };
enum class CollectiveMode { Stick, Fixed };
enum class YawMode { Off, Rate, Heading };
enum class Profile { Flight, RpmTuning, YawTuning };
enum class StartStick { Low, Center, Custom };
enum class View { Status, Rpm, Yaw, Cyclic, All };
struct Modes {
  RotorMode rotor;
  CollectiveMode collective;
  YawMode yaw;
  bool rollCyclic, pitchCyclic;
};
struct Gains {
  float kp, ki, kd, integralLimit, derivativeFilterSeconds;
};
struct Stick {
  uint8_t channel;
  uint16_t lowUs, centerUs, highUs, deadbandUs;
};
struct Travel {
  float neutral, negativeTravel, positiveTravel;
};
struct Surface {
  float minDeg, maxDeg, trimDeg, stoppedDeg, faultDeg;
  int signToNano;
};
struct Motor {
  float scale, offset;
};

namespace cfg {
// The main flight sketch and both tuning sketches compile this same file.
// Channel numbers below are the labels on the transmitter (1 through 6).
constexpr Modes flight = {RotorMode::Governor, CollectiveMode::Stick, YawMode::Heading, true, true};
constexpr Stick roll = {1, 1000, 1500, 2000, 8}, pitch = {2, 1000, 1500, 2000, 8};
constexpr Stick collective = {3, 2000, 1500, 1000, 8}, yaw = {4, 1000, 1500, 2000, 10};
constexpr uint8_t rotorSwitchChannel = 6;
constexpr bool switchHighRuns = true;
constexpr int switchLowUs = 1600, switchHighUs = 1750;
constexpr int throttleOffUs = 2000, throttleFullUs = 1000, throttleOffToleranceUs = 15;
constexpr StartStick governorStartStick = StartStick::Center;
constexpr int customStartUs = 1500, governorStartToleranceUs = 35;

// Angles refer to flaperon deflection. Endpoints are initial bench settings.
constexpr float collectiveLowDeg = 0, collectiveCenterDeg = 5, collectiveHighDeg = 10;
constexpr float fixedCollectiveDeg = 5;
constexpr Travel rollCyclic = {0, 5, 5}, pitchCyclic = {0, 5, 5};
// Last field is the existing helicopter sign. Use +1 if the Nano rows already
// define positive in the helicopter lift convention. Never reverse at both ends.
constexpr Surface leftSurface = {-20, 20, 0, 0, 0, -1}, rightSurface = {-20, 20, 0, 0, 0, -1};
constexpr float phaseOffsetDeg = 0, commandDelayMs = 0;
constexpr float cyclicEngageRpm = 40, cyclicDropoutRpm = 30;

// Rotor RPM PIDF. Commands and integral limits are in ESC byte units.
// F = target RPM * (ffPerRpm + ffPerRpmDegree * abs(collective - center)).
constexpr Gains rpmGains = {0, 0, 0, 40, 0.05f};
constexpr float rpmFfPerRpm = 0, rpmFfPerRpmDegree = 0;
constexpr float targetRpm = 50, spoolRpmPerSecond = 10;
constexpr uint32_t governorIntervalUs = 10000;
constexpr float governorEscMax = 64, manualEscMax = 255, escRisePerSecond = 25;
constexpr Motor leftEsc = {1, 0}, rightEsc = {1, 0};
constexpr float overspeedRpm = 75, minimumRunRpm = 20;
constexpr uint32_t spoolDeadlineMs = 12000, lowRpmTimeoutMs = 2000;

// Heading mode uses heading error, filtered gyro damping and a bounded moving
// target. Rate mode uses separate gains because its error has different units.
constexpr Gains headingGains = {0, 0, 0, 40, 0.025f}, rateGains = {0, 0, 0, 40, 0.025f};
constexpr float yawMaxRateDps = 30, yawMaxLeadDeg = 3, yawExpo = 0;
constexpr float yawFfBias = 0, yawFfPerRpm = 0, yawFfPerTargetDps = 0;
constexpr uint32_t yawIntervalUs = 5000;
constexpr int axleSign = 1, axleDeadband = 4;
constexpr float axleMaxPwm = 80, axleSlewPerSecond = 200;

// Right wingtip to aircraft right at boot; CCW viewed from above is positive.
// Change encoderSign if a hand turn toward the nose reads negative.
constexpr float encoderTurnsPerRotorTurn = 1.34375f, phaseZeroDeg = 0;
constexpr int encoderSign = 1;
constexpr uint32_t sampleUs = 5000, rpmWindowUs = 100000, encoderStaleUs = 35000;
constexpr float maxEncoderDps = 3500, encoderSlackDeg = 3;
constexpr float rpmAlpha = 0.25f, rpmZeroBand = 0.5f;
constexpr uint32_t imuStaleUs = 40000, controlGapUs = 50000;
constexpr bool calibrateGyro = true;
constexpr unsigned gyroSamples = 1000;
constexpr float yawGyroTrimDps = 0, yawDeadbandDps = 0.05f, attitudeTimeConstantS = 0.25f;
constexpr int yawGyroSign = 1;
// Adafruit MPU6050 range/bandwidth enum names; consumed by Hardware.cpp.
#define MORPHEUS_ACCEL_RANGE MPU6050_RANGE_8_G
#define MORPHEUS_GYRO_RANGE MPU6050_RANGE_500_DEG
#define MORPHEUS_IMU_FILTER MPU6050_BAND_94_HZ

constexpr uint8_t rcPins[6] = {48, 46, 44, 42, 40, 38};
constexpr int rcMinValidUs = 800, rcMaxValidUs = 2200;
constexpr uint32_t rcTimeoutUs = 100000, mechanismSettleMs = 3000;
constexpr uint8_t servoPins[6] = {39, 43, 45, 47, 49, 51};
constexpr int servoMinUs = 500, servoMaxUs = 2500;
// Same order as servoPins: tail locked, H-tail folded, wing released, spares.
constexpr int mechanismUs[6] = {1000, 1000, 900, 1500, 1500, 1500};
constexpr uint8_t axlePinA = 4, axlePinB = 5;
constexpr uint8_t imuAddress = 0x68, encoderAddress = 0x36;
constexpr uint32_t imuBusHz = 400000, encoderBusHz = 400000;
constexpr uint8_t radioCs = 10, radioReset = 9, radioDio = 2, radioSf = 7, radioCr = 5;
constexpr long radioHz = 433000000L, radioBandwidthHz = 500000L;
constexpr int radioDbm = 17;
constexpr uint32_t txSpacingUs = 0, txTimeoutUs = 250000; // 0: as soon as free

constexpr bool usbEnabled = true, openLogEnabled = true;
constexpr unsigned long usbBaud = 115200, openLogBaud = 115200;
constexpr uint32_t usbIntervalMs = 100, openLogIntervalMs = 50;
constexpr View usbView = View::Status, openLogView = View::All;
// Optional comma-separated exact field names, e.g. "ms,rpm,esc_l,axle_pwm".
// An empty string includes every field in the chosen view.
constexpr char usbFields[] = "", openLogFields[] = "";

static_assert(encoderSign * encoderSign == 1 && yawGyroSign * yawGyroSign == 1 &&
                  axleSign * axleSign == 1,
              "Signs must be +1 or -1");
static_assert(encoderTurnsPerRotorTurn > 0 && sampleUs > 0 && rpmWindowUs >= sampleUs && rpmAlpha > 0 &&
                  rpmAlpha <= 1,
              "Invalid rotor sampling");
static_assert(maxEncoderDps * encoderStaleUs / 1e6f + encoderSlackDeg < 180,
              "Encoder stale window can alias a half turn");
static_assert(yawMaxLeadDeg > 0 && yawMaxLeadDeg < 180 && yawExpo >= 0 && yawExpo <= 1,
              "Invalid yaw reference");
static_assert(cyclicEngageRpm >= cyclicDropoutRpm && cyclicDropoutRpm > 0,
              "Invalid cyclic RPM thresholds");
static_assert(governorIntervalUs > 0 && yawIntervalUs > 0 && controlGapUs > governorIntervalUs &&
                  controlGapUs > yawIntervalUs,
              "Invalid schedule");
static_assert(targetRpm > 0 && minimumRunRpm < targetRpm && targetRpm < overspeedRpm &&
                  spoolRpmPerSecond > 0,
              "Invalid governed RPM envelope");
static_assert(governorEscMax > 0 && governorEscMax <= 255 && axleMaxPwm > 0 && axleMaxPwm <= 255 &&
                  escRisePerSecond > 0 && axleSlewPerSecond > 0,
              "Invalid actuator envelope");
static_assert(switchLowUs < switchHighUs && throttleOffUs != throttleFullUs && gyroSamples > 0,
              "Invalid hardware settings");
static_assert(flight.rotor != RotorMode::Manual || flight.collective == CollectiveMode::Fixed,
              "CH3 manual thrust requires fixed collective");
static_assert(manualEscMax > 0 && manualEscMax <= 255 && axleDeadband >= 0 &&
                  axleDeadband <= axleMaxPwm,
              "Invalid manual output limits");
static_assert(rpmGains.integralLimit >= 0 && headingGains.integralLimit >= 0 &&
                  rateGains.integralLimit >= 0 && rpmGains.derivativeFilterSeconds >= 0 &&
                  headingGains.derivativeFilterSeconds >= 0 && rateGains.derivativeFilterSeconds >= 0 &&
                  attitudeTimeConstantS > 0,
              "Invalid controller filters or limits");
constexpr bool validStick(Stick s) {
  return s.channel >= 1 && s.channel <= 6 &&
         ((s.lowUs + s.deadbandUs < s.centerUs && s.centerUs + s.deadbandUs < s.highUs) ||
          (s.highUs + s.deadbandUs < s.centerUs && s.centerUs + s.deadbandUs < s.lowUs));
}
constexpr bool validSurface(Surface s) {
  return s.minDeg >= -20 && s.maxDeg <= 20 && s.minDeg < s.maxDeg && s.signToNano * s.signToNano == 1 &&
         s.stoppedDeg >= s.minDeg && s.stoppedDeg <= s.maxDeg && s.faultDeg >= s.minDeg &&
         s.faultDeg <= s.maxDeg;
}
static_assert(validStick(roll) && validStick(pitch) && validStick(collective) && validStick(yaw) &&
                  rotorSwitchChannel >= 1 && rotorSwitchChannel <= 6,
              "Invalid RC calibration");
static_assert(validSurface(leftSurface) && validSurface(rightSurface),
              "Surface limits must fit the Nano calibration");
} // namespace cfg
} // namespace heli
