/*
  DueFlightComputerBase_v2.ino

  This is the main tab of the Arduino Due flight computer.

  The job of this file is to connect all the smaller tabs together:
  - start the hardware in setup()
  - run the repeated flight-computer tasks in loop()

  The real hardware code is split into the other tabs, so this file stays easy to read.
*/

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "Config.h"
#include "TypesAndGlobals.h"
#include "FunctionPrototypes.h"

void setup() {
  // USB serial is for debug messages to the computer.
  Serial.begin(USB_BAUD);

  // Serial2 goes to the OpenLog module.
  Serial2.begin(OPENLOG_BAUD);

  // I2C0 / Wire is used by the MPU-6050.
  Wire.begin();
  Wire.setClock(I2C0_SPEED_HZ);

  // I2C1 / Wire1 is used by the AS5600 encoder.
  Wire1.begin();
  Wire1.setClock(I2C1_SPEED_HZ);

  // Use 8-bit PWM values for the motor driver: 0 to 255.
  analogWriteResolution(8);

  // Start outputs in a safe state.
  setupMotor();
  setupServos();
  centerAllServos();
  initializeFlightCommand();

  // Start receiver interrupt reading.
  setupRcInputs();

  // Start sensors and radio.
  imuOk = setupImu();

  if (imuOk && ENABLE_GYRO_CALIBRATION) {
    // The aircraft must be still during this startup calibration.
    gyroCalibrated = calibrateGyroBias();
  }

  encoderOk = setupAs5600();
  loraOk = setupLoRaRadio();

  // Start timing from now.
  previousLoopStartUs = micros();
  lastImuUpdateUs = previousLoopStartUs;
  lastOpenLogUs = previousLoopStartUs;
  lastUsbDebugUs = previousLoopStartUs;

  // Write the CSV header once to the SD card through OpenLog.
  if (ENABLE_OPENLOG) {
    writeOpenLogHeader();
  }

  if (ENABLE_USB_DEBUG) {
    Serial.println("Due flight computer ready");
    Serial.print("IMU: ");
    Serial.println(imuOk ? "OK" : "FAILED");
    Serial.print("Gyro calibration: ");
    Serial.println(gyroCalibrated ? "OK" : "OFF/FAILED");
    Serial.print("AS5600: ");
    Serial.println(encoderOk ? "OK" : "FAILED");
    Serial.print("LoRa: ");
    Serial.println(loraOk ? "OK" : "FAILED");
  }
}

void loop() {
  static uint32_t lastPrintMs = 0;

  static uint32_t loopCount = 0;
  static uint32_t minLoopUs = 999999;
  static uint32_t maxLoopUs = 0;
  static uint32_t totalLoopUs = 0;

  static uint32_t maxOpenLogUs = 0;
  static uint32_t totalOpenLogUs = 0;
  static uint32_t openLogCount = 0;

  uint32_t loopStartUs = micros();

  updateLoopTiming();

  updateRcFrame();

  if (imuOk) {
    lastImuReadOk = readImu();
  }

  lastEncoderReadOk = readAs5600();

  if (lastEncoderReadOk) {
    encoderOk = true;
  }

  userFlightLogic();
  applyFlightCommand();

  serviceLoRaTxTimeout();

  if (loraOk && !loraTxBusy) {
    buildLoRaPacket(telemetryPacket);

    if (sendLoRaPacket(telemetryPacket)) {
      packetsSentThisSecond++;
    }
  }

  // Measure OpenLog time
  if (ENABLE_OPENLOG) {
    uint32_t openLogStartUs = micros();

    logOpenLogIfReady();

    uint32_t openLogTimeUs = micros() - openLogStartUs;

    totalOpenLogUs += openLogTimeUs;
    openLogCount++;

    if (openLogTimeUs > maxOpenLogUs) {
      maxOpenLogUs = openLogTimeUs;
    }
  }

  uint32_t loopTimeThisRunUs = micros() - loopStartUs;

  loopCount++;
  totalLoopUs += loopTimeThisRunUs;

  if (loopTimeThisRunUs < minLoopUs) {
    minLoopUs = loopTimeThisRunUs;
  }

  if (loopTimeThisRunUs > maxLoopUs) {
    maxLoopUs = loopTimeThisRunUs;
  }

  uint32_t nowMs = millis();

  if (nowMs - lastPrintMs >= 1000) {
    lastPrintMs = nowMs;

    float avgLoopUs = (float)totalLoopUs / loopCount;
    float avgLoopHz = 1000000.0f / avgLoopUs;

    float avgOpenLogUs = 0.0f;

    if (openLogCount > 0) {
      avgOpenLogUs = (float)totalOpenLogUs / openLogCount;
    }

    Serial.print("Loop_Hz:");
    Serial.print(avgLoopHz, 1);
    Serial.print('\t');

    Serial.print("Avg_loop_us:");
    Serial.print(avgLoopUs, 1);
    Serial.print('\t');

    Serial.print("Min_loop_us:");
    Serial.print(minLoopUs);
    Serial.print('\t');

    Serial.print("Max_loop_us:");
    Serial.print(maxLoopUs);
    Serial.print('\t');

    Serial.print("Avg_OpenLog_us:");
    Serial.print(avgOpenLogUs, 1);
    Serial.print('\t');

    Serial.print("Max_OpenLog_us:");
    Serial.print(maxOpenLogUs);
    Serial.print('\t');

    Serial.print("LoRa_sent:");
    Serial.println(packetsSentThisSecond);

    packetsSentThisSecond = 0;

    loopCount = 0;
    minLoopUs = 999999;
    maxLoopUs = 0;
    totalLoopUs = 0;

    maxOpenLogUs = 0;
    totalOpenLogUs = 0;
    openLogCount = 0;
  }
}