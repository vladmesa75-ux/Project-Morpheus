#ifndef FUNCTION_PROTOTYPES_H
#define FUNCTION_PROTOTYPES_H

/*
  FunctionPrototypes.h

  This tab lists the functions that exist in the other tabs.
  It helps Arduino know about the functions before they are used.
*/

void setupRcInputs();
void handleRcInterrupt(uint8_t index);
void readRc0();
void readRc1();
void readRc2();
void readRc3();
void readRc4();
void readRc5();
void updateRcFrame();

void setupServos();
bool setServoUs(uint8_t servoIndex, int16_t pulseUs);
void centerAllServos();

void setupMotor();
void setMotorPower(int16_t power);

bool setupImu();
bool calibrateGyroBias();
bool readImu();
void updateAttitudeFromImu();
void wrapAngle180(float &angle);

bool setupAs5600();
bool readAs5600();

bool setupLoRaRadio();
void onLoRaTxDone();
void serviceLoRaTxTimeout();
bool sendLoRaPacket(const uint8_t packet[LORA_PACKET_BYTES]);
void buildLoRaPacket(uint8_t packet[LORA_PACKET_BYTES]);
uint8_t xorChecksum(const uint8_t *buffer, uint8_t length);
void writeInt16LittleEndian(uint8_t *buffer, uint8_t startIndex, int16_t value);
uint8_t rcUsToByte(uint16_t pulseUs);
int16_t angleToTenthsDegInt16(float angleDeg);
int8_t constrainSignedByte(int16_t value);
uint8_t constrainUnsignedByte(int16_t value);

void initializeFlightCommand();
void userFlightLogic();
void applyReceiverFailsafe();
int16_t rcPulseToSignedCommand(uint16_t pulseUs, uint16_t minUs, uint16_t centerUs, uint16_t maxUs, int8_t direction);
uint8_t rcThrottleToUnsignedCommand(uint16_t pulseUs);
int16_t ch5ToFlapCommand(uint16_t ch5Us);
uint8_t ch5ToPositionNumber(uint16_t ch5Us);
int16_t applyMotorScaleAndTrim(int16_t value, int16_t scalePercent, int16_t offset);
int16_t applyPercent(int16_t value, int16_t percent);
int16_t limitSignedCommand(int32_t value);
int16_t constrainInt16(int32_t value, int16_t low, int16_t high);
void applyFlightCommand();

void updateLoopTiming();
void writeOpenLogHeader();
void logOpenLogIfReady();
void printUsbDebugIfReady();

#endif