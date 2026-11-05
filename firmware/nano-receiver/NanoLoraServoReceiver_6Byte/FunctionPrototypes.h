#ifndef FUNCTION_PROTOTYPES_H
#define FUNCTION_PROTOTYPES_H

// ============================================================
// FunctionPrototypes.h
//
// These are simple declarations that tell Arduino which functions
// exist in the other tabs.
//
// You normally do not need to edit this file.
// ============================================================

uint8_t calculateChecksum(const uint8_t *packet);
int16_t readInt16LittleEndian(uint8_t lowByte, uint8_t highByte);
bool tryReceiveOnePacket();
void readPacketData(const uint8_t *packet);

void setupOutputs();
void applyFailsafe();
void applyOutputs();
int mapSignedToServoUs(int8_t value, uint8_t servoOutputIndex);
int mapUnsignedToEscUs(uint8_t value, uint8_t escOutputIndex);

bool setupLoraRadio();
void printOutputHeader();
void printOutputLine();

#endif