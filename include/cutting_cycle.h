#ifndef CUTTING_CYCLE_H
#define CUTTING_CYCLE_H

#include <AccelStepper.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Bounce2.h>

// Forward declarations for external dependencies
extern AccelStepper stepper;
extern Bounce cameraSignal;
extern String lastDetectedClass;
extern bool analysisResultReceived;
extern unsigned long inferenceStartTime;
extern bool inferenceTimingActive;

// Pin definitions (from main.cpp)
namespace Pins {
extern const int LEFT_CLAMP;
extern const int RIGHT_CLAMP;
extern const int ALIGN_CYLINDER;
extern const int TRANSFER_ARM_SIGNAL;
}

// Motion parameters (from main.cpp)
namespace Motion {
extern const int STEPS_PER_INCH;
extern const float HOME_OFFSET;
extern const float APPROACH_DISTANCE;
extern const float CUTTING_DISTANCE;
extern const float FORWARD_DISTANCE;
extern const float END_DROP_DISTANCE_OFFSET;
extern const float APPROACH_SPEED;
extern const float CUTTING_SPEED;
extern const float FINISH_SPEED;
extern const float RETURN_SPEED;
extern const float HOMING_SPEED;
extern const float FORWARD_ACCEL;
extern const float RETURN_ACCEL;
}

// Timing parameters (from main.cpp)
namespace Timing {
extern const int MOTION_SETTLE_TIME;
}

// Function declarations
void runCuttingCycle();
void moveStepperToPosition(float position, float speed, float acceleration);
void releaseClamps();
void sendBurstRequest();
void handleSerialResponse(const String &response);
void sendSerialMessage(const String &message);
void logMessage(const String &message, const String &level = "info", bool excludeSerial = false);
void updateTransferArmStartSignalDebouncer();

//* ************************************************************************
//* ****************** CUTTING CYCLE FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to the cutting cycle operations
// including state management, motion control, and sequence execution

namespace CuttingCycle {

//* ************************************************************************
//* ************************ STATE MANAGEMENT *************************
//* ************************************************************************
// Functions for managing cutting cycle states

void initializeCuttingCycle();
void resetCuttingCycle();
bool startCuttingCycle();
void stopCuttingCycle();

//* ************************************************************************
//* ************************ MOTION CONTROL ***************************
//* ************************************************************************
// Functions for controlling cutting motion sequences

void executeHomingSequence();
void executeApproachSequence();
void executeCuttingSequence();
void executeFinishSequence();
void executeReturnSequence();

//* ************************************************************************
//* ************************ CYCLE MANAGEMENT *************************
//* ************************************************************************
// Functions for complete cycle management

void runFullCuttingCycle();
void handleCycleError();
bool isCycleComplete();
void updateCycleStatus();

} // namespace CuttingCycle

#endif // CUTTING_CYCLE_H 