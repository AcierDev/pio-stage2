#ifndef FINISH_FUNCTIONS_H
#define FINISH_FUNCTIONS_H

#include <AccelStepper.h>
#include <Arduino.h>

//* ************************************************************************
//* ****************** FINISH FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to finish operations
// including end drop handling, clamp control, and final positioning

// Forward declarations
extern AccelStepper stepper;
extern String lastDetectedClass;
extern bool analysisResultReceived;

// Function declarations
void initializeFinishSequence();
void executeFinishMovement();
void handleEndClassDetection();
void moveToForwardPosition();
void waitForFinishComplete();
void validateFinishPosition();
void setupFinishParameters();

#endif // FINISH_FUNCTIONS_H 