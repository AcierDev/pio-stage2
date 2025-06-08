#ifndef HOMING_FUNCTIONS_H
#define HOMING_FUNCTIONS_H

#include <AccelStepper.h>
#include <Arduino.h>

//* ************************************************************************
//* ****************** HOMING FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to homing operations
// including motion control, position establishment, and sequence execution

// Forward declarations
extern AccelStepper stepper;

// Function declarations
void initializeHomingSequence();
void executeHomingMovement();
void moveToHomeOffset();
void waitForHomingComplete();
void validateHomingPosition();
void setupHomingParameters();

#endif // HOMING_FUNCTIONS_H 