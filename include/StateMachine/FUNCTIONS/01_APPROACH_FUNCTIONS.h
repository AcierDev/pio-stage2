#ifndef APPROACH_FUNCTIONS_H
#define APPROACH_FUNCTIONS_H

#include <AccelStepper.h>
#include <Arduino.h>

//* ************************************************************************
//* ****************** APPROACH FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to approach operations
// including positioning, motion control, and sequence execution

// Forward declarations
extern AccelStepper stepper;

// Function declarations
void initializeApproachSequence();
void executeApproachMovement();
void moveToApproachPosition();
void waitForApproachComplete();
void validateApproachPosition();
void setupApproachParameters();

#endif // APPROACH_FUNCTIONS_H 