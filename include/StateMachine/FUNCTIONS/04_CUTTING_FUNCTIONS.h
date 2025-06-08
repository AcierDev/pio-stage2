#ifndef CUTTING_FUNCTIONS_H
#define CUTTING_FUNCTIONS_H

#include <AccelStepper.h>
#include <Arduino.h>

//* ************************************************************************
//* ****************** CUTTING FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to cutting operations
// including motion control, cutting execution, and sequence management

// Forward declarations
extern AccelStepper stepper;

// Function declarations
void initializeCuttingSequence();
void executeCuttingMovement();
void moveThroughCuttingDistance();
void waitForCuttingComplete();
void validateCuttingPosition();
void setupCuttingParameters();

#endif // CUTTING_FUNCTIONS_H 