#ifndef RETURN_FUNCTIONS_H
#define RETURN_FUNCTIONS_H

#include <AccelStepper.h>
#include <Arduino.h>

//* ************************************************************************
//* ****************** RETURN FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to return operations
// including fast return, slow approach, and final positioning

// Forward declarations
extern AccelStepper stepper;

// Function declarations
void initializeReturnSequence();
void executeReturnMovement();
void activateTransferArmSignal();
void executeFastReturn();
void executeSlowApproach();
void deactivateTransferArmSignal();
void setupReturnParameters();

#endif // RETURN_FUNCTIONS_H 