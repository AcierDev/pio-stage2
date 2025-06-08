#ifndef SEQUENCE_MANAGEMENT_H
#define SEQUENCE_MANAGEMENT_H

#include <Arduino.h>

//* ************************************************************************
//* ****************** SEQUENCE MANAGEMENT FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to sequence management
// including initialization, validation, and control operations

// Function declarations
void initializeApproachSequence();
void initializeCuttingSequence();
void initializeReturnSequence();
void initializeClampSequence();
void executeClampSequence();
void executeApproachMovement();
void executeCuttingMovement();
void executeReturnMovement();
void validateApproachPosition();
void validateCuttingPosition();
void validateClampState();
void initializeFinishSequence();
void executeFinishMovement();
void handleEndClassDetection();
void validateFinishPosition();

#endif // SEQUENCE_MANAGEMENT_H 