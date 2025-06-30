#pragma once

#include <FastAccelStepper.h>

//* ************************************************************************
//* ************************ CUTTING CYCLE FUNCTIONS ***************************
//* ************************************************************************

// Motor movement functions
bool executeProgressiveMovement(float targetPosition, float maxSpeed, float acceleration, unsigned long timeoutMs);
bool executeControlledMovement(float targetPosition, float speed, float acceleration, unsigned long timeoutMs);

// Clamp control functions
void executeClampSequence();
void releaseClamps();

// Utility functions
void updateTransferArmStartSignalDebouncer();

// Main cutting cycle function
void runCuttingCycle(); 