#pragma once

#include <Arduino.h>

//* ************************************************************************
//* ************************ CUTTING CYCLE HEADER ***************************
//* ************************************************************************
// This header provides the interface for the complete cutting cycle implementation
// including motor movements, clamp sequences, and all phase management

// Main cutting cycle function
void runCuttingCycle();

// Motor movement helper functions
bool executeProgressiveMovement(float targetPosition, float maxSpeed, float acceleration, unsigned long timeoutMs);
bool executeControlledMovement(float targetPosition, float speed, float acceleration, unsigned long timeoutMs);

// Clamp control functions
void executeClampSequence();
void releaseClamps();

// Utility functions
void updateTransferArmStartSignalDebouncer(); 