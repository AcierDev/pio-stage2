#ifndef CLAMP_FUNCTIONS_H
#define CLAMP_FUNCTIONS_H

#include <Arduino.h>

//* ************************************************************************
//* ****************** CLAMP FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to clamp operations
// including extend, retract, and alignment cylinder control

// Global variable declarations
extern bool clampPositionReached;
extern bool alignmentCylinderExtended;

// Function declarations
void initializeClampSequence();
void executeClampSequence();
void extendLeftClamp();
void retractLeftClamp();
void extendRightClamp();
void retractRightClamp();
void extendClamps();
void retractClamps();
void extendAlignmentCylinder();
void retractAlignmentCylinder();
void executeAlignmentSequence();
void waitForClampPosition();
void validateClampState();

#endif // CLAMP_FUNCTIONS_H 