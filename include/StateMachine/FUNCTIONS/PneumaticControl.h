#ifndef PNEUMATIC_CONTROL_H
#define PNEUMATIC_CONTROL_H

#include <Arduino.h>

//* ************************************************************************
//* ****************** PNEUMATIC CONTROL FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to pneumatic operations
// including clamp cylinders, alignment cylinder, and solenoid control

// Global variable declarations
extern bool clampPositionReached;
extern bool alignmentCylinderExtended;

// Function declarations
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

#endif // PNEUMATIC_CONTROL_H 