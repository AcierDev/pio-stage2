#ifndef CUTTING_H
#define CUTTING_H

#include <AccelStepper.h>
#include <Arduino.h>

//* ************************************************************************
//* ************************ CUTTING ***************************
//* ************************************************************************
// This state handles the cutting sequence where the stepper moves
// slowly through the material to perform the actual cutting operation

// Forward declarations
extern AccelStepper stepper;

// Function declarations
void executeCuttingState();
bool isCuttingComplete();
void resetCuttingState();

#endif // CUTTING_H 