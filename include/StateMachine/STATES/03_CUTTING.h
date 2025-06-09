#ifndef CUTTING_H
#define CUTTING_H

#include <FastAccelStepper.h>
#include <Arduino.h>

//* ************************************************************************
//* ************************ CUTTING ***************************
//* ************************************************************************
// This state handles the cutting sequence where the stepper moves
// slowly through the material to perform the actual cutting operation

// Forward declarations
extern FastAccelStepper* stepper;

// Function declarations
void executeCuttingState();
void executeCuttingMovement();
bool isCuttingComplete();
void resetCuttingState();

#endif // CUTTING_H 