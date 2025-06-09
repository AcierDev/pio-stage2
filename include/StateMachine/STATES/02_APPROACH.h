#ifndef APPROACH_H
#define APPROACH_H

#include <FastAccelStepper.h>
#include <Arduino.h>

//* ************************************************************************
//* ************************ APPROACH ***************************
//* ************************************************************************
// This state handles the approach sequence to position the stepper
// at the cutting start position before beginning the cutting operation

// Forward declarations
extern FastAccelStepper* stepper;

// Function declarations
void executeApproachState();
void executeApproachMovement();
bool isApproachComplete();
void resetApproachState();

#endif // APPROACH_H 