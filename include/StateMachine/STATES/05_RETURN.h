#ifndef RETURN_H
#define RETURN_H

#include <AccelStepper.h>
#include <Arduino.h>

//* ************************************************************************
//* ************************ RETURN ***************************
//* ************************************************************************
// This state handles the return sequence to bring the stepper back
// to the home position after completing the cutting operation

// Forward declarations
extern AccelStepper stepper;

// Function declarations
void executeReturnState();
void executeReturnMovement();
void executeEmptyClassReturn();
bool isReturnComplete();
void resetReturnState();

#endif // RETURN_H 