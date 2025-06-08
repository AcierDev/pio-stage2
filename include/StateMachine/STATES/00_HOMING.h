#ifndef HOMING_H
#define HOMING_H

#include <AccelStepper.h>
#include <Arduino.h>

//* ************************************************************************
//* ************************ HOMING ***************************
//* ************************************************************************
// This state handles the homing sequence to establish reference position
// for the stepper motor before starting cutting operations

// Forward declarations
extern AccelStepper stepper;

// Function declarations
void executeHomingState();
bool isHomingComplete();
void resetHomingState();

#endif // HOMING_H 