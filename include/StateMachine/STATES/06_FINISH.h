#ifndef FINISH_H
#define FINISH_H

#include <AccelStepper.h>
#include <Arduino.h>

//* ************************************************************************
//* ************************ FINISH ***************************
//* ************************************************************************
// This state handles the finish sequence to complete the cutting operation
// and position the stepper at the forward position

// Forward declarations
extern AccelStepper stepper;
extern String lastDetectedClass;
extern bool analysisResultReceived;

// Function declarations
void executeFinishState();
bool isFinishComplete();
void resetFinishState();

#endif // FINISH_H 