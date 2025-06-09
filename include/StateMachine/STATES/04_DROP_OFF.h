#ifndef DROP_OFF_H
#define DROP_OFF_H

#include <AccelStepper.h>
#include <Arduino.h>

//* ************************************************************************
//* ************************ DROP OFF ***************************
//* ************************************************************************
// This state handles the drop off sequence to complete the cutting operation
// and position the stepper at the forward position for piece drop off

// Forward declarations
extern AccelStepper stepper;
extern String lastDetectedClass;
extern bool analysisResultReceived;

// Function declarations
void executeDropOffState();
void executeDropOffMovement();
void handleEndClassDetection();
bool isDropOffComplete();
void resetDropOffState();

#endif // DROP_OFF_H 