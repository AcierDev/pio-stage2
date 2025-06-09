#ifndef INITIALIZING_STATE_H
#define INITIALIZING_STATE_H

#include <Arduino.h>

//* ************************************************************************
//* ************************ INITIALIZING ***************************
//* ************************************************************************
// This state handles the initial hardware setup and system initialization
// before proceeding to homing and operations

// Function declarations
void executeInitializingState();
void initializeHardware();
bool isInitializationComplete();
void resetInitializingState();

#endif // INITIALIZING_STATE_H 