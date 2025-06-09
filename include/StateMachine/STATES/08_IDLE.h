#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include <Arduino.h>

//* ************************************************************************
//* ************************ IDLE ***************************
//* ************************************************************************
// This state represents the idle/waiting state where the machine waits
// for the start signal to begin a cutting cycle

// Function declarations
void executeIdleState();
bool isStartSignalReceived();
bool isIdleComplete();
void resetIdleState();

#endif // IDLE_STATE_H 