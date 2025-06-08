#ifndef SIGNAL_CONTROL_H
#define SIGNAL_CONTROL_H

#include <Arduino.h>

//* ************************************************************************
//* ****************** SIGNAL CONTROL FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to digital I/O signal control
// including transfer arm signals and other digital outputs

// Function declarations
void activateTransferArmSignal();
void deactivateTransferArmSignal();

#endif // SIGNAL_CONTROL_H 