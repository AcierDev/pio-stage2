#pragma once

#include "system_states.h"

//* ************************************************************************
//* ************************ APPROACH ***************************
//* ************************************************************************
// This state handles the approach sequence for the stepper motor,
// moving to the approach position before cutting

// Function declarations
void executeApproachState();
bool isApproachComplete();
void resetApproachState(); 