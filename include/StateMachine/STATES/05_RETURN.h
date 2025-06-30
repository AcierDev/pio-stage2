#pragma once

#include "system_states.h"

//* ************************************************************************
//* ************************ RETURN ***************************
//* ************************************************************************
// This state handles the return sequence for the stepper motor,
// moving back to the home position after cutting

// Function declarations
void executeReturnState();
bool isReturnComplete();
void resetReturnState(); 