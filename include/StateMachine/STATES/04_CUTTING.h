#pragma once

#include "system_states.h"

//* ************************************************************************
//* ************************ CUTTING ***************************
//* ************************************************************************
// This state handles the cutting sequence for the stepper motor,
// moving through the cutting distance at slow speed

// Function declarations
void executeCuttingState();
bool isCuttingComplete();
void resetCuttingState(); 