#pragma once

#include "system_states.h"

//* ************************************************************************
//* ************************ HOMING ***************************
//* ************************************************************************
// This state handles the homing sequence for the stepper motor,
// establishing the zero position and moving to the home offset

// Function declarations
void executeHomingState();
bool isHomingComplete();
void resetHomingState(); 