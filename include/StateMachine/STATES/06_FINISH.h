#pragma once

#include "system_states.h"

//* ************************************************************************
//* ************************ FINISH ***************************
//* ************************************************************************
// This state handles the finish sequence for the stepper motor,
// moving to the final forward position

// Function declarations
void executeFinishState();
bool isFinishComplete();
void resetFinishState(); 