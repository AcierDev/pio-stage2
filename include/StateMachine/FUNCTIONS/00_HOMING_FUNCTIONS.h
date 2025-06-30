#pragma once

#include "system_states.h"

//* ************************************************************************
//* ****************** HOMING FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to homing operations
// including motion control, position establishment, and sequence execution

// Function declarations
void initializeHomingSequence();
void executeHomingMovement();
void moveToHomeOffset();
void waitForHomingComplete();
void validateHomingPosition();
void setupHomingParameters(); 