#ifndef CUTTING_CYCLE_FUNCTIONS_H
#define CUTTING_CYCLE_FUNCTIONS_H

#include "StateMachine/STATES/07_CUTTING_CYCLE.h"

//* ************************************************************************
//* ****************** CUTTING CYCLE FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to cutting cycle orchestration
// including state transitions, sequence management, and error handling

// Function declarations
void initializeCuttingCycleSequence();
void executeCurrentState();
void transitionToNextState();
void handleStateTransition();
void checkForEmptyClass();
void executeEmptyClassReturn();
void validateStateCompletion();
void handleCycleError();
void resetAllStates();

#endif // CUTTING_CYCLE_FUNCTIONS_H 