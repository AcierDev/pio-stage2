#include "StateMachine/STATES/07_CUTTING_CYCLE.h"
#include "StateMachine/FUNCTIONS/07_CUTTING_CYCLE_FUNCTIONS.h"
#include "StateMachine/FUNCTIONS/SequenceManagement.h"
#include "StateMachine/FUNCTIONS/MotionControl.h"
#include "StateMachine/FUNCTIONS/PneumaticControl.h"
#include "StateMachine/FUNCTIONS/SignalControl.h"
#include "config/Config.h"

//* ************************************************************************
//* ************************ CUTTING CYCLE ***************************
//* ************************************************************************
// This state orchestrates the complete cutting cycle by managing
// all individual states in the proper sequence

// Global state variable
CuttingCycleState currentCycleState = CYCLE_IDLE;

void executeCuttingCycle() {
    //! Execute the complete cutting cycle state machine
    initializeCuttingCycleSequence();
    
    while (!isCuttingCycleComplete()) {
        executeCurrentState();
        validateStateCompletion();
        transitionToNextState();
        updateCuttingCycleState();
    }
}

void initializeCuttingCycle() {
    //! Initialize cutting cycle to starting state
    currentCycleState = CYCLE_CLAMP;
    resetAllStates();
}

void updateCuttingCycleState() {
    //! Update cutting cycle state based on current conditions
    handleStateTransition();
}

bool isCuttingCycleComplete() {
    //! Check if cutting cycle is complete
    return (currentCycleState == CYCLE_COMPLETE || currentCycleState == CYCLE_ERROR);
}

void resetCuttingCycle() {
    //! Reset cutting cycle to initial state
    currentCycleState = CYCLE_IDLE;
    resetAllStates();
}

void handleEmptyClassDetection() {
    //! Handle empty class detection by executing early return
    if (analysisResultReceived && lastDetectedClass.equalsIgnoreCase("Empty")) {
        executeEmptyClassReturn();
        currentCycleState = CYCLE_COMPLETE;
    }
} 