#include "StateMachine/FUNCTIONS/07_CUTTING_CYCLE_FUNCTIONS.h"
#include "StateMachine/STATES/02_CLAMP.h"
#include "StateMachine/STATES/03_ANALYSIS.h"
#include "StateMachine/STATES/00_HOMING.h"
#include "StateMachine/STATES/01_APPROACH.h"
#include "StateMachine/STATES/04_CUTTING.h"
#include "StateMachine/STATES/06_FINISH.h"
#include "StateMachine/STATES/05_RETURN.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"
#include "system_states.h"

//* ************************************************************************
//* ****************** CUTTING CYCLE FUNCTIONS ********************
//* ************************************************************************
// This module implements all functions related to cutting cycle orchestration
// including state transitions, sequence management, and error handling

void initializeCuttingCycleSequence() {
    //! Initialize cutting cycle sequence parameters
    currentCycleState = CYCLE_CLAMP;
    resetAllStates();
}

void executeCurrentState() {
    //! Execute the current state in the cutting cycle
    switch (currentCycleState) {
        case CYCLE_CLAMP:
            executeClampState();
            break;
        case CYCLE_ANALYSIS:
            executeAnalysisState();
            break;
        case CYCLE_HOMING:
            executeHomingState();
            break;
        case CYCLE_APPROACH:
            executeApproachState();
            break;
        case CYCLE_CUTTING:
            executeCuttingState();
            break;
        case CYCLE_FINISH:
            executeFinishState();
            break;
        case CYCLE_RETURN:
            executeReturnState();
            break;
        default:
            // Handle unexpected state
            handleCycleError();
            break;
    }
}

void transitionToNextState() {
    //! Transition to the next state in the sequence
    switch (currentCycleState) {
        case CYCLE_CLAMP:
            currentCycleState = CYCLE_ANALYSIS;
            break;
        case CYCLE_ANALYSIS:
            checkForEmptyClass();
            if (currentCycleState != CYCLE_COMPLETE) {
                currentCycleState = CYCLE_APPROACH;
            }
            break;
        case CYCLE_APPROACH:
            currentCycleState = CYCLE_CUTTING;
            break;
        case CYCLE_CUTTING:
            currentCycleState = CYCLE_FINISH;
            break;
        case CYCLE_FINISH:
            currentCycleState = CYCLE_RETURN;
            break;
        case CYCLE_RETURN:
            currentCycleState = CYCLE_COMPLETE;
            break;
        default:
            currentCycleState = CYCLE_ERROR;
            break;
    }
}

void handleStateTransition() {
    //! Handle any special state transition logic
    // Add any special transition handling here if needed
}

void checkForEmptyClass() {
    //! Check for empty class detection and handle accordingly
    if (analysisResultReceived && lastDetectedClass.equalsIgnoreCase("Empty")) {
        executeEmptyClassReturn();
        currentCycleState = CYCLE_COMPLETE;
    }
}

void executeEmptyClassReturn() {
    //! Execute return sequence for empty class detection
    // Retract clamps and return to home position
    digitalWrite(Pins::LEFT_CLAMP, HIGH);   // Retract left clamp
    digitalWrite(Pins::RIGHT_CLAMP, HIGH);  // Retract right clamp
    delay(200);
    digitalWrite(Pins::LEFT_CLAMP, LOW);    // Extend left clamp
    digitalWrite(Pins::RIGHT_CLAMP, LOW);   // Extend right clamp

    // Signal transfer arm to prevent Z-axis lowering during return
    digitalWrite(Pins::TRANSFER_ARM_SIGNAL, HIGH);

    // Return directly to home
    moveStepperToPosition(Motion::HOME_OFFSET, Motion::RETURN_SPEED, Motion::RETURN_ACCEL);
    
    // Deactivate transfer arm signal - return is complete
    digitalWrite(Pins::TRANSFER_ARM_SIGNAL, LOW);

    delay(50);
}

void validateStateCompletion() {
    //! Validate that the current state has completed successfully
    switch (currentCycleState) {
        case CYCLE_CLAMP:
            if (!isClampComplete()) {
                handleCycleError();
            }
            break;
        case CYCLE_ANALYSIS:
            if (!isAnalysisComplete()) {
                handleCycleError();
            }
            break;
        case CYCLE_HOMING:
            if (!isHomingComplete()) {
                handleCycleError();
            }
            break;
        case CYCLE_APPROACH:
            if (!isApproachComplete()) {
                handleCycleError();
            }
            break;
        case CYCLE_CUTTING:
            if (!isCuttingComplete()) {
                handleCycleError();
            }
            break;
        case CYCLE_FINISH:
            if (!isFinishComplete()) {
                handleCycleError();
            }
            break;
        case CYCLE_RETURN:
            if (!isReturnComplete()) {
                handleCycleError();
            }
            break;
        default:
            break;
    }
}

void handleCycleError() {
    //! Handle errors during cutting cycle
    currentCycleState = CYCLE_ERROR;
    resetAllStates();
}

void resetAllStates() {
    //! Reset all individual states to initial conditions
    resetClampState();
    resetAnalysisState();
    resetHomingState();
    resetApproachState();
    resetCuttingState();
    resetFinishState();
    resetReturnState();
} 