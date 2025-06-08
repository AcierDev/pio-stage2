#include "StateMachine/STATES/00_HOMING.h"
#include "StateMachine/FUNCTIONS/00_HOMING_FUNCTIONS.h"
#include "config/Config.h"

//* ************************************************************************
//* ************************ HOMING ***************************
//* ************************************************************************
// This state handles the homing sequence to establish reference position
// for the stepper motor before starting cutting operations

void executeHomingState() {
    //! Execute complete homing sequence
    initializeHomingSequence();
    executeHomingMovement();
    moveToHomeOffset();
    validateHomingPosition();
}

bool isHomingComplete() {
    //! Check if homing sequence is complete
    return !stepper.isRunning();
}

void resetHomingState() {
    //! Reset homing state to initial conditions
    stepper.stop();
    stepper.setCurrentPosition(0);
} 