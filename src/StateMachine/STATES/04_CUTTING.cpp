#include "StateMachine/STATES/04_CUTTING.h"
#include "StateMachine/FUNCTIONS/SequenceManagement.h"
#include "StateMachine/FUNCTIONS/MotionControl.h"
#include "config/Config.h"

//* ************************************************************************
//* ************************ CUTTING ***************************
//* ************************************************************************
// This state handles the cutting sequence where the stepper moves
// slowly through the material to perform the actual cutting operation

void executeCuttingState() {
    //! Execute complete cutting sequence
    initializeCuttingSequence();
    executeCuttingMovement();
    validateCuttingPosition();
}

bool isCuttingComplete() {
    //! Check if cutting sequence is complete
    return !stepper.isRunning();
}

void resetCuttingState() {
    //! Reset cutting state to initial conditions
    stepper.stop();
} 