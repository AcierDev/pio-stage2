#include "StateMachine/STATES/01_APPROACH.h"
#include "StateMachine/FUNCTIONS/01_APPROACH_FUNCTIONS.h"
#include "config/Config.h"

//* ************************************************************************
//* ************************ APPROACH ***************************
//* ************************************************************************
// This state handles the approach sequence to position the stepper
// at the cutting start position before beginning the cutting operation

void executeApproachState() {
    //! Execute complete approach sequence
    initializeApproachSequence();
    executeApproachMovement();
    validateApproachPosition();
}

bool isApproachComplete() {
    //! Check if approach sequence is complete
    return !stepper.isRunning();
}

void resetApproachState() {
    //! Reset approach state to initial conditions
    stepper.stop();
} 