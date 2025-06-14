#include "StateMachine/STATES/05_RETURN.h"
#include "StateMachine/FUNCTIONS/SequenceManagement.h"
#include "StateMachine/FUNCTIONS/MotionControl.h"
#include "StateMachine/FUNCTIONS/PneumaticControl.h"
#include "StateMachine/FUNCTIONS/SignalControl.h"
#include "StateMachine/FUNCTIONS/00_HOMING_FUNCTIONS.h"
#include "config/Config.h"

//* ************************************************************************
//* ************************ RETURN ***************************
//* ************************************************************************
// This state handles the return sequence to bring the stepper back
// to the home position after completing the cutting operation

void executeReturnState() {
    //! Execute complete return sequence
    initializeReturnSequence();
    retractClamps();
    activateTransferArmSignal();
    executeReturnMovement();
    moveToHomeOffset();
    deactivateTransferArmSignal();
}

bool isReturnComplete() {
    //! Check if return sequence is complete
    return !stepper.isRunning();
}

void resetReturnState() {
    //! Reset return state to initial conditions
    stepper.stop();
} 