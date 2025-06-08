#include "StateMachine/STATES/06_FINISH.h"
#include "StateMachine/FUNCTIONS/SequenceManagement.h"
#include "StateMachine/FUNCTIONS/MotionControl.h"
#include "config/Config.h"

//* ************************************************************************
//* ************************ FINISH ***************************
//* ************************************************************************
// This state handles the finish sequence to complete the cutting operation
// and position the stepper at the forward position

void executeFinishState() {
    //! Execute complete finish sequence
    initializeFinishSequence();
    handleEndClassDetection();
    executeFinishMovement();
    validateFinishPosition();
}

bool isFinishComplete() {
    //! Check if finish sequence is complete
    return !stepper.isRunning();
}

void resetFinishState() {
    //! Reset finish state to initial conditions
    stepper.stop();
} 