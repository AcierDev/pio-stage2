#include "StateMachine/STATES/02_APPROACH.h"
#include "system_states.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ APPROACH ***************************
//* ************************************************************************
// This state handles the approach sequence to position the stepper
// at the cutting start position before beginning the cutting operation

void executeApproachState() {
    //! Execute complete approach sequence
    executeApproachMovement();
}

void executeApproachMovement() {
    logMessage("🚀 Approach phase...");
    stepper.setMaxSpeed(Motion::APPROACH_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
    stepper.moveTo(Motion::APPROACH_DISTANCE * Motion::STEPS_PER_INCH);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    logMessage("✅ Approach phase complete");
}

bool isApproachComplete() {
    //! Check if approach sequence is complete
    return !stepper.isRunning();
}

void resetApproachState() {
    //! Reset approach state to initial conditions
    stepper.stop();
} 