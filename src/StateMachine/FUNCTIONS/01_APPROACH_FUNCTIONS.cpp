#include "system_states.h"
#include "StateMachine/FUNCTIONS/01_APPROACH_FUNCTIONS.h"
#include "config/Config.h"

//* ************************************************************************
//* ****************** APPROACH FUNCTIONS ********************
//* ************************************************************************
// This module implements all functions related to approach operations
// including positioning, motion control, and sequence execution

void initializeApproachSequence() {
    //! Initialize approach sequence parameters
    setupApproachParameters();
}

void executeApproachMovement() {
    //! Execute the approach movement to cutting position
    moveToApproachPosition();
    waitForApproachComplete();
}

void moveToApproachPosition() {
    //! Move to the approach position
    moveStepperToPosition(Motion::APPROACH_DISTANCE, Motion::APPROACH_SPEED, Motion::FORWARD_ACCEL);
}

void waitForApproachComplete() {
    //! Wait for approach movement to complete
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    delay(Timing::MOTION_SETTLE_TIME);
}

void validateApproachPosition() {
    //! Validate that approach was successful
    // Add any position validation logic here if needed
}

void setupApproachParameters() {
    //! Setup parameters for approach sequence
    stepper.setMaxSpeed(Motion::APPROACH_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
} 