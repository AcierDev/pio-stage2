#include "system_states.h"
#include "StateMachine/FUNCTIONS/00_HOMING_FUNCTIONS.h"
#include "StateMachine/FUNCTIONS/MotionControl.h"
#include "config/Config.h"

//* ************************************************************************
//* ****************** HOMING FUNCTIONS ********************
//* ************************************************************************
// This module implements all functions related to homing operations
// including motion control, position establishment, and sequence execution

void initializeHomingSequence() {
    //! Initialize homing sequence parameters
    setupHomingParameters();
    if (stepper) {
        stepper->forceStopAndNewPosition(0);
    }
}

void executeHomingMovement() {
    //! Execute the main homing movement
    if (!stepper) return;
    stepper->setSpeedInHz(Motion::HOMING_SPEED);
    stepper->setAcceleration(Motion::FORWARD_ACCEL);
    stepper->moveTo(0);
    
    waitForHomingComplete();
}

void moveToHomeOffset() {
    //! Move to the home offset position
    moveStepperToPosition(Motion::HOME_OFFSET, Motion::HOMING_SPEED, Motion::FORWARD_ACCEL);
}

void waitForHomingComplete() {
    //! Wait for homing movement to complete
    if (!stepper) return;
    while (stepper->isRunning()) {
        delay(1);
    }
    delay(Timing::MOTION_SETTLE_TIME);
}

void validateHomingPosition() {
    //! Validate that homing was successful
    // Add any position validation logic here if needed
}

void setupHomingParameters() {
    //! Setup parameters for homing sequence
    if (!stepper) return;
    stepper->setSpeedInHz(Motion::HOMING_SPEED);
    stepper->setAcceleration(Motion::FORWARD_ACCEL);
} 