#include "system_states.h"
#include "StateMachine/FUNCTIONS/04_CUTTING_FUNCTIONS.h"
#include "config/Config.h"

//* ************************************************************************
//* ****************** CUTTING FUNCTIONS ********************
//* ************************************************************************
// This module implements all functions related to cutting operations
// including motion control, cutting execution, and sequence management

void initializeCuttingSequence() {
    //! Initialize cutting sequence parameters
    setupCuttingParameters();
}

void executeCuttingMovement() {
    //! Execute the cutting movement through material
    moveThroughCuttingDistance();
    waitForCuttingComplete();
}

void moveThroughCuttingDistance() {
    //! Move through the cutting distance at slow speed
    float targetPosition = Motion::APPROACH_DISTANCE + Motion::CUTTING_DISTANCE;
    moveStepperToPosition(targetPosition, Motion::CUTTING_SPEED, Motion::FORWARD_ACCEL * 2.0);
}

void waitForCuttingComplete() {
    //! Wait for cutting movement to complete
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    delay(Timing::MOTION_SETTLE_TIME);
}

void validateCuttingPosition() {
    //! Validate that cutting was successful
    // Add any position validation logic here if needed
}

void setupCuttingParameters() {
    //! Setup parameters for cutting sequence
    stepper.setMaxSpeed(Motion::CUTTING_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL * 2.0);
} 