#include "system_states.h"
#include "StateMachine/FUNCTIONS/06_FINISH_FUNCTIONS.h"
#include "config/Config.h"

//* ************************************************************************
//* ****************** FINISH FUNCTIONS ********************
//* ************************************************************************
// This module implements all functions related to finish operations
// including end drop handling, clamp control, and final positioning

void initializeFinishSequence() {
    //! Initialize finish sequence parameters
    setupFinishParameters();
}

void executeFinishMovement() {
    //! Execute the finish movement to forward position
    moveToForwardPosition();
    waitForFinishComplete();
}

void handleEndClassDetection() {
    //! Handle "End" class detection with intermediate positioning
    if (analysisResultReceived && lastDetectedClass.equalsIgnoreCase("End")) {
        float intermediatePosition = Motion::FORWARD_DISTANCE - Motion::END_DROP_DISTANCE_OFFSET;
        moveStepperToPosition(intermediatePosition, Motion::FINISH_SPEED, Motion::FORWARD_ACCEL);
        stepper.stop();
        delay(Timing::MOTION_SETTLE_TIME);

        // Retract left clamp for end pieces
        digitalWrite(Pins::LEFT_CLAMP, HIGH);
        delay(200);
    }
}

void moveToForwardPosition() {
    //! Move to the forward position
    moveStepperToPosition(Motion::FORWARD_DISTANCE, Motion::FINISH_SPEED, Motion::FORWARD_ACCEL);
}

void waitForFinishComplete() {
    //! Wait for finish movement to complete
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    delay(50);
}

void validateFinishPosition() {
    //! Validate that finish was successful
    // Add any position validation logic here if needed
}

void setupFinishParameters() {
    //! Setup parameters for finish sequence
    stepper.setMaxSpeed(Motion::FINISH_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
} 