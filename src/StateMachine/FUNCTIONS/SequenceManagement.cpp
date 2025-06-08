#include "system_states.h"
#include "StateMachine/FUNCTIONS/SequenceManagement.h"
#include "StateMachine/FUNCTIONS/MotionControl.h"
#include "StateMachine/FUNCTIONS/PneumaticControl.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"

//* ************************************************************************
//* ****************** SEQUENCE MANAGEMENT FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to sequence management
// including initialization, validation, and control operations

void initializeApproachSequence() {
    //! Initialize approach sequence parameters
    setupApproachParameters();
}

void initializeCuttingSequence() {
    //! Initialize cutting sequence parameters
    setupCuttingParameters();
}

void initializeReturnSequence() {
    //! Initialize return sequence parameters
    setupReturnParameters();
}

void initializeClampSequence() {
    //! Initialize clamp sequence variables
    clampPositionReached = false;
    alignmentCylinderExtended = false;
}

void executeClampSequence() {
    //! Execute the complete clamp sequence
    // First retract both clamps, then extend both
    retractClamps();
    delay(Timing::CLAMP_EXTEND_TIME);
    extendClamps();
    delay(Timing::CLAMP_EXTEND_TIME);
    extendAlignmentCylinder();
    delay(Timing::ALIGNMENT_TIME);
    clampPositionReached = true;
}

void executeApproachMovement() {
    //! Execute the approach movement to cutting position
    moveToApproachPosition();
    waitForApproachComplete();
}

void executeCuttingMovement() {
    //! Execute the cutting movement through material
    moveThroughCuttingDistance();
    waitForCuttingComplete();
}

void executeReturnMovement() {
    //! Execute the complete return movement sequence
    executeFastReturn();
    executeSlowApproach();
}

void validateApproachPosition() {
    //! Validate that approach was successful
    // Add any position validation logic here if needed
}

void validateCuttingPosition() {
    //! Validate that cutting was successful
    // Add any position validation logic here if needed
}

void validateClampState() {
    //! Validate clamp state (placeholder)
    clampPositionReached = true;
}

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

void validateFinishPosition() {
    //! Validate that finish was successful
    // Add any position validation logic here if needed
} 