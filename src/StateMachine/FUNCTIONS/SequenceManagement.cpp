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



void validateFinishPosition() {
    //! Validate that finish was successful
    // Add any position validation logic here if needed
} 