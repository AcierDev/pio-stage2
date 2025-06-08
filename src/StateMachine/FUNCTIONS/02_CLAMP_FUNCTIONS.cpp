#include "StateMachine/FUNCTIONS/02_CLAMP_FUNCTIONS.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"

//* ************************************************************************
//* ****************** CLAMP FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to clamp operations
// including extend, retract, and alignment cylinder control

// Global variables for clamp state tracking
bool clampPositionReached = false;
bool alignmentCylinderExtended = false;

void initializeClampSequence() {
    //! Initialize clamp sequence variables
    clampPositionReached = false;
    alignmentCylinderExtended = false;
}

void executeClampSequence() {
    //! Execute the complete clamp sequence
    extendClamps();
    delay(Timing::CLAMP_EXTEND_TIME);
    extendAlignmentCylinder();
    delay(Timing::ALIGNMENT_TIME);
    clampPositionReached = true;
}

void extendLeftClamp() {
    //! Extend left clamp cylinder
    digitalWrite(Pins::LEFT_CLAMP, HIGH);
}

void retractLeftClamp() {
    //! Retract left clamp cylinder
    digitalWrite(Pins::LEFT_CLAMP, LOW);
}

void extendRightClamp() {
    //! Extend right clamp cylinder
    digitalWrite(Pins::RIGHT_CLAMP, HIGH);
}

void retractRightClamp() {
    //! Retract right clamp cylinder
    digitalWrite(Pins::RIGHT_CLAMP, LOW);
}

void extendClamps() {
    //! Extend both clamp cylinders
    extendLeftClamp();
    extendRightClamp();
}

void retractClamps() {
    //! Retract both clamp cylinders
    retractLeftClamp();
    retractRightClamp();
}

void extendAlignmentCylinder() {
    //! Extend alignment cylinder
    digitalWrite(Pins::ALIGN_CYLINDER, HIGH);
    alignmentCylinderExtended = true;
}

void retractAlignmentCylinder() {
    //! Retract alignment cylinder
    digitalWrite(Pins::ALIGN_CYLINDER, LOW);
    alignmentCylinderExtended = false;
}

void executeAlignmentSequence() {
    //! Execute alignment cylinder sequence
    extendAlignmentCylinder();
    delay(Timing::ALIGNMENT_TIME);
}

void waitForClampPosition() {
    //! Wait for clamp to reach position (placeholder)
    delay(Timing::CLAMP_EXTEND_TIME);
}

void validateClampState() {
    //! Validate clamp state (placeholder)
    clampPositionReached = true;
} 