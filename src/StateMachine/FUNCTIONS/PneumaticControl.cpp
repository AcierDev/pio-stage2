#include "StateMachine/FUNCTIONS/PneumaticControl.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"

//* ************************************************************************
//* ****************** PNEUMATIC CONTROL FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to pneumatic operations
// including clamp cylinders, alignment cylinder, and solenoid control

// Global variables for pneumatic state tracking
bool clampPositionReached = false;
bool alignmentCylinderExtended = false;

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



void waitForClampPosition() {
    //! Wait for clamp to reach position (placeholder)
    delay(Timing::CLAMP_EXTEND_TIME);
} 