#include "system_states.h"
#include "StateMachine/FUNCTIONS/05_RETURN_FUNCTIONS.h"
#include "config/Config.h"

//* ************************************************************************
//* ****************** RETURN FUNCTIONS ********************
//* ************************************************************************
// This module implements all functions related to return operations
// including fast return, slow approach, and final positioning

void initializeReturnSequence() {
    //! Initialize return sequence parameters
    setupReturnParameters();
}

void executeReturnMovement() {
    //! Execute the complete return movement sequence
    executeFastReturn();
    executeSlowApproach();
}



void activateTransferArmSignal() {
    //! Activate transfer arm signal to prevent Z-axis lowering
    digitalWrite(Pins::TRANSFER_ARM_SIGNAL, HIGH);
}

void executeFastReturn() {
    //! Execute fast return to slowdown position
    float currentPosition = stepper.currentPosition() / (float)Motion::STEPS_PER_INCH;
    float slowDownPosition = currentPosition * 0.01;

    stepper.setMaxSpeed(Motion::RETURN_SPEED);
    stepper.setAcceleration(Motion::RETURN_ACCEL);
    stepper.moveTo(slowDownPosition * Motion::STEPS_PER_INCH);

    unsigned long fastReturnStartTime = millis();
    unsigned long fastReturnTimeout = 15000;

    while (stepper.distanceToGo() != 0) {
        stepper.run();
        if (millis() - fastReturnStartTime > fastReturnTimeout) {
            break;
        }
    }
}

void executeSlowApproach() {
    //! Execute slow approach to home position
    float slowHomingSpeed = Motion::HOMING_SPEED / 2.0;
    stepper.setMaxSpeed(slowHomingSpeed);
    stepper.setAcceleration(Motion::RETURN_ACCEL / 4.0);
    stepper.moveTo(0);

    unsigned long slowApproachStartTime = millis();
    unsigned long slowApproachTimeout = 20000;

    while (stepper.distanceToGo() != 0) {
        stepper.run();
        if (millis() - slowApproachStartTime > slowApproachTimeout) {
            break;
        }
    }

    delay(30);
}



void deactivateTransferArmSignal() {
    //! Deactivate transfer arm signal - return is complete
    digitalWrite(Pins::TRANSFER_ARM_SIGNAL, LOW);
    delay(50);
}

void setupReturnParameters() {
    //! Setup parameters for return sequence
    stepper.setMaxSpeed(Motion::RETURN_SPEED);
    stepper.setAcceleration(Motion::RETURN_ACCEL);
} 