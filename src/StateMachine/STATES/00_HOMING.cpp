#include "StateMachine/STATES/00_HOMING.h"
#include "system_states.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ HOMING ***************************
//* ************************************************************************
// This state handles the homing sequence to establish reference position
// for the stepper motor before starting cutting operations

void executeHomingState() {
    //! Execute complete homing sequence
    performHomingSequence();
}

void performHomingSequence() {
    logMessage("🏠 Starting homing sequence...");
    logMessage("Using home offset: " + String(Motion::HOME_OFFSET) + " inches");
    currentState = SystemState::HOMING;

    // Clamps are already engaged from initialization

    // First, move a significant distance in the negative direction to ensure
    // we're past the home switch
    stepper->setSpeedInHz(Motion::HOMING_SPEED);
    stepper->setAcceleration(Motion::FORWARD_ACCEL);
    stepper->moveTo(-10000);  // Move 10,000 steps in negative direction

    // Use a much slower approach speed for final homing
    float slowHomingSpeed = Motion::HOMING_SPEED / 3;  // One-third of normal homing speed

    // Run until we hit the home switch or reach the target
    while (stepper->rampState() != RAMP_STATE_IDLE) {
        homeSwitch.update();

        // If we're within 2000 steps of where we think home might be, slow down
        // significantly
        if (abs(stepper->getCurrentPosition()) < 2000) {
            stepper->setSpeedInHz(slowHomingSpeed);
        }

        if (homeSwitch.read() == HIGH) {
            // When home switch is triggered, stop immediately
            stepper->forceStopAndNewPosition(0);
            break;
        }
        delay(1);  // Wait for stepper motor ramp to complete
    }

    // If we didn't hit the home switch, we have a problem
    if (homeSwitch.read() == LOW) {
        logMessage("⚠ Failed to find home switch during initial homing!", "error");
        currentState = SystemState::ERROR;
        return;
    }

    // Now move to home offset with a gentler motion
    logMessage("Moving to home offset position: " + String(Motion::HOME_OFFSET) + " inches");
    stepper->setSpeedInHz(Motion::HOMING_SPEED / 2);  // Half speed for moving to offset
    stepper->setAcceleration(Motion::FORWARD_ACCEL / 2);  // Gentler acceleration
    stepper->moveTo(Motion::HOME_OFFSET * Motion::STEPS_PER_INCH);
    
    while (stepper->rampState() != RAMP_STATE_IDLE) {
        delay(1);  // Wait for stepper motor ramp to complete
    }
    
    // Add settle time after reaching home offset
    delay(Timing::HOME_SETTLE_TIME);
    
    logMessage("Home offset position reached. Current position: " + 
               String(stepper->getCurrentPosition() / Motion::STEPS_PER_INCH) + " inches");

    // Now that we're at home position, release the clamps
    digitalWrite(Pins::LEFT_CLAMP, HIGH);
    digitalWrite(Pins::RIGHT_CLAMP, HIGH);
    
    // Add settle time after releasing clamps
    delay(Timing::CLAMP_RELEASE_TIME);

    isHomed = true;
    logMessage("✅ Homing complete");
}

bool isHomingComplete() {
    //! Check if homing sequence is complete
    return isHomed;
}

void resetHomingState() {
    //! Reset homing state to initial conditions
    isHomed = false;
    stepper->forceStop();
    stepper->setCurrentPosition(0);
} 