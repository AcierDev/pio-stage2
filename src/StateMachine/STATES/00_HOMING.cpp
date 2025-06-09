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

    // Check if we're already on the home switch - if so, move away first
    homeSwitch.update();
    if (homeSwitch.read() == HIGH) {
        logMessage("Already on home switch - moving away first");
        stepper->setSpeedInHz(Motion::HOMING_SPEED);
        stepper->setAcceleration(Motion::FORWARD_ACCEL);
        stepper->move(2000);  // Move 2000 steps away from home
        
        while (stepper->rampState() != RAMP_STATE_IDLE) {
            delay(1);  // Wait for stepper motor ramp to complete
        }
        delay(100);  // Extra settle time
    }

    // Now move toward home switch (negative direction)
    stepper->setSpeedInHz(Motion::HOMING_SPEED);
    stepper->setAcceleration(Motion::FORWARD_ACCEL);
    stepper->move(-15000);  // Move a large distance toward home

    // Monitor for home switch activation during movement
    bool homeFound = false;
    while (stepper->rampState() != RAMP_STATE_IDLE && !homeFound) {
        homeSwitch.update();
        
        if (homeSwitch.read() == HIGH) {
            // Home switch activated - stop immediately and set this as position 0
            stepper->forceStopAndNewPosition(0);
            homeFound = true;
            logMessage("Home switch found during movement");
        }
        delay(1);  // Wait for stepper motor ramp to complete
    }

    // If we didn't find home switch during movement, check final position
    if (!homeFound) {
        homeSwitch.update();
        if (homeSwitch.read() == HIGH) {
            stepper->setCurrentPosition(0);
            homeFound = true;
            logMessage("Home switch found at final position");
        }
    }

    // If we still didn't find the home switch, we have a problem
    if (!homeFound) {
        logMessage("⚠ Failed to find home switch during homing!", "error");
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