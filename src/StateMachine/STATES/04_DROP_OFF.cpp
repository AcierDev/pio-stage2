#include "StateMachine/STATES/04_DROP_OFF.h"
#include "system_states.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ DROP OFF ***************************
//* ************************************************************************
// This state handles the drop off sequence to complete the cutting operation
// and position the stepper at the forward position for piece drop off

void executeDropOffState() {
    //! Execute complete drop off sequence
    handleEndClassDetection();
    executeDropOffMovement();
    releaseClamps();
}

void executeDropOffMovement() {
    logMessage("🏁 Drop off phase...");
    stepper->setSpeedInHz(Motion::FINISH_SPEED);
    stepper->setAcceleration(Motion::FORWARD_ACCEL);
    stepper->moveTo(Motion::FORWARD_DISTANCE * Motion::STEPS_PER_INCH);
    while (stepper->rampState() != RAMP_STATE_IDLE) {
        delay(1);  // Wait for stepper motor ramp to complete
    }
    stepper->forceStop();
    delay(50);
    logMessage("✅ Drop off phase complete");
}

void handleEndClassDetection() {
    // Handle "End" class detection
    if (analysisResultReceived && lastDetectedClass.equalsIgnoreCase("End")) {
        logMessage("======= END CLASS DETECTED =======");
        float intermediatePosition = Motion::FORWARD_DISTANCE - Motion::END_DROP_DISTANCE_OFFSET;
        logMessage("Moving to intermediate position: " + String(intermediatePosition));

        stepper->setSpeedInHz(Motion::FINISH_SPEED);
        stepper->setAcceleration(Motion::FORWARD_ACCEL);
        stepper->moveTo(intermediatePosition * Motion::STEPS_PER_INCH);
        while (stepper->rampState() != RAMP_STATE_IDLE) {
            delay(1);  // Wait for stepper motor ramp to complete
        }
        stepper->forceStop();
        delay(Timing::MOTION_SETTLE_TIME);

        logMessage("Deactivating left clamp...");
        digitalWrite(Pins::LEFT_CLAMP, HIGH);  // Deactivate left clamp
        delay(200);

        logMessage("Proceeding to final forward distance.");
        logMessage("==================================");
    }
}

// releaseClamps() function is defined in main.cpp

bool isDropOffComplete() {
    //! Check if drop off sequence is complete (motor ramp idle)
    return stepper->rampState() == RAMP_STATE_IDLE;
}

void resetDropOffState() {
    //! Reset drop off state to initial conditions
    stepper->forceStop();
} 