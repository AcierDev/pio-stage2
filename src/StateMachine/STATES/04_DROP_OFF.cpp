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
    stepper.setMaxSpeed(Motion::FINISH_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
    stepper.moveTo(Motion::FORWARD_DISTANCE * Motion::STEPS_PER_INCH);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    stepper.stop();
    delay(50);
    logMessage("✅ Drop off phase complete");
}

void handleEndClassDetection() {
    // Handle "End" class detection
    if (analysisResultReceived && lastDetectedClass.equalsIgnoreCase("End")) {
        logMessage("======= END CLASS DETECTED =======");
        float intermediatePosition = Motion::FORWARD_DISTANCE - Motion::END_DROP_DISTANCE_OFFSET;
        logMessage("Moving to intermediate position: " + String(intermediatePosition));

        stepper.setMaxSpeed(Motion::FINISH_SPEED);
        stepper.setAcceleration(Motion::FORWARD_ACCEL);
        stepper.moveTo(intermediatePosition * Motion::STEPS_PER_INCH);
        while (stepper.distanceToGo() != 0) {
            stepper.run();
        }
        stepper.stop();
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
    //! Check if drop off sequence is complete
    return !stepper.isRunning();
}

void resetDropOffState() {
    //! Reset drop off state to initial conditions
    stepper.stop();
} 