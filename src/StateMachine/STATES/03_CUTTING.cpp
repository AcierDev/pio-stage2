#include "StateMachine/STATES/03_CUTTING.h"
#include "system_states.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ CUTTING ***************************
//* ************************************************************************
// This state handles the cutting sequence where the stepper moves
// slowly through the material to perform the actual cutting operation

void executeCuttingState() {
    //! Execute complete cutting sequence
    executeCuttingMovement();
}

void executeCuttingMovement() {
    logMessage("🔪 Cutting phase...");
    stepper.setMaxSpeed(Motion::CUTTING_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL * 2);
    stepper.moveTo((Motion::APPROACH_DISTANCE + Motion::CUTTING_DISTANCE) * Motion::STEPS_PER_INCH);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    logMessage("✅ Cutting phase complete");
}

bool isCuttingComplete() {
    //! Check if cutting sequence is complete
    return !stepper.isRunning();
}

void resetCuttingState() {
    //! Reset cutting state to initial conditions
    stepper.stop();
} 