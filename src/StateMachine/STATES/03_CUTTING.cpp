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
    stepper->setSpeedInHz(Motion::CUTTING_SPEED);
    stepper->setAcceleration(Motion::FORWARD_ACCEL);
    stepper->moveTo((Motion::APPROACH_DISTANCE + Motion::CUTTING_DISTANCE) * Motion::STEPS_PER_INCH);
    while (stepper->rampState() != RAMP_STATE_IDLE) {
        delay(1);  // Wait for stepper motor ramp to complete
    }
    logMessage("✅ Cutting phase complete");
}

bool isCuttingComplete() {
    //! Check if cutting sequence is complete (motor ramp idle)
    return stepper->rampState() == RAMP_STATE_IDLE;
}

void resetCuttingState() {
    //! Reset cutting state to initial conditions
    stepper->forceStop();
} 