#include "cutting_cycle.h"
#include "Config/system_config.h"

//* ************************************************************************
//* ****************** CUTTING CYCLE FUNCTIONS ********************
//* ************************************************************************
// This module implements all functions related to the cutting cycle operations
// including state management, motion control, and sequence execution

namespace CuttingCycle {

//* ************************************************************************
//* ************************ STATE MANAGEMENT *************************
//* ************************************************************************
// Functions for managing cutting cycle states

void initializeCuttingCycle() {
    //! Initialize cutting cycle system
    logMessage("Initializing cutting cycle system", "info");
    
    // Reset stepper position and state
    stepper.setCurrentPosition(0);
    stepper.setMaxSpeed(Motion::HOMING_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
    
    // Initialize clamps to released state
    releaseClamps();
    
    logMessage("Cutting cycle system initialized", "info");
}

void resetCuttingCycle() {
    //! Reset cutting cycle to initial state
    logMessage("Resetting cutting cycle", "info");
    
    // Stop any current motion
    stepper.stop();
    
    // Release all clamps
    releaseClamps();
    
    // Reset flags
    analysisResultReceived = false;
    inferenceTimingActive = false;
    lastDetectedClass = "";
    
    logMessage("Cutting cycle reset complete", "info");
}

bool startCuttingCycle() {
    //! Start the cutting cycle sequence
    logMessage("Starting cutting cycle", "info");
    
    // Check if system is ready
    if (stepper.isRunning()) {
        logMessage("Cannot start cycle - stepper still running", "error");
        return false;
    }
    
    // Start the full cutting cycle
    runFullCuttingCycle();
    
    return true;
}

void stopCuttingCycle() {
    //! Emergency stop of cutting cycle
    logMessage("Stopping cutting cycle", "warn");
    
    // Immediate stop
    stepper.stop();
    
    // Release clamps for safety
    releaseClamps();
    
    logMessage("Cutting cycle stopped", "warn");
}

//* ************************************************************************
//* ************************ MOTION CONTROL ***************************
//* ************************************************************************
// Functions for controlling cutting motion sequences

void executeHomingSequence() {
    //! Execute homing sequence to establish reference position
    logMessage("Starting homing sequence", "info");
    
    stepper.setMaxSpeed(Motion::HOMING_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
    
    // Move to home position
    moveStepperToPosition(-Motion::HOME_OFFSET, Motion::HOMING_SPEED, Motion::FORWARD_ACCEL);
    
    delay(Timing::HOME_SETTLE_TIME);
    logMessage("Homing sequence complete", "info");
}

void executeApproachSequence() {
    //! Execute approach to cutting position
    logMessage("Starting approach sequence", "info");
    
    stepper.setMaxSpeed(Motion::APPROACH_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
    
    // Move to approach position
    moveStepperToPosition(Motion::APPROACH_DISTANCE, Motion::APPROACH_SPEED, Motion::FORWARD_ACCEL);
    
    delay(Timing::MOTION_SETTLE_TIME);
    logMessage("Approach sequence complete", "info");
}

void executeCuttingSequence() {
    //! Execute cutting motion at slow speed
    logMessage("Starting cutting sequence", "info");
    
    stepper.setMaxSpeed(Motion::CUTTING_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
    
    // Move through cutting distance
    moveStepperToPosition(Motion::CUTTING_DISTANCE, Motion::CUTTING_SPEED, Motion::FORWARD_ACCEL);
    
    delay(Timing::MOTION_SETTLE_TIME);
    logMessage("Cutting sequence complete", "info");
}

void executeFinishSequence() {
    //! Execute finish motion to complete cut
    logMessage("Starting finish sequence", "info");
    
    stepper.setMaxSpeed(Motion::FINISH_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
    
    // Move to forward position
    float endDropPosition = Motion::FORWARD_DISTANCE - Motion::END_DROP_DISTANCE_OFFSET;
    moveStepperToPosition(endDropPosition, Motion::FINISH_SPEED, Motion::FORWARD_ACCEL);
    
    delay(Timing::MOTION_SETTLE_TIME);
    logMessage("Finish sequence complete", "info");
}

void executeReturnSequence() {
    //! Execute return motion to home position
    logMessage("Starting return sequence", "info");
    
    stepper.setMaxSpeed(Motion::RETURN_SPEED);
    stepper.setAcceleration(Motion::RETURN_ACCEL);
    
    // Return to home
    moveStepperToPosition(-Motion::HOME_OFFSET, Motion::RETURN_SPEED, Motion::RETURN_ACCEL);
    
    delay(Timing::MOTION_SETTLE_TIME);
    logMessage("Return sequence complete", "info");
}

//* ************************************************************************
//* ************************ CYCLE MANAGEMENT *************************
//* ************************************************************************
// Functions for complete cycle management

void runFullCuttingCycle() {
    //! Execute complete cutting cycle sequence
    logMessage("Starting full cutting cycle", "info");
    
    // Execute all sequences in order
    executeHomingSequence();
    executeApproachSequence();
    executeCuttingSequence();
    executeFinishSequence();
    executeReturnSequence();
    
    logMessage("Full cutting cycle complete", "info");
}

void handleCycleError() {
    //! Handle errors during cutting cycle
    logMessage("Handling cycle error", "error");
    
    // Stop all motion
    stopCuttingCycle();
    
    // Reset to safe state
    resetCuttingCycle();
    
    logMessage("Cycle error handled - system reset", "error");
}

bool isCycleComplete() {
    //! Check if cutting cycle is complete
    return !stepper.isRunning();
}

void updateCycleStatus() {
    //! Update cutting cycle status
    if (stepper.isRunning()) {
        logMessage("Cycle in progress - stepper running", "debug");
    } else {
        logMessage("Cycle idle - stepper stopped", "debug");
    }
}

//* ************************************************************************
//* ************************ UTILITY FUNCTIONS ************************
//* ************************************************************************
// Additional utility functions for cutting cycle operations

void releaseClamps() {
    //! Release both clamps simultaneously
    logMessage("Releasing clamps", "info");
    
    digitalWrite(Pins::LEFT_CLAMP, HIGH);   // Retract left clamp
    digitalWrite(Pins::RIGHT_CLAMP, HIGH);  // Retract right clamp
    
    delay(Timing::CLAMP_RELEASE_TIME);
    logMessage("Clamps released", "info");
}

void engageClamps() {
    //! Engage both clamps simultaneously
    logMessage("Engaging clamps", "info");
    
    digitalWrite(Pins::LEFT_CLAMP, LOW);    // Extend left clamp
    digitalWrite(Pins::RIGHT_CLAMP, LOW);   // Extend right clamp
    
    delay(Timing::CLAMP_ENGAGE_TIME);
    logMessage("Clamps engaged", "info");
}

void moveStepperToPosition(float position, float speed, float acceleration) {
    //! Move stepper to specified position with given speed and acceleration
    logMessage("Moving stepper to position: " + String(position) + " inches", "debug");
    
    // Convert position from inches to steps
    long targetSteps = position * Motion::STEPS_PER_INCH;
    
    // Set speed and acceleration
    stepper.setMaxSpeed(speed);
    stepper.setAcceleration(acceleration);
    
    // Move to target position
    stepper.moveTo(targetSteps);
    
    // Wait for movement to complete
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    
    // Add settle time
    delay(Timing::MOTION_SETTLE_TIME);
    
    logMessage("Stepper movement complete", "debug");
}

} // namespace CuttingCycle 