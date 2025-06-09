#include "system_states.h"
#include "StateMachine/FUNCTIONS/MotionControl.h"
#include "config/Config.h"

//* ************************************************************************
//* ****************** MOTION CONTROL FUNCTIONS ********************
//* ************************************************************************
// This module implements all functions related to stepper motor control
// including positioning, movement execution, and motion parameters

void moveStepperToPosition(float position, float speed, float acceleration) {
    //! Move stepper to specified position with given parameters
    stepper->setSpeedInHz(speed);
    stepper->setAcceleration(acceleration);
    stepper->moveTo(position * Motion::STEPS_PER_INCH);
}

void moveToApproachPosition() {
    //! Move to the approach position
    moveStepperToPosition(Motion::APPROACH_DISTANCE, Motion::APPROACH_SPEED, Motion::FORWARD_ACCEL);
}

void waitForApproachComplete() {
    //! Wait for approach movement to complete
    while (stepper->rampState() != RAMP_STATE_IDLE) {
        delay(1);  // Wait for stepper motor ramp to complete
    }
    delay(Timing::MOTION_SETTLE_TIME);
}

void moveThroughCuttingDistance() {
    //! Move through the cutting distance at slow speed
    float targetPosition = Motion::APPROACH_DISTANCE + Motion::CUTTING_DISTANCE;
    moveStepperToPosition(targetPosition, Motion::CUTTING_SPEED, Motion::FORWARD_ACCEL * 2.0);
}

void waitForCuttingComplete() {
    //! Wait for cutting movement to complete
    while (stepper->rampState() != RAMP_STATE_IDLE) {
        delay(1);  // Wait for stepper motor ramp to complete
    }
    delay(Timing::MOTION_SETTLE_TIME);
}

void executeFastReturn() {
    //! Execute fast return to slowdown position
    float currentPosition = stepper->getCurrentPosition() / (float)Motion::STEPS_PER_INCH;
    float slowDownPosition = currentPosition * 0.01;

    stepper->setSpeedInHz(Motion::RETURN_SPEED);
    stepper->setAcceleration(Motion::RETURN_ACCEL);
    stepper->moveTo(slowDownPosition * Motion::STEPS_PER_INCH);

    unsigned long fastReturnStartTime = millis();
    unsigned long fastReturnTimeout = 15000;

    while (stepper->rampState() != RAMP_STATE_IDLE) {
        delay(1);  // Wait for stepper motor ramp to complete
        if (millis() - fastReturnStartTime > fastReturnTimeout) {
            break;
        }
    }
}

void executeSlowApproach() {
    //! Execute slow approach to home position
    float slowHomingSpeed = Motion::HOMING_SPEED;
    stepper->setSpeedInHz(slowHomingSpeed);
    stepper->setAcceleration(Motion::RETURN_ACCEL);
    stepper->moveTo(Motion::HOME_OFFSET * Motion::STEPS_PER_INCH);  // Go to home offset, not 0

    unsigned long slowApproachStartTime = millis();
    unsigned long slowApproachTimeout = 20000;

    while (stepper->rampState() != RAMP_STATE_IDLE) {
        delay(1);  // Wait for stepper motor ramp to complete
        if (millis() - slowApproachStartTime > slowApproachTimeout) {
            logMessage("⚠ Slow approach timeout - stopping movement");
            stepper->forceStop();
            break;
        }
    }

    delay(30);
}

void setupApproachParameters() {
    //! Setup parameters for approach sequence
    stepper->setSpeedInHz(Motion::APPROACH_SPEED);
    stepper->setAcceleration(Motion::FORWARD_ACCEL);
}

void setupCuttingParameters() {
    //! Setup parameters for cutting sequence
    stepper->setSpeedInHz(Motion::CUTTING_SPEED);
    stepper->setAcceleration(Motion::FORWARD_ACCEL * 2.0);
}

void setupReturnParameters() {
    //! Setup parameters for return sequence
    stepper->setSpeedInHz(Motion::RETURN_SPEED);
    stepper->setAcceleration(Motion::RETURN_ACCEL);
}

void stopMotion() {
    //! Stop all motor motion
    stepper->forceStop();
}

void setMotorPosition(float position) {
    //! Set current motor position
    stepper->setCurrentPosition(position);
}

void moveToForwardPosition() {
    //! Move to the forward position
    moveStepperToPosition(Motion::FORWARD_DISTANCE, Motion::FINISH_SPEED, Motion::FORWARD_ACCEL);
}

void waitForFinishComplete() {
    //! Wait for finish movement to complete
    while (stepper->rampState() != RAMP_STATE_IDLE) {
        delay(1);  // Wait for stepper motor ramp to complete
    }
    delay(50);
}

void setupFinishParameters() {
    //! Setup parameters for finish sequence
    stepper->setSpeedInHz(Motion::FINISH_SPEED);
    stepper->setAcceleration(Motion::FORWARD_ACCEL);
} 