#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <Arduino.h>

//* ************************************************************************
//* ****************** MOTION CONTROL FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to stepper motor control
// including positioning, movement execution, and motion parameters

// Function declarations
void moveStepperToPosition(float position, float speed, float acceleration);
void moveToApproachPosition();
void waitForApproachComplete();
void moveThroughCuttingDistance();
void waitForCuttingComplete();
void executeFastReturn();
void executeSlowApproach();
void setupApproachParameters();
void setupCuttingParameters();
void setupReturnParameters();
void stopMotion();
void setMotorPosition(float position);
void moveToForwardPosition();
void waitForFinishComplete();
void setupFinishParameters();

#endif // MOTION_CONTROL_H 