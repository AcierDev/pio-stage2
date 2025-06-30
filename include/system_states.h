#pragma once

#include <FastAccelStepper.h>
#include <Arduino.h>
#include <Bounce2.h>

// System state enumeration - keep it simple
enum class SystemState { 
    INITIALIZING, 
    HOMING, 
    READY, 
    CYCLE_RUNNING, 
    ERROR 
};

// Global objects declarations
extern FastAccelStepperEngine engine;
extern FastAccelStepper *stepper;
extern Bounce homeSwitch;
extern Bounce startButton;
extern Bounce transferArmStartSignal;

// System state tracking
extern SystemState currentState;
extern bool isHomed; 