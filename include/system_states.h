#pragma once

#include <FastAccelStepper.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Bounce2.h>

#include "config/Config.h"
#include "config/Pins_Definitions.h"

// System state enumeration
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

// Analysis result tracking
extern String lastDetectedClass;
extern bool analysisResultReceived;

// Utility function declarations
void moveStepperToPosition(float position, float speed, float acceleration);
void sendSerialMessage(const String &message);
void handleSerialResponse(const String &response); 