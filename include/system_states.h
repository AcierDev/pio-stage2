#pragma once

#include <AccelStepper.h>
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
extern AccelStepper stepper;
extern Bounce homeSwitch;
extern Bounce startButton;
extern Bounce transferArmStartSignal;
extern Bounce cameraSignal;

// System state tracking
extern SystemState currentState;
extern bool isHomed;

// Analysis result tracking
extern String lastDetectedClass;
extern bool analysisResultReceived;

// Image inference timing tracking
extern unsigned long inferenceStartTime;
extern bool inferenceTimingActive;

// Utility function declarations
void logMessage(const String &message, const String &level = "INFO", bool excludeSerial = false);
void moveStepperToPosition(float position, float speed, float acceleration);
void sendSerialMessage(const String &message);
void handleSerialResponse(const String &response); 