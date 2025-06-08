#ifndef ANALYSIS_FUNCTIONS_H
#define ANALYSIS_FUNCTIONS_H

#include <Bounce2.h>
#include <Arduino.h>

//* ************************************************************************
//* ****************** ANALYSIS FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to camera analysis operations
// including signal detection, burst requests, and response handling

// Forward declarations
extern Bounce cameraSignal;
extern String lastDetectedClass;
extern bool analysisResultReceived;
extern unsigned long inferenceStartTime;
extern bool inferenceTimingActive;

// Function declarations
void initializeAnalysisSequence();
void checkCameraSignal();
void sendBurstRequest();
void waitForAnalysisResult();
void processAnalysisResponse();
void handleAnalysisTimeout();
void validateAnalysisResult();

#endif // ANALYSIS_FUNCTIONS_H 