#ifndef ANALYSIS_CONTROL_H
#define ANALYSIS_CONTROL_H

#include <Arduino.h>

//* ************************************************************************
//* ****************** ANALYSIS CONTROL FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to camera analysis operations
// including signal detection, burst requests, and response handling

// Function declarations
void initializeAnalysisSequence();
void checkCameraSignal();
void sendBurstRequest();
void waitForAnalysisResult();
void processAnalysisResponse();
void handleAnalysisTimeout();
void validateAnalysisResult();

#endif // ANALYSIS_CONTROL_H 