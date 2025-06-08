#include "system_states.h"
#include "StateMachine/FUNCTIONS/AnalysisControl.h"
#include "config/Config.h"

//* ************************************************************************
//* ****************** ANALYSIS CONTROL FUNCTIONS ********************
//* ************************************************************************
// This module implements all functions related to camera analysis operations
// including signal detection, burst requests, and response handling

void initializeAnalysisSequence() {
    //! Initialize analysis sequence parameters
    lastDetectedClass = "";
    analysisResultReceived = false;
    inferenceTimingActive = false;
}

void checkCameraSignal() {
    //! Check camera signal and send burst request if active
    if (cameraSignal.read() == HIGH) {
        sendBurstRequest();
    }
}

void sendBurstRequest() {
    //! Send burst request to camera system
    sendSerialMessage("BURST");
    inferenceStartTime = millis();
    inferenceTimingActive = true;
}

void waitForAnalysisResult() {
    //! Wait for analysis result with timeout
    unsigned long startWaitTime = millis();
    unsigned long waitTimeout = 2000;

    while (!analysisResultReceived && (millis() - startWaitTime < waitTimeout)) {
        if (Serial.available()) {
            String response = Serial.readStringUntil('\n');
            response.trim();
            if (response.length() > 0 && response.startsWith("{")) {
                handleSerialResponse(response);
            }
        }
        delay(10);
    }

    if (!analysisResultReceived) {
        handleAnalysisTimeout();
    }
}

void processAnalysisResponse() {
    //! Process the received analysis response
    // This function is called from handleSerialResponse
}

void handleAnalysisTimeout() {
    //! Handle timeout case for inference timing
    if (inferenceTimingActive) {
        inferenceTimingActive = false;
    }
}

void validateAnalysisResult() {
    //! Validate that analysis result is valid
    // Add any result validation logic here if needed
} 