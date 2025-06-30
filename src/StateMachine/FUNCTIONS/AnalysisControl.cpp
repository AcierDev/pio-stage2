#include "system_states.h"
#include "StateMachine/FUNCTIONS/AnalysisControl.h"
#include "config/Config.h"

//* ************************************************************************
//* ****************** ANALYSIS CONTROL FUNCTIONS ********************
//* ************************************************************************
// This module implements basic analysis result tracking functions

void initializeAnalysisSequence() {
    //! Initialize analysis sequence parameters
    lastDetectedClass = "";
    analysisResultReceived = false;
}

void processAnalysisResponse() {
    //! Process the received analysis response
    // This function is called from handleSerialResponse
}

void validateAnalysisResult() {
    //! Validate that analysis result is valid
    // Add any result validation logic here if needed
} 