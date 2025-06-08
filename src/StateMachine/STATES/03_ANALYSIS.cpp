#include "StateMachine/STATES/03_ANALYSIS.h"
#include "StateMachine/FUNCTIONS/03_ANALYSIS_FUNCTIONS.h"
#include "config/Config.h"

//* ************************************************************************
//* ************************ ANALYSIS ***************************
//* ************************************************************************
// This state handles the camera analysis sequence including wood detection,
// classification, and response processing for the cutting cycle

void executeAnalysisState() {
    //! Execute complete analysis sequence
    initializeAnalysisSequence();
    checkCameraSignal();
    waitForAnalysisResult();
    validateAnalysisResult();
}

bool isAnalysisComplete() {
    //! Check if analysis sequence is complete
    return analysisResultReceived;
}

void resetAnalysisState() {
    //! Reset analysis state to initial conditions
    lastDetectedClass = "";
    analysisResultReceived = false;
    inferenceTimingActive = false;
} 