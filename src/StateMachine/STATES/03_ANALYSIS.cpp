#include "StateMachine/STATES/03_ANALYSIS.h"
#include "StateMachine/FUNCTIONS/AnalysisControl.h"
#include "config/Config.h"

//* ************************************************************************
//* ************************ ANALYSIS ***************************
//* ************************************************************************
// This state handles basic analysis sequence initialization for the cutting cycle

void executeAnalysisState() {
    //! Execute basic analysis sequence initialization
    initializeAnalysisSequence();
}

bool isAnalysisComplete() {
    //! Check if analysis sequence is complete
    return analysisResultReceived;
}

void resetAnalysisState() {
    //! Reset analysis state to initial conditions
    lastDetectedClass = "";
    analysisResultReceived = false;
} 