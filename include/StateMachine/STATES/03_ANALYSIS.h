#ifndef ANALYSIS_H
#define ANALYSIS_H

#include <Bounce2.h>
#include <Arduino.h>

//* ************************************************************************
//* ************************ ANALYSIS ***************************
//* ************************************************************************
// This state handles the camera analysis sequence including wood detection,
// classification, and response processing for the cutting cycle

// Forward declarations
extern Bounce cameraSignal;
extern String lastDetectedClass;
extern bool analysisResultReceived;
extern unsigned long inferenceStartTime;
extern bool inferenceTimingActive;

// Function declarations
void executeAnalysisState();
bool isAnalysisComplete();
void resetAnalysisState();

#endif // ANALYSIS_H 