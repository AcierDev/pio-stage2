#ifndef CUTTING_CYCLE_H
#define CUTTING_CYCLE_H

#include <Arduino.h>

//* ************************************************************************
//* ************************ CUTTING CYCLE ***************************
//* ************************************************************************
// This state orchestrates the complete cutting cycle by managing
// all individual states in the proper sequence

// State enumeration
enum CuttingCycleState {
    CYCLE_IDLE,
    CYCLE_CLAMP,
    CYCLE_ANALYSIS,
    CYCLE_HOMING,
    CYCLE_APPROACH,
    CYCLE_CUTTING,
    CYCLE_FINISH,
    CYCLE_RETURN,
    CYCLE_COMPLETE,
    CYCLE_ERROR
};

// Forward declarations
extern CuttingCycleState currentCycleState;
extern String lastDetectedClass;
extern bool analysisResultReceived;

// Function declarations
void executeCuttingCycle();
void initializeCuttingCycle();
void updateCuttingCycleState();
bool isCuttingCycleComplete();
void resetCuttingCycle();
void handleEmptyClassDetection();

#endif // CUTTING_CYCLE_H 