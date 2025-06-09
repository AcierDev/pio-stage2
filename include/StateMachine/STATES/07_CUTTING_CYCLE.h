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
    CYCLE_ALIGN,
    CYCLE_APPROACH,
    CYCLE_CUTTING,
    CYCLE_DROP_OFF,
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
void executeCurrentCycleState();
void transitionToNextCycleState();
void initializeCuttingCycle();
bool isCuttingCycleComplete();
void resetCuttingCycle();

#endif // CUTTING_CYCLE_H 