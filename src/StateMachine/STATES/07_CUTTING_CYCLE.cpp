#include "StateMachine/STATES/07_CUTTING_CYCLE.h"
#include "StateMachine/STATES/01_ALIGN.h"
#include "StateMachine/STATES/02_APPROACH.h"
#include "StateMachine/STATES/03_CUTTING.h"
#include "StateMachine/STATES/04_DROP_OFF.h"
#include "StateMachine/STATES/05_RETURN.h"
#include "system_states.h"
#include "config/Config.h"

//* ************************************************************************
//* ************************ CUTTING CYCLE ***************************
//* ************************************************************************
// This state orchestrates the complete cutting cycle by managing
// all individual states in the proper sequence: ALIGN → APPROACH → CUTTING → DROP OFF → RETURN

// Global state variable
CuttingCycleState currentCycleState = CYCLE_IDLE;

void executeCuttingCycle() {
    //! Execute the complete cutting cycle state machine
    logMessage("🔄 Starting cutting cycle...");
    
    // Reset analysis result tracking at the start of each cycle
    lastDetectedClass = "";
    analysisResultReceived = false;
    
    currentCycleState = CYCLE_ALIGN;
    
    while (!isCuttingCycleComplete()) {
        executeCurrentCycleState();
        transitionToNextCycleState();
    }
    
    logMessage("✅ Cutting cycle complete!");
}

void executeCurrentCycleState() {
    //! Execute the current state in the cutting cycle
    switch (currentCycleState) {
        case CYCLE_ALIGN:
            executeAlignState();
            break;
        case CYCLE_APPROACH:
            executeApproachState();
            break;
        case CYCLE_CUTTING:
            executeCuttingState();
            break;
        case CYCLE_DROP_OFF:
            executeDropOffState();
            break;
        case CYCLE_RETURN:
            executeReturnState();
            break;
        default:
            break;
    }
}

void transitionToNextCycleState() {
    //! Transition to the next state in the cutting cycle
    switch (currentCycleState) {
        case CYCLE_ALIGN:
            // Check for empty class detection after alignment/analysis
            if (isEmptyClassDetected()) {
                logMessage("======= SHORT-CIRCUITING CYCLE =======");
                logMessage("Empty wood detected - skipping cutting operations");
                logMessage("========================================");
                releaseClamps();
                executeEmptyClassReturn();
                currentCycleState = CYCLE_COMPLETE;
            } else {
                currentCycleState = CYCLE_APPROACH;
            }
            break;
        case CYCLE_APPROACH:
            currentCycleState = CYCLE_CUTTING;
            break;
        case CYCLE_CUTTING:
            currentCycleState = CYCLE_DROP_OFF;
            break;
        case CYCLE_DROP_OFF:
            currentCycleState = CYCLE_RETURN;
            break;
        case CYCLE_RETURN:
            currentCycleState = CYCLE_COMPLETE;
            break;
        default:
            currentCycleState = CYCLE_COMPLETE;
            break;
    }
}

void initializeCuttingCycle() {
    //! Initialize cutting cycle to starting state
    currentCycleState = CYCLE_ALIGN;
    lastDetectedClass = "";
    analysisResultReceived = false;
}

bool isCuttingCycleComplete() {
    //! Check if cutting cycle is complete
    return (currentCycleState == CYCLE_COMPLETE || currentCycleState == CYCLE_ERROR);
}

void resetCuttingCycle() {
    //! Reset cutting cycle to initial state
    currentCycleState = CYCLE_IDLE;
    lastDetectedClass = "";
    analysisResultReceived = false;
} 