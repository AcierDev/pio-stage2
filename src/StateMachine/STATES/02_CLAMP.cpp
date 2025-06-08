#include "StateMachine/STATES/02_CLAMP.h"
#include "StateMachine/FUNCTIONS/SequenceManagement.h"
#include "StateMachine/FUNCTIONS/PneumaticControl.h"
#include "config/Config.h"

//* ************************************************************************
//* ************************ CLAMP ***************************
//* ************************************************************************
// This state handles the clamp sequence including alignment cylinder
// operations and clamp positioning for the cutting cycle

void executeClampState() {
    //! Execute complete clamp sequence
    initializeClampSequence();
    executeClampSequence();
    validateClampState();
}

bool isClampComplete() {
    //! Check if clamp sequence is complete
    return clampPositionReached;
}

void resetClampState() {
    //! Reset clamp state to initial conditions
    clampPositionReached = false;
    alignmentCylinderExtended = false;
} 