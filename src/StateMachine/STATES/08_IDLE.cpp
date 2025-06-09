#include "StateMachine/STATES/08_IDLE.h"
#include "system_states.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ IDLE ***************************
//* ************************************************************************
// This state represents the idle/waiting state where the machine waits
// for the start signal to begin a cutting cycle

void executeIdleState() {
    //! Execute idle state - wait for start signal
    currentState = SystemState::READY;
    
    // Update debounced inputs
    startButton.update();
    transferArmStartSignal.update();
    
    // Check for start conditions
    if (isStartSignalReceived()) {
        logMessage("🚀 Start signal received - beginning cutting cycle");
        currentState = SystemState::CYCLE_RUNNING;
    }
}

bool isStartSignalReceived() {
    //! Check if start signal has been received
    return (startButton.read() == HIGH || transferArmStartSignal.read() == HIGH);
}

bool isIdleComplete() {
    //! Check if idle state is complete (start signal received)
    return isStartSignalReceived();
}

void resetIdleState() {
    //! Reset idle state to initial conditions
    currentState = SystemState::READY;
} 