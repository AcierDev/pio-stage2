#include "StateMachine/FUNCTIONS/SignalControl.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"

//* ************************************************************************
//* ****************** SIGNAL CONTROL FUNCTIONS ********************
//* ************************************************************************
// This module contains all functions related to digital I/O signal control
// including transfer arm signals and other digital outputs

void activateTransferArmSignal() {
    //! Activate transfer arm signal to prevent Z-axis lowering
    digitalWrite(Pins::TRANSFER_ARM_SIGNAL, HIGH);
}

void deactivateTransferArmSignal() {
    //! Deactivate transfer arm signal - return is complete
    digitalWrite(Pins::TRANSFER_ARM_SIGNAL, LOW);
    delay(50);
} 