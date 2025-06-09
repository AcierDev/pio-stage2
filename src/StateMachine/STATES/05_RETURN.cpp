#include "StateMachine/STATES/05_RETURN.h"
#include "system_states.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ RETURN ***************************
//* ************************************************************************
// This state handles the return sequence to bring the stepper back
// to the home position after completing the cutting operation

void executeReturnState() {
    //! Execute complete return sequence
    executeReturnMovement();
}

void executeReturnMovement() {
    logMessage("🏠 Return to home phase...");
    
    // Signal transfer arm to prevent Z-axis lowering during return
    digitalWrite(Pins::TRANSFER_ARM_SIGNAL, HIGH);
    logMessage("Transfer arm signal activated (preventing Z-axis lowering)");

    // Fast return to slow-down point
    float currentPosition = stepper.currentPosition() / (float)Motion::STEPS_PER_INCH;
    float slowDownPosition = currentPosition * 0.01;

    stepper.setMaxSpeed(Motion::RETURN_SPEED);
    stepper.setAcceleration(Motion::RETURN_ACCEL);
    stepper.moveTo(slowDownPosition * Motion::STEPS_PER_INCH);

    unsigned long fastReturnStartTime = millis();
    unsigned long fastReturnTimeout = 15000;

    while (stepper.distanceToGo() != 0) {
        stepper.run();
        if (millis() - fastReturnStartTime > fastReturnTimeout) {
            logMessage("⚠ Fast return timeout - proceeding to slow approach...", "warn");
            break;
        }
    }

    // Slow approach to home position with home switch detection
    float slowHomingSpeed = Motion::HOMING_SPEED / 2;
    stepper.setMaxSpeed(slowHomingSpeed);
    stepper.setAcceleration(Motion::RETURN_ACCEL / 4);
    stepper.moveTo(-1000);  // Move towards home switch

    unsigned long slowApproachStartTime = millis();
    unsigned long slowApproachTimeout = 20000;
    bool homeFound = false;

    logMessage("Slow approach to home switch...");
    while (stepper.distanceToGo() != 0) {
        homeSwitch.update();
        
        if (homeSwitch.read() == HIGH) {
            stepper.stop();
            while (stepper.isRunning()) {
                stepper.run();
            }
            stepper.setCurrentPosition(0);  // Reset position when home switch triggers
            homeFound = true;
            logMessage("Home switch found during return. Position reset to 0.");
            break;
        }
        
        stepper.run();
        if (millis() - slowApproachStartTime > slowApproachTimeout) {
            logMessage("⚠ Slow approach timeout - stopping movement...", "warn");
            break;
        }
    }

    if (!homeFound) {
        logMessage("⚠ Warning: Home switch not found during return", "warn");
    }

    delay(30);

    // Move to home offset
    logMessage("Moving to home offset position: " + String(Motion::HOME_OFFSET) + " inches");
    logMessage("Current position before offset move: " + String(stepper.currentPosition() / (float)Motion::STEPS_PER_INCH) + " inches");
    
    stepper.setMaxSpeed(Motion::APPROACH_SPEED / 2);  // Use slower speed for offset move
    stepper.setAcceleration(Motion::FORWARD_ACCEL / 2);
    stepper.moveTo(Motion::HOME_OFFSET * Motion::STEPS_PER_INCH);
    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }
    
    logMessage("Final position after offset move: " + String(stepper.currentPosition() / (float)Motion::STEPS_PER_INCH) + " inches");
    
    // Deactivate transfer arm signal - return is complete
    digitalWrite(Pins::TRANSFER_ARM_SIGNAL, LOW);
    logMessage("Transfer arm signal deactivated (return complete)");

    delay(50);
    updateTransferArmStartSignalDebouncer();

    logMessage("✅ Return phase complete");
}

void executeEmptyClassReturn() {
    logMessage("🏠 Empty class return to home phase...");
    
    // Signal transfer arm to prevent Z-axis lowering during return
    digitalWrite(Pins::TRANSFER_ARM_SIGNAL, HIGH);
    logMessage("Transfer arm signal activated (preventing Z-axis lowering)");

    // Return directly to home switch first, then to offset
    stepper.setMaxSpeed(Motion::RETURN_SPEED);
    stepper.setAcceleration(Motion::RETURN_ACCEL);
    stepper.moveTo(-1000);  // Move towards home switch
    
    bool homeFound = false;
    unsigned long emptyReturnStart = millis();
    unsigned long emptyReturnTimeout = 15000;
    
    while (stepper.distanceToGo() != 0) {
        homeSwitch.update();
        
        if (homeSwitch.read() == HIGH) {
            stepper.stop();
            while (stepper.isRunning()) {
                stepper.run();
            }
            stepper.setCurrentPosition(0);
            homeFound = true;
            logMessage("Home switch found during empty return.");
            break;
        }
        
        stepper.run();
        if (millis() - emptyReturnStart > emptyReturnTimeout) {
            logMessage("⚠ Empty return timeout", "warn");
            break;
        }
    }
    
    if (homeFound) {
        // Move to home offset
        logMessage("Moving to home offset after empty return...");
        stepper.setMaxSpeed(Motion::APPROACH_SPEED / 2);
        stepper.setAcceleration(Motion::FORWARD_ACCEL / 2);
        stepper.moveTo(Motion::HOME_OFFSET * Motion::STEPS_PER_INCH);
        while (stepper.distanceToGo() != 0) {
            stepper.run();
        }
        logMessage("Empty return complete at offset position: " + String(stepper.currentPosition() / (float)Motion::STEPS_PER_INCH) + " inches");
    }
    
    // Deactivate transfer arm signal - return is complete
    digitalWrite(Pins::TRANSFER_ARM_SIGNAL, LOW);
    logMessage("Transfer arm signal deactivated (return complete)");

    delay(50);
    updateTransferArmStartSignalDebouncer();
    logMessage("✅ Empty class return complete");
}

bool isReturnComplete() {
    //! Check if return sequence is complete
    return !stepper.isRunning();
}

void resetReturnState() {
    //! Reset return state to initial conditions
    stepper.stop();
} 