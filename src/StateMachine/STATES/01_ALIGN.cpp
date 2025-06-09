#include "StateMachine/STATES/01_ALIGN.h"
#include "system_states.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ ALIGN ***************************
//* ************************************************************************
// This state handles the initial clamp movements and alignment cylinder
// operations before motor movement begins

void executeAlignState() {
    //! Execute complete alignment sequence including clamps and analysis
    logMessage("🔧 Starting alignment sequence...");
    
    // Reset analysis result tracking
    lastDetectedClass = "";
    analysisResultReceived = false;
    
    executeAlignmentSequence();
    executeAnalysisSequence();
    
    logMessage("✅ Alignment sequence complete");
}

void executeAlignmentSequence() {
    logMessage("🔧 Executing clamp alignment sequence...");
    
    // Initial left clamp pulse and alignment cylinder extension
    digitalWrite(Pins::LEFT_CLAMP, LOW);  // Engage (extend) left clamp

    // Left clamp only extends for 300ms (200 ms + 100 ms delay)
    delay(200);
    digitalWrite(Pins::ALIGN_CYLINDER, HIGH);  // Extend alignment cylinder
    delay(100);
    digitalWrite(Pins::LEFT_CLAMP, HIGH);      // Retract left clamp

    // Alignment cylinder stays extended for the remainder of the time
    delay(100);

    digitalWrite(Pins::ALIGN_CYLINDER, LOW);  // Retract alignment cylinder
    digitalWrite(Pins::RIGHT_CLAMP, LOW);  // Extend right clamp
    delay(150);
    digitalWrite(Pins::RIGHT_CLAMP, HIGH);  // Retract right clamp
    digitalWrite(Pins::ALIGN_CYLINDER, HIGH);  // Extend alignment cylinder
    delay(150);
    digitalWrite(Pins::ALIGN_CYLINDER, LOW);  // Retract alignment cylinder
    delay(125);

    // Engage clamps
    digitalWrite(Pins::RIGHT_CLAMP, HIGH);  // Retract right clamp
    delay(200);
    digitalWrite(Pins::LEFT_CLAMP, LOW);  // Extend left clamp
    digitalWrite(Pins::RIGHT_CLAMP, LOW);  // Extend right clamp
    
    logMessage("✅ Clamp alignment complete");
}

void executeAnalysisSequence() {
    logMessage("📷 Starting wood analysis...");
    
    // Check camera signal using debounced value
    if (cameraSignal.read() == HIGH) {
        // Send burst request to camera system
        sendSerialMessage("BURST");
        inferenceStartTime = millis();
        inferenceTimingActive = true;

        // Wait for analysis result (max 2 seconds)
        unsigned long startWaitTime = millis();
        unsigned long waitTimeout = 2000;
        logMessage("Waiting for wood analysis result...");

        while (!analysisResultReceived && (millis() - startWaitTime < waitTimeout)) {
            // Check for and process any available serial responses
            if (Serial.available()) {
                String response = Serial.readStringUntil('\n');
                response.trim();
                if (response.length() > 0 && response.startsWith("{")) {
                    handleSerialResponse(response);
                }
            }
            delay(10);
        }

        // Handle timeout case for inference timing
        if (!analysisResultReceived && inferenceTimingActive) {
            unsigned long timeoutDuration = millis() - inferenceStartTime;
            logMessage("Analysis timeout after " + String(timeoutDuration) + " ms", "warn");
            inferenceTimingActive = false;
        }
    } else {
        logMessage("Camera not ready (signal LOW)");
    }
    
    logMessage("✅ Analysis sequence complete");
}

bool isAlignComplete() {
    //! Check if alignment sequence is complete
    return true; // Alignment sequence is synchronous
}

bool isEmptyClassDetected() {
    //! Check if empty class was detected
    return analysisResultReceived && lastDetectedClass.equalsIgnoreCase("Empty");
}

bool isEndClassDetected() {
    //! Check if end class was detected
    return analysisResultReceived && lastDetectedClass.equalsIgnoreCase("End");
}

void resetAlignState() {
    //! Reset alignment state to initial conditions
    lastDetectedClass = "";
    analysisResultReceived = false;
    inferenceTimingActive = false;
} 