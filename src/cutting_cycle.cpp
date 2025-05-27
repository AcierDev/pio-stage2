#include "cutting_cycle.h"

//* ************************************************************************
//* ************************ CUTTING CYCLE ****************************
//* ************************************************************************
// This file contains the main cutting cycle logic and related functions
// for the Stage 2 cutting controller. The cycle includes wood analysis,
// positioning, cutting operations, and return to home.

void runCuttingCycle() {
  // Reset analysis result tracking at the start of each cycle
  lastDetectedClass = "";
  analysisResultReceived = false;

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

  // Check camera signal using debounced value
  if (cameraSignal.read() == HIGH) {
    // Send burst request to webcam endpoint
    sendBurstRequest();

    // Wait for analysis result (max 2 seconds)
    unsigned long startWaitTime = millis();
    unsigned long waitTimeout = 2000;  // Increased from 700ms to 2000ms
    logMessage("Waiting for wood analysis result...");

    while (!analysisResultReceived &&
           (millis() - startWaitTime < waitTimeout)) {
      // Check for and process any available serial responses
      if (Serial.available()) {
        String response = Serial.readStringUntil('\n');
        response.trim();
        if (response.length() > 0 && response.startsWith("{")) {
          handleSerialResponse(response);  // Process JSON responses during wait
        }
      }
      delay(10);  // Smaller delay for more responsive checking
    }

    // Handle timeout case for inference timing
    if (!analysisResultReceived && inferenceTimingActive) {
      unsigned long timeoutDuration = millis() - inferenceStartTime;
      logMessage("Analysis timeout after " + String(timeoutDuration) + " ms",
                 "warn");
      inferenceTimingActive = false;
    }

    // Check if we received analysis result and it's "Empty"
    if (analysisResultReceived && lastDetectedClass.equalsIgnoreCase("Empty")) {
      logMessage("======= SHORT-CIRCUITING CYCLE =======");
      logMessage("Empty wood detected - skipping cutting operations");
      logMessage("========================================");

      // Release clamps and return to home position
      releaseClamps();

      // Signal transfer arm to prevent Z-axis lowering during return
      digitalWrite(Pins::TRANSFER_ARM_SIGNAL, HIGH);
      logMessage("Transfer arm signal activated (preventing Z-axis lowering)");

      // Return directly to home
      moveStepperToPosition(Motion::HOME_OFFSET, Motion::RETURN_SPEED,
                            Motion::RETURN_ACCEL);
      
      // Deactivate transfer arm signal - return is complete
      digitalWrite(Pins::TRANSFER_ARM_SIGNAL, LOW);
      logMessage("Transfer arm signal deactivated (return complete)");

      // At the end of the cycle, add a small delay
      delay(50);

      // Force update the debouncer to capture the current state
      updateTransferArmStartSignalDebouncer();

      logMessage("✅ Cycle short-circuited and completed!");
      return;  // Exit the function early
    }
  } else {
    logMessage("Camera not ready (signal LOW)");
  }

  // Approach phase
  logMessage("🚀 Approach phase...");
  moveStepperToPosition(Motion::APPROACH_DISTANCE, Motion::APPROACH_SPEED,
                        Motion::FORWARD_ACCEL);

  // Cutting phase
  logMessage("🔪 Cutting phase...");
  moveStepperToPosition(Motion::APPROACH_DISTANCE + Motion::CUTTING_DISTANCE,
                        Motion::CUTTING_SPEED, Motion::FORWARD_ACCEL * 2);

  // Handle "End" class detection
  if (analysisResultReceived && lastDetectedClass.equalsIgnoreCase("End")) {
    logMessage("======= END CLASS DETECTED =======");
    float intermediatePosition =
        Motion::FORWARD_DISTANCE - Motion::END_DROP_DISTANCE_OFFSET;
    logMessage("Moving to intermediate position: " +
               String(intermediatePosition));

    // Move to the intermediate position
    moveStepperToPosition(intermediatePosition, Motion::FINISH_SPEED,
                          Motion::FORWARD_ACCEL);
    stepper.stop();  // Ensure motor has completely stopped
    delay(Timing::MOTION_SETTLE_TIME);

    logMessage("Deactivating left clamp...");
    digitalWrite(Pins::LEFT_CLAMP, HIGH);  // Deactivate left clamp
    delay(200);                            // Wait for 200ms

    logMessage("Proceeding to final forward distance.");
    logMessage("==================================");
  }

  // Finish phase
  logMessage("🏁 Finish phase...");
  moveStepperToPosition(Motion::FORWARD_DISTANCE, Motion::FINISH_SPEED,
                        Motion::FORWARD_ACCEL);

  // Ensure motor has completely stopped
  stepper.stop();
  delay(50);

  // Verify position before releasing clamps
  float finalPosition =
      stepper.currentPosition() / (float)Motion::STEPS_PER_INCH;
  if (abs(finalPosition - Motion::FORWARD_DISTANCE) >
      0.1) {  // If more than 0.1 inches off
    logMessage("⚠ Position error at forward position!", "warn");
    // Try to correct position
    moveStepperToPosition(Motion::FORWARD_DISTANCE, Motion::CUTTING_SPEED,
                          Motion::FORWARD_ACCEL);
    stepper.stop();
    delay(50);
  }

  // Store the position before releasing clamps
  long positionBeforeRelease = stepper.currentPosition();

  // Release both clamps simultaneously
  releaseClamps();

  // Add extra settle time after clamp release
  delay(100);

  // Verify position hasn't changed significantly after clamp release
  if (abs(stepper.currentPosition() - positionBeforeRelease) >
      Motion::STEPS_PER_INCH /
          4) {  // If position changed by more than 1/4 inch
    logMessage("⚠ Position shifted during clamp release!", "warn");
    // Try to correct position before return
    moveStepperToPosition(Motion::FORWARD_DISTANCE, Motion::CUTTING_SPEED,
                          Motion::FORWARD_ACCEL);
    stepper.stop();
    delay(50);
  }

  // --- Updated Return Phase (Home Detection removed) ---
  // Return phase without checking for premature home switch activation
  logMessage("🏠 Return to home phase...");
  
  // Signal transfer arm to prevent Z-axis lowering during return
  digitalWrite(Pins::TRANSFER_ARM_SIGNAL, HIGH);
  logMessage("Transfer arm signal activated (preventing Z-axis lowering)");

  // First calculate current position and determine a slow-down point
  float currentPosition =
      stepper.currentPosition() / (float)Motion::STEPS_PER_INCH;
  float slowDownPosition =
      currentPosition *
      0.01;  // Slow down at 1% of the way back (99% of the way there)

  // Fast return: move quickly to the slow-down point
  stepper.setMaxSpeed(Motion::RETURN_SPEED);
  stepper.setAcceleration(Motion::RETURN_ACCEL);
  stepper.moveTo(slowDownPosition * Motion::STEPS_PER_INCH);

  unsigned long fastReturnStartTime = millis();
  unsigned long fastReturnTimeout = 15000;  // 15 seconds timeout

  while (stepper.distanceToGo() != 0) {
    // Removed home switch check here
    stepper.run();
    if (millis() - fastReturnStartTime > fastReturnTimeout) {
      logMessage("⚠ Fast return timeout - proceeding to slow approach...",
                 "warn");
      break;
    }
  }

  // Slow approach phase: move slowly to home position
  float slowHomingSpeed = Motion::HOMING_SPEED / 2;  // Half of homing speed
  stepper.setMaxSpeed(slowHomingSpeed);
  stepper.setAcceleration(Motion::RETURN_ACCEL / 4);  // Gentler acceleration
  stepper.moveTo(0);  // Move toward home (position 0)

  unsigned long slowApproachStartTime = millis();
  unsigned long slowApproachTimeout = 20000;  // 20 seconds timeout

  while (stepper.distanceToGo() != 0) {
    // Removed home switch check here as well
    stepper.run();
    if (millis() - slowApproachStartTime > slowApproachTimeout) {
      logMessage("⚠ Slow approach timeout - stopping movement...", "warn");
      break;
    }
  }
  // --- End of Updated Return Phase ---

  delay(30);

  // Move to home offset
  logMessage("Moving to home offset position...");
  moveStepperToPosition(Motion::HOME_OFFSET, Motion::APPROACH_SPEED,
                        Motion::FORWARD_ACCEL);
  
  // Deactivate transfer arm signal - return is complete
  digitalWrite(Pins::TRANSFER_ARM_SIGNAL, LOW);
  logMessage("Transfer arm signal deactivated (return complete)");

  // At the end of the cycle, add a small delay to ensure the machine start
  // signal can be properly detected for the next cycle
  delay(50);  // Small delay to allow signal to stabilize

  // Force update the debouncer to capture the current state
  updateTransferArmStartSignalDebouncer();

  logMessage("✅ Cycle complete!");
} 