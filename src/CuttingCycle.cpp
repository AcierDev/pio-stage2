#include <FastAccelStepper.h>
#include <Arduino.h>
#include "system_states.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ CUTTING CYCLE IMPLEMENTATION ***************************
//* ************************************************************************
// This module contains the complete cutting cycle implementation
// including motor movements, clamp sequences, and all phase management

// Forward declarations for helper functions
bool executeProgressiveMovement(float targetPosition, float maxSpeed, float acceleration, unsigned long timeoutMs);
bool executeControlledMovement(float targetPosition, float speed, float acceleration, unsigned long timeoutMs);
void executeClampSequence();
void releaseClamps();
void updateTransferArmStartSignalDebouncer();

//* ************************************************************************
//* ************************ MOTOR MOVEMENT HELPERS ***************************
//* ************************************************************************

bool executeProgressiveMovement(float targetPosition, float maxSpeed, float acceleration, unsigned long timeoutMs) {
  //! Execute movement with progressive speed control to handle high speeds
  if (!stepper) return false;
  
  float currentPos = stepper->getCurrentPosition() / Motion::STEPS_PER_INCH;
  float distance = abs(targetPosition - currentPos);
  
  // For very short distances, use direct movement
  if (distance < 1.0) {
    return executeControlledMovement(targetPosition, maxSpeed / 4, acceleration / 2, timeoutMs);
  }
  
  // Calculate progressive speeds based on distance
  float startSpeed = maxSpeed / 8;  // Start at 1/8 max speed
  float midSpeed = maxSpeed / 2;    // Mid at 1/2 max speed
  float endSpeed = maxSpeed / 4;    // End at 1/4 max speed
  
  // Phase 1: Start with lower speed
  stepper->setSpeedInHz(startSpeed);
  stepper->setAcceleration(acceleration / 4);
  stepper->moveTo(targetPosition * Motion::STEPS_PER_INCH);
  
  unsigned long startTime = millis();
  bool phaseComplete = false;
  
  // Monitor movement and increase speed progressively
  while (stepper->isRunning()) {
    if (millis() - startTime > timeoutMs) {
      stepper->forceStop();
      Serial.println("Progressive movement TIMEOUT!");
      return false;
    }
    
    float currentMovementPos = stepper->getCurrentPosition() / Motion::STEPS_PER_INCH;
    float progressPercent = abs(currentMovementPos - currentPos) / distance;
    
    // Increase speed as we progress
    if (progressPercent > 0.2 && progressPercent < 0.8 && !phaseComplete) {
      stepper->setSpeedInHz(midSpeed);
      phaseComplete = true;
    } else if (progressPercent > 0.8) {
      stepper->setSpeedInHz(endSpeed);
    }
    
    delay(10); // Small delay for monitoring
  }
  
  return true;
}

bool executeControlledMovement(float targetPosition, float speed, float acceleration, unsigned long timeoutMs) {
  //! Execute movement with controlled speed and proper timeout handling
  if (!stepper) return false;
  
  stepper->setSpeedInHz(speed);
  stepper->setAcceleration(acceleration);
  stepper->moveTo(targetPosition * Motion::STEPS_PER_INCH);
  
  unsigned long startTime = millis();
  
  while (stepper->isRunning()) {
    if (millis() - startTime > timeoutMs) {
      stepper->forceStop();
      Serial.println("Controlled movement TIMEOUT!");
      return false;
    }
    delay(1);
  }
  
  return true;
}

//* ************************************************************************
//* ************************ CLAMP CONTROL FUNCTIONS ***************************
//* ************************************************************************

void executeClampSequence() {
  //! Execute the complete clamp sequence for material alignment
  
  // Initial left clamp pulse and alignment cylinder extension
  digitalWrite(Pins::LEFT_CLAMP, LOW);  // Engage (extend) left clamp
  delay(200);
  digitalWrite(Pins::ALIGN_CYLINDER, HIGH);  // Extend alignment cylinder
  delay(100);
  digitalWrite(Pins::LEFT_CLAMP, HIGH);      // Retract left clamp
  delay(100);

  digitalWrite(Pins::ALIGN_CYLINDER, LOW);  // Retract alignment cylinder
  digitalWrite(Pins::RIGHT_CLAMP, LOW);  // Extend right clamp
  delay(150);
  digitalWrite(Pins::RIGHT_CLAMP, HIGH);  // Retract right clamp
  digitalWrite(Pins::ALIGN_CYLINDER, HIGH);  // Extend alignment cylinder
  delay(150);
  digitalWrite(Pins::ALIGN_CYLINDER, LOW);  // Retract alignment cylinder
  delay(125);

  // Engage both clamps for cutting
  digitalWrite(Pins::RIGHT_CLAMP, HIGH);  // Retract right clamp
  delay(200);
  digitalWrite(Pins::LEFT_CLAMP, LOW);  // Extend left clamp
  digitalWrite(Pins::RIGHT_CLAMP, LOW);  // Extend right clamp
}

void releaseClamps() {
  //! Release both clamps simultaneously
  digitalWrite(Pins::LEFT_CLAMP, HIGH);   // Release left clamp
  digitalWrite(Pins::RIGHT_CLAMP, HIGH);  // Release right clamp
}

void updateTransferArmStartSignalDebouncer() {
  //! Force update the debouncer to capture the current state
  // This is crucial for detecting the next signal change
  for (int i = 0; i < 5; i++) {  // Multiple updates to ensure proper state capture
    transferArmStartSignal.update();
    delay(10);
  }
}

//* ************************************************************************
//* ************************ MAIN CUTTING CYCLE ***************************
//* ************************************************************************

void runCuttingCycle() {
  if (!stepper) return; // Safety check

  // Reset analysis result tracking at the start of each cycle
  lastDetectedClass = "";
  analysisResultReceived = false;

  Serial.println("=== CUTTING CYCLE START ===");

  //* ************************************************************************
  //* ************************ CLAMP SEQUENCE ***************************
  //* ************************************************************************
  
  executeClampSequence();

  //* ************************************************************************
  //* ************************ APPROACH PHASE ***************************
  //* ************************************************************************
  
  Serial.println("=== APPROACH PHASE ===");
  float currentPos = stepper->getCurrentPosition() / Motion::STEPS_PER_INCH;
  float targetPos = Motion::APPROACH_DISTANCE;
  Serial.println("Moving from " + String(currentPos) + "\" to " + String(targetPos) + "\"");
  
  // Use progressive speed approach for high-speed movement
  if (!executeProgressiveMovement(targetPos, Motion::APPROACH_SPEED, Motion::FORWARD_ACCEL, 10000)) {
    Serial.println("APPROACH PHASE FAILED!");
    return;
  }
  
  Serial.println("Approach complete. Position: " + String(stepper->getCurrentPosition() / Motion::STEPS_PER_INCH) + " inches");

  //* ************************************************************************
  //* ************************ CUTTING PHASE ***************************
  //* ************************************************************************
  
  Serial.println("=== CUTTING PHASE ===");
  float cuttingTarget = Motion::APPROACH_DISTANCE + Motion::CUTTING_DISTANCE;
  Serial.println("Cutting " + String(Motion::CUTTING_DISTANCE) + "\" to position " + String(cuttingTarget) + "\"");
  
  // Use slow, controlled movement for cutting
  if (!executeControlledMovement(cuttingTarget, Motion::CUTTING_SPEED, Motion::FORWARD_ACCEL / 4, 20000)) {
    Serial.println("CUTTING PHASE FAILED!");
    return;
  }
  
  Serial.println("Cutting complete. Position: " + String(stepper->getCurrentPosition() / Motion::STEPS_PER_INCH) + " inches");

  //* ************************************************************************
  //* ************************ FINISH PHASE ***************************
  //* ************************************************************************
  
  Serial.println("=== FINISH PHASE ===");
  Serial.println("Moving to final position " + String(Motion::FORWARD_DISTANCE) + "\"");
  
  // Use progressive speed for finish movement
  if (!executeProgressiveMovement(Motion::FORWARD_DISTANCE, Motion::FINISH_SPEED, Motion::FORWARD_ACCEL, 15000)) {
    Serial.println("FINISH PHASE FAILED!");
    return;
  }

  stepper->forceStop();
  delay(50);

  //* ************************************************************************
  //* ************************ CLAMP RELEASE ***************************
  //* ************************************************************************
  
  // Release both clamps simultaneously
  releaseClamps();
  delay(100);

  //* ************************************************************************
  //* ************************ RETURN PHASE ***************************
  //* ************************************************************************
  
  Serial.println("=== RETURN PHASE ===");
  // Signal transfer arm to prevent Z-axis lowering during return
  digitalWrite(Pins::TRANSFER_ARM_SIGNAL, HIGH);

  // Fast return with progressive speed control
  float currentPosition = stepper->getCurrentPosition() / Motion::STEPS_PER_INCH;
  float slowDownPosition = currentPosition * 0.1;  // 10% of current position for slowdown
  
  Serial.println("Fast return from " + String(currentPosition) + "\" to " + String(slowDownPosition) + "\"");

  if (!executeProgressiveMovement(slowDownPosition, Motion::RETURN_SPEED, Motion::RETURN_ACCEL, 15000)) {
    Serial.println("FAST RETURN FAILED!");
  }
  
  Serial.println("Fast return complete. Position: " + String(stepper->getCurrentPosition() / Motion::STEPS_PER_INCH) + " inches");

  //* ************************************************************************
  //* ************************ SLOW RETURN TO HOME ***************************
  //* ************************************************************************
  
  Serial.println("=== SLOW RETURN TO HOME ===");
  
  if (!executeControlledMovement(0.0, Motion::HOMING_SPEED / 2, Motion::RETURN_ACCEL / 4, 20000)) {
    Serial.println("SLOW RETURN FAILED!");
  }
  
  Serial.println("Slow return complete. Position: " + String(stepper->getCurrentPosition() / Motion::STEPS_PER_INCH) + " inches");

  delay(30);

  //* ************************************************************************
  //* ************************ HOME OFFSET ***************************
  //* ************************************************************************
  
  Serial.println("=== MOVE TO HOME OFFSET ===");
  Serial.println("Home offset target: " + String(Motion::HOME_OFFSET) + " inches");
  
  if (!executeControlledMovement(Motion::HOME_OFFSET, Motion::APPROACH_SPEED / 4, Motion::FORWARD_ACCEL / 2, 10000)) {
    Serial.println("HOME OFFSET FAILED!");
  }
  
  Serial.println("Home offset complete. Final position: " + String(stepper->getCurrentPosition() / Motion::STEPS_PER_INCH) + " inches");
  
  // Deactivate transfer arm signal - return is complete
  digitalWrite(Pins::TRANSFER_ARM_SIGNAL, LOW);
  Serial.println("=== CUTTING CYCLE COMPLETE ===");

  delay(50);
  updateTransferArmStartSignalDebouncer();
} 