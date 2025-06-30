#include <FastAccelStepper.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Bounce2.h>

#include "system_states.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"
#include "StateMachine/STATES/07_CUTTING_CYCLE.h"
#include "StateMachine/FUNCTIONS/MotionControl.h"
#include "StateMachine/FUNCTIONS/PneumaticControl.h"
#include "OTA_Manager.h"

// System state (defined in system_states.h)

// Global objects
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
Bounce homeSwitch = Bounce();
Bounce startButton = Bounce();
Bounce transferArmStartSignal = Bounce();  // Transfer arm start signal

// System state tracking
SystemState currentState = SystemState::INITIALIZING;
bool isHomed = false;

// Analysis result tracking
String lastDetectedClass = "";  // Will store the last detected wood class
bool analysisResultReceived = false;  // Flag to indicate if we've received an analysis result

// Function declarations
void initializeHardware();
void performHomingSequence();
void runCuttingCycle();
void handleSerialCommand(const String &command);
void printCurrentSettings();
void printSystemStatus();
void engageClamps();
void releaseClamps();
void staggeredReleaseClamps();

void handleSerialResponse(const String &response);  // New function to handle serial responses
void sendSerialMessage(const String &message);  // New function to send serial messages

// Manual Control Functions
void manualHome();
void manualJog(bool jogLeft, float distance);
void manualToggleLeftClamp();
void manualToggleRightClamp();
void manualToggleAlignCylinder();
void manualStartCycle();
void updateTransferArmStartSignalDebouncer();

void setup() {
  Serial.begin(SERIAL_BAUDRATE);

  // Initialize FastAccelStepper engine
  engine.init();
  stepper = engine.stepperConnectToPin(Pins::STEP);
  if (stepper) {
    stepper->setDirectionPin(Pins::DIR);
    stepper->setEnablePin(Pins::ENABLE);
    stepper->setAutoEnable(true);
  }

  // Initialize OTA functionality
  initOTA();

  // Initialize hardware and perform homing sequence
  initializeHardware();
  performHomingSequence();

  currentState = SystemState::READY;
  printCurrentSettings();
}

void loop() {
  // Handle OTA updates
  handleOTA();

  // Handle serial communication
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      // Check if this is a JSON command from Python or a plain text command
      if (command.startsWith("{")) {
        // Handle JSON commands from Python (like burst responses)
        handleSerialResponse(command);
      } else {
        // Handle plain text commands (manual serial commands)
        handleSerialCommand(command);
      }
    }
  }

  // Update button states
  homeSwitch.update();
  startButton.update();
  transferArmStartSignal.update();

  // Check for cycle start (either from button or machine start signal)
  if (startButton.fell() && currentState == SystemState::READY) {
    currentState = SystemState::CYCLE_RUNNING;

    runCuttingCycle();
    updateTransferArmStartSignalDebouncer();
    currentState = SystemState::READY;
  }

  if (transferArmStartSignal.read() == HIGH && currentState == SystemState::READY) {
    currentState = SystemState::CYCLE_RUNNING;

    runCuttingCycle();
    updateTransferArmStartSignalDebouncer();
    currentState = SystemState::READY;
  }
}

void initializeHardware() {
  // Configure input pins
  pinMode(Pins::HOME_SWITCH, INPUT_PULLDOWN);
  pinMode(Pins::START_BUTTON, INPUT_PULLDOWN);
  pinMode(Pins::TRANSFER_ARM_START_SIGNAL, INPUT_PULLDOWN);  // Pin 15

  // Configure output pins
  pinMode(Pins::ENABLE, OUTPUT);
  pinMode(Pins::LEFT_CLAMP, OUTPUT);
  pinMode(Pins::RIGHT_CLAMP, OUTPUT);
  pinMode(Pins::ALIGN_CYLINDER, OUTPUT);
  pinMode(Pins::TRANSFER_ARM_SIGNAL, OUTPUT);

  // Initialize clamps to engaged state (extended)
  digitalWrite(Pins::LEFT_CLAMP, LOW);   // Start with clamps engaged
  digitalWrite(Pins::RIGHT_CLAMP, LOW);  // Start with clamps engaged

  // Initialize alignment cylinder to retracted position
  digitalWrite(Pins::ALIGN_CYLINDER, LOW);
  
  // Initialize transfer arm signal to LOW (not returning)
  digitalWrite(Pins::TRANSFER_ARM_SIGNAL, LOW);

  // Setup debouncing
  homeSwitch.attach(Pins::HOME_SWITCH);
  homeSwitch.interval(10);
  startButton.attach(Pins::START_BUTTON);
  startButton.interval(20);
  transferArmStartSignal.attach(Pins::TRANSFER_ARM_START_SIGNAL);
  transferArmStartSignal.interval(50);  // 50ms debounce for transfer arm start signal

  // Initialize stepper with FastAccelStepper
  if (stepper) {
    stepper->setSpeedInHz(Motion::APPROACH_SPEED);
    stepper->setAcceleration(Motion::FORWARD_ACCEL);
  }
}

void performHomingSequence() {
  currentState = SystemState::HOMING;

  if (!stepper) return; // Safety check

  // Clamps are already engaged from initialization

  // First, move a significant distance in the negative direction to ensure
  // we're past the home switch
  stepper->setSpeedInHz(Motion::HOMING_SPEED);
  stepper->setAcceleration(Motion::FORWARD_ACCEL);
  stepper->move(-10000);  // Move 10,000 steps in negative direction

  // Use a much slower approach speed for final homing
  float slowHomingSpeed = Motion::HOMING_SPEED / 3;  // One-third of normal homing speed

  // Run until we hit the home switch or reach the target
  while (stepper->isRunning()) {
    homeSwitch.update();

    // If we're within 2000 steps of where we think home might be, slow down significantly
    if (abs(stepper->getCurrentPosition()) < 2000) {
      stepper->setSpeedInHz(slowHomingSpeed);
    }

    if (homeSwitch.read() == HIGH) {
      // When home switch is triggered, stop immediately
      stepper->forceStopAndNewPosition(0);
      break;
    }
    delay(1); // Small delay to prevent watchdog issues
  }

  // If we didn't hit the home switch, we have a problem
  if (homeSwitch.read() == LOW) {
    currentState = SystemState::ERROR;
    return;
  }

  // Now move to home offset with a gentler motion
  stepper->setSpeedInHz(Motion::HOMING_SPEED / 2);  // Half speed for moving to offset
  stepper->setAcceleration(Motion::FORWARD_ACCEL / 2);  // Gentler acceleration
  stepper->moveTo(Motion::HOME_OFFSET * Motion::STEPS_PER_INCH);
  
  while (stepper->isRunning()) {
    delay(1);
  }
  
  // Add settle time after reaching home offset
  delay(Timing::HOME_SETTLE_TIME);

  // Now that we're at home position, release the clamps
  digitalWrite(Pins::LEFT_CLAMP, HIGH);
  digitalWrite(Pins::RIGHT_CLAMP, HIGH);
  
  // Add settle time after releasing clamps
  delay(Timing::CLAMP_RELEASE_TIME);

  isHomed = true;
}

void engageClamps() {
  digitalWrite(Pins::LEFT_CLAMP, LOW);
  digitalWrite(Pins::RIGHT_CLAMP, LOW);
  delay(200);
}

void releaseClamps() {
  digitalWrite(Pins::LEFT_CLAMP, HIGH);
  digitalWrite(Pins::RIGHT_CLAMP, HIGH);
  delay(200);
}

void staggeredReleaseClamps() {
  digitalWrite(Pins::RIGHT_CLAMP, HIGH);  // Release right clamp first
  delay(200);
  digitalWrite(Pins::LEFT_CLAMP, HIGH);  // Release left clamp
  delay(200);
}

void handleSerialCommand(const String &command) {
  // Check for JSON commands first
  if (command.startsWith("{")) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, command);

    if (!error) {
      const char *cmd = doc["command"];
      if (cmd && strcmp(cmd, "identify") == 0) {
        // Send board identification
        JsonDocument response;
        response["board_id"] = Config::BOARD_ID;
        response["description"] = Config::BOARD_DESCRIPTION;
        response["type"] = "STAGE_2";

        String jsonResponse;
        serializeJson(response, jsonResponse);
        sendSerialMessage(jsonResponse);
        return;
      }
    }
  }

  // Handle plain text commands
  if (command == "status") {
    printSystemStatus();
  } else if (command == "home") {
    performHomingSequence();
  } else if (command == "settings") {
    printCurrentSettings();
  } else if (command == "identify") {
    // Plain text identification response
    sendSerialMessage("BOARD_ID:" + String(Config::BOARD_ID));
  }
}

void printSystemStatus() {
  // Print state
  String stateStr = "UNKNOWN";
  switch (currentState) {
    case SystemState::INITIALIZING:
      stateStr = "INITIALIZING";
      break;
    case SystemState::HOMING:
      stateStr = "HOMING";
      break;
    case SystemState::READY:
      stateStr = "READY";
      break;
    case SystemState::CYCLE_RUNNING:
      stateStr = "CYCLE RUNNING";
      break;
    case SystemState::ERROR:
      stateStr = "ERROR";
      break;
  }

  Serial.println("System State: " + stateStr);
  if (stepper) {
    Serial.println("Position: " + String(stepper->getCurrentPosition() / Motion::STEPS_PER_INCH) + " inches");
  }
  Serial.println("Home Switch: " + String(homeSwitch.read() ? "TRIGGERED" : "NOT TRIGGERED"));
  Serial.println("Start Button: " + String(startButton.read() ? "PRESSED" : "NOT PRESSED"));
  Serial.println("Transfer Arm Start Signal: " + String(transferArmStartSignal.read() ? "TRIGGERED" : "NOT TRIGGERED"));
  Serial.println("Left Clamp: " + String(digitalRead(Pins::LEFT_CLAMP) ? "RELEASED" : "ENGAGED"));
  Serial.println("Right Clamp: " + String(digitalRead(Pins::RIGHT_CLAMP) ? "RELEASED" : "ENGAGED"));
  Serial.println("Alignment Cylinder: " + String(digitalRead(Pins::ALIGN_CYLINDER) ? "EXTENDED" : "RETRACTED"));
  Serial.println("Transfer Arm Signal: " + String(digitalRead(Pins::TRANSFER_ARM_SIGNAL) ? "ACTIVE (Z-BLOCKED)" : "INACTIVE"));
}

void printCurrentSettings() {
  Serial.println("\nMotion Parameters:");
  Serial.println("- Steps per inch: " + String(Motion::STEPS_PER_INCH));
  Serial.println("- Home offset: " + String(Motion::HOME_OFFSET));
  Serial.println("- Approach distance: " + String(Motion::APPROACH_DISTANCE));
  Serial.println("- Cutting distance: " + String(Motion::CUTTING_DISTANCE));
  Serial.println("- Forward distance: " + String(Motion::FORWARD_DISTANCE));

  Serial.println("\nSpeed Settings (steps/sec):");
  Serial.println("- Homing: " + String(Motion::HOMING_SPEED));
  Serial.println("- Approach: " + String(Motion::APPROACH_SPEED));
  Serial.println("- Cutting: " + String(Motion::CUTTING_SPEED));
  Serial.println("- Finish: " + String(Motion::FINISH_SPEED));
  Serial.println("- Return: " + String(Motion::RETURN_SPEED));

  Serial.println("\nAcceleration Settings (steps/sec²):");
  Serial.println("- Forward: " + String(Motion::FORWARD_ACCEL));
  Serial.println("- Return: " + String(Motion::RETURN_ACCEL));

  Serial.println("\nTiming Settings (ms):");
  Serial.println("- Clamp engage time: " + String(Timing::CLAMP_ENGAGE_TIME));
  Serial.println("- Clamp release time: " + String(Timing::CLAMP_RELEASE_TIME));
  Serial.println("- Home settle time: " + String(Timing::HOME_SETTLE_TIME));
  Serial.println("- Motion settle time: " + String(Timing::MOTION_SETTLE_TIME));
}

// Function to handle serial responses
void handleSerialResponse(const String &response) {
  // This function handles JSON responses from Python
  // Try to parse JSON response
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, response);

  if (!error) {
    // Handle JSON responses from Python
    if (doc["status"].is<String>()) {
      String status = doc["status"].as<String>();

      if (status == "success" && doc["burst_complete"].is<String>()) {
        String result = doc["burst_complete"].as<String>();

        // Check for analysis results
        if (doc["analysis_result"].is<JsonObject>()) {
          // Extract the analysis results
          JsonObject analysis = doc["analysis_result"];

          if (analysis["class"].is<String>()) {
            String detectedClass = analysis["class"].as<String>();
            float confidence = 0.0;

            if (analysis["confidence"].is<float>()) {
              confidence = analysis["confidence"].as<float>();
            }

            // Update analysis result tracking
            lastDetectedClass = detectedClass;
            analysisResultReceived = true;
          } else if (analysis["error"].is<String>()) {
            // Handle error in analysis
            String errorMsg = analysis["error"].as<String>();

            // Reset analysis result tracking on error
            lastDetectedClass = "";
            analysisResultReceived = false;
          }
        }
      } else if (status == "error" && doc["message"].is<String>()) {
        String errorMsg = doc["message"].as<String>();

        // Reset analysis result tracking on error
        lastDetectedClass = "";
        analysisResultReceived = false;
      }
    }
  } else {
    // Reset analysis result tracking on error
    lastDetectedClass = "";
    analysisResultReceived = false;
  }
}

// Function to send serial messages
void sendSerialMessage(const String &message) {
  Serial.println(message);
}

// Manual Control Function Implementations
void manualHome() {
  if (currentState == SystemState::READY) {
    performHomingSequence();
    if (isHomed) {
      currentState = SystemState::READY;  // Should be set by performHomingSequence if successful
    }
  }
}

void manualJog(bool jogLeft, float distance) {
  if (currentState == SystemState::READY && stepper) {
    if (distance <= 0 || distance > 5.0) {  // Basic validation for jog distance
      return;
    }
    float currentPosInches = stepper->getCurrentPosition() / (float)Motion::STEPS_PER_INCH;
    float targetPosInches;
    if (jogLeft) {
      targetPosInches = currentPosInches - distance;
      // Prevent jogging beyond a safe minimum (e.g., slightly before 0)
      if (targetPosInches < -0.1) targetPosInches = -0.1;
    } else {
      targetPosInches = currentPosInches + distance;
      // Prevent jogging beyond a safe maximum
      if (targetPosInches > (Motion::FORWARD_DISTANCE + 10.0))
        targetPosInches = Motion::FORWARD_DISTANCE + 10.0;
    }

    // Use a moderate speed and acceleration for jogging
    moveStepperToPosition(targetPosInches, Motion::APPROACH_SPEED / 2, Motion::FORWARD_ACCEL / 2);
  }
}

void manualToggleLeftClamp() {
  if (currentState == SystemState::READY) {
    bool currentLeftClampState = digitalRead(Pins::LEFT_CLAMP);  // HIGH = RELEASED, LOW = ENGAGED
    digitalWrite(Pins::LEFT_CLAMP, !currentLeftClampState);
  }
}

void manualToggleRightClamp() {
  if (currentState == SystemState::READY) {
    bool currentRightClampState = digitalRead(Pins::RIGHT_CLAMP);  // HIGH = RELEASED, LOW = ENGAGED
    digitalWrite(Pins::RIGHT_CLAMP, !currentRightClampState);
  }
}

void manualToggleAlignCylinder() {
  if (currentState == SystemState::READY) {
    bool currentAlignmentCylinderState = digitalRead(Pins::ALIGN_CYLINDER);  // HIGH = EXTENDED, LOW = RETRACTED
    digitalWrite(Pins::ALIGN_CYLINDER, !currentAlignmentCylinderState);
  }
}

void manualStartCycle() {
  if (currentState == SystemState::READY) {
    currentState = SystemState::CYCLE_RUNNING;

    runCuttingCycle();
    updateTransferArmStartSignalDebouncer();
    currentState = SystemState::READY;
  }
}

void updateTransferArmStartSignalDebouncer() {
  // Force update the debouncer to capture the current state
  // This is crucial for detecting the next signal change
  for (int i = 0; i < 5; i++) {  // Multiple updates to ensure proper state capture
    transferArmStartSignal.update();
    delay(10);
  }
}

void runCuttingCycle() {
  if (!stepper) return; // Safety check

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
  digitalWrite(Pins::RIGHT_CLAMP, LOW);  // Extend right clamp

  // Approach phase
  stepper->setSpeedInHz(Motion::APPROACH_SPEED);
  stepper->setAcceleration(Motion::FORWARD_ACCEL);
  stepper->moveTo(Motion::APPROACH_DISTANCE * Motion::STEPS_PER_INCH);
  while (stepper->isRunning()) {
    delay(1);
  }

  // Cutting phase
  stepper->setSpeedInHz(Motion::CUTTING_SPEED);
  stepper->setAcceleration(Motion::FORWARD_ACCEL);  // Removed * 2 multiplier to prevent stalling
  stepper->moveTo((Motion::APPROACH_DISTANCE + Motion::CUTTING_DISTANCE) * Motion::STEPS_PER_INCH);
  while (stepper->isRunning()) {
    delay(1);
  }

  // Finish phase
  stepper->setSpeedInHz(Motion::FINISH_SPEED);
  stepper->setAcceleration(Motion::FORWARD_ACCEL);
  stepper->moveTo(Motion::FORWARD_DISTANCE * Motion::STEPS_PER_INCH);
  while (stepper->isRunning()) {
    delay(1);
  }

  stepper->forceStop();
  delay(50);

  // Release both clamps simultaneously
  releaseClamps();
  delay(100);

  // Return phase
  // Signal transfer arm to prevent Z-axis lowering during return
  digitalWrite(Pins::TRANSFER_ARM_SIGNAL, HIGH);

  // Fast return to slow-down point
  float currentPosition = stepper->getCurrentPosition() / (float)Motion::STEPS_PER_INCH;
  float slowDownPosition = currentPosition * 0.05;  // Changed from 0.01 to 0.05 for more reasonable slowdown

  stepper->setSpeedInHz(Motion::RETURN_SPEED);
  stepper->setAcceleration(Motion::RETURN_ACCEL);
  stepper->moveTo(slowDownPosition * Motion::STEPS_PER_INCH);

  unsigned long fastReturnStartTime = millis();
  unsigned long fastReturnTimeout = 15000;

  while (stepper->isRunning()) {
    if (millis() - fastReturnStartTime > fastReturnTimeout) {
      stepper->forceStop();
      break;
    }
    delay(1);
  }

  // Slow approach to home position
  float slowHomingSpeed = Motion::HOMING_SPEED / 2;
  stepper->setSpeedInHz(slowHomingSpeed);
  stepper->setAcceleration(Motion::RETURN_ACCEL / 4);
  stepper->moveTo(0);

  unsigned long slowApproachStartTime = millis();
  unsigned long slowApproachTimeout = 20000;

  while (stepper->isRunning()) {
    if (millis() - slowApproachStartTime > slowApproachTimeout) {
      stepper->forceStop();
      break;
    }
    delay(1);
  }

  delay(30);

  // Move to home offset
  stepper->setSpeedInHz(Motion::APPROACH_SPEED);
  stepper->setAcceleration(Motion::FORWARD_ACCEL);
  stepper->moveTo(Motion::HOME_OFFSET * Motion::STEPS_PER_INCH);
  while (stepper->isRunning()) {
    delay(1);
  }
  
  // Deactivate transfer arm signal - return is complete
  digitalWrite(Pins::TRANSFER_ARM_SIGNAL, LOW);

  delay(50);
  updateTransferArmStartSignalDebouncer();
}