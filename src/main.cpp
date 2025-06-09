#include <AccelStepper.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Bounce2.h>

#include "system_states.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"
#include "OTA_Manager.h"
#include "StateMachine/STATES/00_INITIALIZING.h"
#include "StateMachine/STATES/00_HOMING.h"
#include "StateMachine/STATES/01_ALIGN.h"
#include "StateMachine/STATES/07_CUTTING_CYCLE.h"
#include "StateMachine/STATES/08_IDLE.h"

// System state (defined in system_states.h)

// Global objects
AccelStepper stepper(AccelStepper::DRIVER, Pins::STEP, Pins::DIR);
Bounce homeSwitch = Bounce();
Bounce startButton = Bounce();
Bounce transferArmStartSignal = Bounce();  // Transfer arm start signal
Bounce cameraSignal = Bounce();        // New debouncer for camera signal

// System state tracking
SystemState currentState = SystemState::INITIALIZING;
bool isHomed = false;

// Analysis result tracking
String lastDetectedClass = "";  // Will store the last detected wood class
bool analysisResultReceived =
    false;  // Flag to indicate if we've received an analysis result

// Image inference timing tracking
unsigned long inferenceStartTime = 0;  // Time when burst request was sent
bool inferenceTimingActive =
    false;  // Flag to track if we're timing an inference

// Function declarations
void runCuttingCycle();
void handleSerialCommand(const String &command);
void printCurrentSettings();
void printSystemStatus();

void handleSerialResponse(
    const String &response);  // New function to handle serial responses
void sendSerialMessage(
    const String &message);  // New function to send serial messages

// Manual Control Functions
void manualHome();
void manualJog(bool jogLeft, float distance);
void manualToggleLeftClamp();
void manualToggleRightClamp();
void manualToggleAlignCylinder();
void manualStartCycle();
void updateTransferArmStartSignalDebouncer();

// Function to send log messages to Serial
void logMessage(const String &message, const String &level,
                bool excludeSerial) {
  if (!excludeSerial) {
    Serial.println(message);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  logMessage("Serial initialized at " + String(SERIAL_BAUDRATE) + " baud.");

  // Initialize OTA functionality
  initOTA();
  logMessage("OTA initialized successfully!");

  // Initialize hardware and perform homing sequence using organized state machine
  executeInitializingState();
  executeHomingState();

  currentState = SystemState::READY;
  logMessage("System Ready!");
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
  cameraSignal.update();  // Update camera signal debouncer

  // Check for cycle start (either from button or machine start signal)
  if (startButton.fell() && currentState == SystemState::READY) {
    logMessage("Starting cycle from button...");
    currentState = SystemState::CYCLE_RUNNING;

    runCuttingCycle();
    updateTransferArmStartSignalDebouncer();
    currentState = SystemState::READY;
    logMessage("Cycle from button finished. System Ready.");
  }

  if (transferArmStartSignal.read() == HIGH && currentState == SystemState::READY) {
    logMessage("Starting cycle from transfer arm start signal...");
    currentState = SystemState::CYCLE_RUNNING;

    runCuttingCycle();
    updateTransferArmStartSignalDebouncer();
    currentState = SystemState::READY;
    logMessage("Cycle from transfer arm signal finished. System Ready.");
  }
}

// Hardware initialization moved to StateMachine/STATES/00_INITIALIZING.cpp

// Homing sequence moved to StateMachine/STATES/00_HOMING.cpp

void releaseClamps() {
  digitalWrite(Pins::LEFT_CLAMP, HIGH);
  digitalWrite(Pins::RIGHT_CLAMP, HIGH);
  delay(200);
}

void handleSerialCommand(const String &command) {
  logMessage("Serial command received: " + command);
  // Check for JSON commands first
  if (command.startsWith("{")) {
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, command);

    if (!error) {
      const char *cmd = doc["command"];
      if (cmd && strcmp(cmd, "identify") == 0) {
        // Send board identification
        DynamicJsonDocument response(128);
        response["board_id"] = Config::BOARD_ID;
        response["description"] = Config::BOARD_DESCRIPTION;
        response["type"] = "STAGE_2";

        String jsonResponse;
        serializeJson(response, jsonResponse);
        sendSerialMessage(jsonResponse);
        logMessage("Sent board identification: " + String(Config::BOARD_ID));
        return;
      }
    }
  }

  // Handle plain text commands
  if (command == "status") {
    printSystemStatus();
  } else if (command == "home") {
    logMessage("🏠 Initiating homing sequence via serial...");
    executeHomingState();
  } else if (command == "settings") {
    printCurrentSettings();
  } else if (command == "identify") {
    // Plain text identification response
    sendSerialMessage("BOARD_ID:" + String(Config::BOARD_ID));
    logMessage("Sent board identification (plain text): " +
               String(Config::BOARD_ID));
  } else if (command == "help") {
    logMessage("\n📚 Available Commands:", "info", true);
    logMessage("- status: Show current system status", "info", true);
    logMessage("- home: Perform homing sequence", "info", true);
    logMessage("- settings: Show current system settings", "info", true);
    logMessage("- identify: Send board identification", "info", true);
    logMessage("- help: Show this help message", "info", true);
  } else {
    logMessage("❌ Unknown serial command: " + command +
                   ". Type 'help' for available commands.",
               "warn", true);
  }
}

void printSystemStatus() {
  // Serial.println("\n📊 Current Status:"); // DO NOT DELETE
  logMessage("\n📊 Current Status:", "info", true);

  // Print state
  // Serial.print("System State: "); // DO NOT DELETE
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
  // Serial.println(stateStr);
  logMessage("System State: " + stateStr, "info", true);

  // Print position
  // Serial.print("Position: "); // DO NOT DELETE
  // Serial.print(stepper.currentPosition() / Motion::STEPS_PER_INCH); // DO NOT
  // DELETE Serial.println(" inches"); // DO NOT DELETE
  logMessage("Position: " +
                 String(stepper.currentPosition() / Motion::STEPS_PER_INCH) +
                 " inches",
             "info", true);

  // Print input states
  // Serial.print("Home Switch: "); // DO NOT DELETE
  // Serial.println(homeSwitch.read() ? "TRIGGERED" : "NOT TRIGGERED"); // DO
  // NOT DELETE
  logMessage("Home Switch: " +
                 String(homeSwitch.read() ? "TRIGGERED" : "NOT TRIGGERED"),
             "info", true);
  // Serial.print("Start Button: "); // DO NOT DELETE
  // Serial.println(startButton.read() ? "PRESSED" : "NOT PRESSED"); // DO NOT
  // DELETE
  logMessage(
      "Start Button: " + String(startButton.read() ? "PRESSED" : "NOT PRESSED"),
      "info", true);
  // Serial.print("Transfer Arm Start Signal: "); // DO NOT DELETE
  // Serial.println(transferArmStartSignal.read() ? "TRIGGERED" : "NOT TRIGGERED");
  // // DO NOT DELETE
  logMessage(
      "Transfer Arm Start Signal: " +
          String(transferArmStartSignal.read() ? "TRIGGERED" : "NOT TRIGGERED"),
      "info", true);
  // Serial.print("Camera Signal: "); // DO NOT DELETE
  // Serial.println(cameraSignal.read() ? "READY" : "NOT READY"); // DO NOT
  // DELETE
  logMessage(
      "Camera Signal: " + String(cameraSignal.read() ? "READY" : "NOT READY"),
      "info", true);

  // Print output states
  // Serial.print("Left Clamp: "); // DO NOT DELETE
  // Serial.println(digitalRead(Pins::LEFT_CLAMP) ? "RELEASED" : "ENGAGED"); //
  // DO NOT DELETE
  logMessage("Left Clamp: " +
                 String(digitalRead(Pins::LEFT_CLAMP) ? "RELEASED" : "ENGAGED"),
             "info", true);
  // Serial.print("Right Clamp: "); // DO NOT DELETE
  // Serial.println(digitalRead(Pins::RIGHT_CLAMP) ? "RELEASED" : "ENGAGED"); //
  // DO NOT DELETE
  logMessage(
      "Right Clamp: " +
          String(digitalRead(Pins::RIGHT_CLAMP) ? "RELEASED" : "ENGAGED"),
      "info", true);
  // Serial.print("Alignment Cylinder: "); // DO NOT DELETE
  // Serial.println(digitalRead(Pins::ALIGN_CYLINDER) ? "EXTENDED" :
  // "RETRACTED"); // DO NOT DELETE
  logMessage(
      "Alignment Cylinder: " +
          String(digitalRead(Pins::ALIGN_CYLINDER) ? "EXTENDED" : "RETRACTED"),
      "info", true);
  logMessage(
      "Transfer Arm Signal: " +
          String(digitalRead(Pins::TRANSFER_ARM_SIGNAL) ? "ACTIVE (Z-BLOCKED)" : "INACTIVE"),
      "info", true);
}

void printCurrentSettings() {
  // Serial.println("\n⚙ Current Settings:"); // DO NOT DELETE
  logMessage("\n⚙ Current Settings:", "info", true);
  // Serial.println("Motion Parameters:"); // DO NOT DELETE
  logMessage("Motion Parameters:", "info", true);
  // Serial.print("- Steps per inch: "); Serial.println(Motion::STEPS_PER_INCH);
  // // DO NOT DELETE
  logMessage("- Steps per inch: " + String(Motion::STEPS_PER_INCH), "info",
             true);
  // Serial.print("- Home offset: "); Serial.println(Motion::HOME_OFFSET); // DO
  // NOT DELETE
  logMessage("- Home offset: " + String(Motion::HOME_OFFSET), "info", true);
  // Serial.print("- Approach distance: ");
  // Serial.println(Motion::APPROACH_DISTANCE); // DO NOT DELETE
  logMessage("- Approach distance: " + String(Motion::APPROACH_DISTANCE),
             "info", true);
  // Serial.print("- Cutting distance: ");
  // Serial.println(Motion::CUTTING_DISTANCE); // DO NOT DELETE
  logMessage("- Cutting distance: " + String(Motion::CUTTING_DISTANCE), "info",
             true);
  // Serial.print("- Forward distance: ");
  // Serial.println(Motion::FORWARD_DISTANCE); // DO NOT DELETE
  logMessage("- Forward distance: " + String(Motion::FORWARD_DISTANCE), "info", true);

  // Serial.println("\nSpeed Settings (steps/sec):"); // DO NOT DELETE
  logMessage("\nSpeed Settings (steps/sec):", "info", true);
  // Serial.print("- Homing: "); Serial.println(Motion::HOMING_SPEED); // DO
  // NOT DELETE
  logMessage("- Homing: " + String(Motion::HOMING_SPEED), "info", true);
  // Serial.print("- Approach: "); Serial.println(Motion::APPROACH_SPEED);
  // // DO NOT DELETE
  logMessage("- Approach: " + String(Motion::APPROACH_SPEED), "info", true);
  // Serial.print("- Cutting: "); Serial.println(Motion::CUTTING_SPEED); //
  // DO NOT DELETE
  logMessage("- Cutting: " + String(Motion::CUTTING_SPEED), "info", true);
  // Serial.print("- Finish: "); Serial.println(Motion::FINISH_SPEED); // DO
  // NOT DELETE
  logMessage("- Finish: " + String(Motion::FINISH_SPEED), "info", true);
  // Serial.print("- Return: "); Serial.println(Motion::RETURN_SPEED); // DO
  // NOT DELETE
  logMessage("- Return: " + String(Motion::RETURN_SPEED), "info", true);

  // Serial.println("\nAcceleration Settings (steps/sec²):"); // DO NOT DELETE
  logMessage("\nAcceleration Settings (steps/sec²):", "info", true);
  // Serial.print("- Forward: "); Serial.println(Motion::FORWARD_ACCEL); //
  // DO NOT DELETE
  logMessage("- Forward: " + String(Motion::FORWARD_ACCEL), "info", true);
  // Serial.print("- Return: "); Serial.println(Motion::RETURN_ACCEL); //
  // DO NOT DELETE
  logMessage("- Return: " + String(Motion::RETURN_ACCEL), "info", true);

  // Serial.println("\nTiming Settings (ms):"); // DO NOT DELETE
  logMessage("\nTiming Settings (ms):", "info", true);
  // Serial.print("- Clamp engage time: ");
  // Serial.println(Timing::CLAMP_ENGAGE_TIME); // DO NOT DELETE
  logMessage("- Clamp engage time: " + String(Timing::CLAMP_ENGAGE_TIME),
             "info", true);
  // Serial.print("- Clamp release time: ");
  // Serial.println(Timing::CLAMP_RELEASE_TIME); // DO NOT DELETE
  logMessage("- Clamp release time: " + String(Timing::CLAMP_RELEASE_TIME),
             "info", true);
  // Serial.print("- Home settle time: ");
  // Serial.println(Timing::HOME_SETTLE_TIME); // DO NOT DELETE
  logMessage("- Home settle time: " + String(Timing::HOME_SETTLE_TIME), "info",
             true);
  // Serial.print("- Motion settle time: ");
  // Serial.println(Timing::MOTION_SETTLE_TIME); // DO NOT DELETE
  logMessage("- Motion settle time: " + String(Timing::MOTION_SETTLE_TIME),
             "info", true);
  // Serial.println(); // DO NOT DELETE
  logMessage("", "info", true);  // Empty line for spacing, only to web log
}



// Function to handle serial responses
void handleSerialResponse(const String &response) {
  // This function handles JSON responses from Python
  logMessage("Serial response received: " + response, "info",
             true);  // Exclude from serial to prevent echo

  // Try to parse JSON response
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, response);

  if (!error) {
    // Handle JSON responses from Python
    if (doc.containsKey("status")) {
      String status = doc["status"].as<String>();
      logMessage("Camera status: " + status);

      if (status == "success" && doc.containsKey("burst_complete")) {
        String result = doc["burst_complete"].as<String>();
        logMessage("Camera burst complete with result: " + result);

        // Check for analysis results
        if (doc.containsKey("analysis_result")) {
          // Extract the analysis results
          JsonObject analysis = doc["analysis_result"];

          if (analysis.containsKey("class")) {
            String detectedClass = analysis["class"].as<String>();
            float confidence = 0.0;

            if (analysis.containsKey("confidence")) {
              confidence = analysis["confidence"].as<float>();
            }

            // Calculate inference duration if timing is active
            unsigned long inferenceDuration = 0;
            if (inferenceTimingActive) {
              inferenceDuration = millis() - inferenceStartTime;
              inferenceTimingActive = false;
            }

            // Log the detected class
            logMessage("======= WOOD ANALYSIS RESULT =======");
            logMessage("Detected class: " + detectedClass);
            logMessage("Confidence: " + String(confidence * 100) + "%");
            if (inferenceDuration > 0) {
              logMessage("Inference duration: " + String(inferenceDuration) +
                         " ms");
            }
            logMessage("====================================");

            // Update analysis result tracking
            lastDetectedClass = detectedClass;
            analysisResultReceived = true;
          } else if (analysis.containsKey("error")) {
            // Handle error in analysis
            String errorMsg = analysis["error"].as<String>();

            // Calculate inference duration if timing is active (even for
            // errors)
            unsigned long inferenceDuration = 0;
            if (inferenceTimingActive) {
              inferenceDuration = millis() - inferenceStartTime;
              inferenceTimingActive = false;
            }

            logMessage("======= WOOD ANALYSIS ERROR =======", "error");
            logMessage("Error: " + errorMsg, "error");
            if (inferenceDuration > 0) {
              logMessage("Time to error: " + String(inferenceDuration) + " ms",
                         "error");
            }
            logMessage("===================================", "error");

            // Reset analysis result tracking on error
            lastDetectedClass = "";
            analysisResultReceived = false;
          }
        }
      } else if (status == "error" && doc.containsKey("message")) {
        String errorMsg = doc["message"].as<String>();

        // Calculate inference duration if timing is active (even for
        // errors)
        unsigned long inferenceDuration = 0;
        if (inferenceTimingActive) {
          inferenceDuration = millis() - inferenceStartTime;
          inferenceTimingActive = false;
        }

        logMessage("Camera error: " + errorMsg, "error");
        if (inferenceDuration > 0) {
          logMessage("Time to error: " + String(inferenceDuration) + " ms",
                     "error");
        }

        // Reset analysis result tracking on error
        lastDetectedClass = "";
        analysisResultReceived = false;
      }
    }
  } else {
    logMessage("JSON parsing error: " + String(error.c_str()), "error");

    // Reset analysis result tracking on error
    lastDetectedClass = "";
    analysisResultReceived = false;
  }
}

// Function to send serial messages
void sendSerialMessage(const String &message) {
  Serial.println(message);
  logMessage("Sent serial message: " + message);
}

// Manual Control Function Implementations
void manualHome() {
  if (currentState == SystemState::READY) {
    logMessage("Initiating manual homing sequence...");
    executeHomingState();
    if (isHomed) {
      currentState = SystemState::READY;  // Should be set by
                                          // executeHomingState if successful
      logMessage("Manual homing complete. System Ready.");
    } else {
      logMessage("Manual homing failed.", "error");
      // currentState might be SystemState::ERROR if homing failed
    }
  } else {
    logMessage("Cannot home: System not in READY state.", "warn");
  }
}

void manualJog(bool jogLeft, float distance) {
  if (currentState == SystemState::READY) {
    if (distance <= 0 || distance > 5.0) {  // Basic validation for jog distance
      logMessage("Invalid jog distance: " + String(distance), "warn");
      return;
    }
    logMessage("Manual jog: " + String(jogLeft ? "LEFT" : "RIGHT") + " by " +
               String(distance) + " inches.");
    float currentPosInches =
        stepper.currentPosition() / (float)Motion::STEPS_PER_INCH;
    float targetPosInches;
    if (jogLeft) {
      targetPosInches = currentPosInches - distance;
      // Prevent jogging beyond a safe minimum (e.g., slightly before 0)
      if (targetPosInches < -0.1) targetPosInches = -0.1;
    } else {
      targetPosInches = currentPosInches + distance;
      // Prevent jogging beyond a safe maximum (e.g., Motion::FORWARD_DISTANCE +
      // some buffer or an absolute max) For now, let's use a generous limit,
      // but this should be refined.
      if (targetPosInches > (Motion::FORWARD_DISTANCE + 10.0))
        targetPosInches = Motion::FORWARD_DISTANCE + 10.0;
    }

    logMessage("Current position: " + String(currentPosInches) +
               ", Target position: " + String(targetPosInches));
    // Use a moderate speed and acceleration for jogging
    moveStepperToPosition(targetPosInches, Motion::APPROACH_SPEED / 2,
                          Motion::FORWARD_ACCEL / 2);
    logMessage(
        "Jog complete. Current position: " +
        String(stepper.currentPosition() / (float)Motion::STEPS_PER_INCH));
  } else {
    logMessage("Cannot jog: System not in READY state.", "warn");
  }
}

void manualToggleLeftClamp() {
  if (currentState == SystemState::READY) {
    bool currentLeftClampState =
        digitalRead(Pins::LEFT_CLAMP);  // HIGH = RELEASED, LOW = ENGAGED
    digitalWrite(Pins::LEFT_CLAMP, !currentLeftClampState);
    logMessage("Left clamp " +
               String(!currentLeftClampState ? "ENGAGED" : "RELEASED") + ".");
  } else {
    logMessage("Cannot toggle left clamp: System not in READY state.", "warn");
  }
}

void manualToggleRightClamp() {
  if (currentState == SystemState::READY) {
    bool currentRightClampState =
        digitalRead(Pins::RIGHT_CLAMP);  // HIGH = RELEASED, LOW = ENGAGED
    digitalWrite(Pins::RIGHT_CLAMP, !currentRightClampState);
    logMessage("Right clamp " +
               String(!currentRightClampState ? "ENGAGED" : "RELEASED") + ".");
  } else {
    logMessage("Cannot toggle right clamp: System not in READY state.", "warn");
  }
}

void manualToggleAlignCylinder() {
  if (currentState == SystemState::READY) {
    bool currentAlignmentCylinderState =
        digitalRead(Pins::ALIGN_CYLINDER);  // HIGH = EXTENDED, LOW = RETRACTED
    digitalWrite(Pins::ALIGN_CYLINDER, !currentAlignmentCylinderState);
    logMessage(
        "Alignment cylinder " +
        String(!currentAlignmentCylinderState ? "EXTENDED" : "RETRACTED") +
        ".");
  } else {
    logMessage("Cannot toggle alignment cylinder: System not in READY state.",
               "warn");
  }
}

void manualStartCycle() {
  if (currentState == SystemState::READY) {
    logMessage("Starting cycle manually...");
    currentState = SystemState::CYCLE_RUNNING;

    runCuttingCycle();
    updateTransferArmStartSignalDebouncer();
    currentState = SystemState::READY;
    logMessage("Cycle manually finished. System Ready.");
  } else {
    logMessage("Cannot start cycle: System not in READY state.", "warn");
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
  // Execute the organized cutting cycle
  executeCuttingCycle();
}