#include <AccelStepper.h>
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
void initializeHardware();
void performHomingSequence();
void runCuttingCycle();
void handleSerialCommand(const String &command);
void printCurrentSettings();
void printSystemStatus();
void engageClamps();
void releaseClamps();
void staggeredReleaseClamps();

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

  // Initialize hardware and perform homing sequence
  initializeHardware();
  performHomingSequence();

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

void initializeHardware() {
  // Configure input pins
  pinMode(Pins::HOME_SWITCH, INPUT_PULLDOWN);
  pinMode(Pins::START_BUTTON, INPUT_PULLDOWN);
  pinMode(Pins::TRANSFER_ARM_START_SIGNAL, INPUT_PULLDOWN);  // Pin 15
  pinMode(Pins::CAMERA_SIGNAL, INPUT);  // Initialize camera signal pin

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
  cameraSignal.attach(Pins::CAMERA_SIGNAL);
  cameraSignal.interval(20);  // Debounce camera signal

  // Initialize stepper
  digitalWrite(Pins::ENABLE, HIGH);  // Disable briefly
  delay(100);                        // Wait 1 second for motor to reset
  digitalWrite(Pins::ENABLE, LOW);   // Enable
  delay(50);                         // Wait for enable to take effect

  stepper.setMaxSpeed(Motion::APPROACH_SPEED);
  stepper.setAcceleration(Motion::FORWARD_ACCEL);

  // Serial.println("✅ Hardware initialized"); // DO NOT DELETE
  logMessage("✅ Hardware initialized");
}

void performHomingSequence() {
  // Serial.println("🏠 Starting homing sequence..."); // DO NOT DELETE
  logMessage("🏠 Starting homing sequence...");
  logMessage("Using home offset: " + String(Motion::HOME_OFFSET) + " inches");
  currentState = SystemState::HOMING;

  // Clamps are already engaged from initialization

  // First, move a significant distance in the negative direction to ensure
  // we're past the home switch
  stepper.setMaxSpeed(Motion::HOMING_SPEED);
  stepper.setAcceleration(Motion::FORWARD_ACCEL);
  stepper.moveTo(-10000);  // Move 10,000 steps in negative direction

  // Use a much slower approach speed for final homing
  float slowHomingSpeed =
      Motion::HOMING_SPEED / 3;  // One-third of normal homing speed

  // Run until we hit the home switch or reach the target
  while (stepper.distanceToGo() != 0) {
    homeSwitch.update();

    // If we're within 2000 steps of where we think home might be, slow down
    // significantly
    if (abs(stepper.currentPosition()) < 2000) {
      stepper.setMaxSpeed(slowHomingSpeed);
    }

    if (homeSwitch.read() == HIGH) {
      // When home switch is triggered, stop immediately
      stepper.stop();
      
      // Wait for the stepper to actually stop before setting position
      while (stepper.isRunning()) {
        stepper.run();
      }
      
      stepper.setCurrentPosition(0);
      break;
    }
    stepper.run();
  }

  // If we didn't hit the home switch, we have a problem
  if (homeSwitch.read() == LOW) {
    // Serial.println("⚠ Failed to find home switch during initial homing!"); //
    // DO NOT DELETE
    logMessage("⚠ Failed to find home switch during initial homing!", "error");
    currentState = SystemState::ERROR;
    return;
  }

  // Now move to home offset with a gentler motion
  logMessage("Moving to home offset position: " + String(Motion::HOME_OFFSET) + " inches");
  stepper.setMaxSpeed(Motion::HOMING_SPEED / 2);  // Half speed for moving to offset
  stepper.setAcceleration(Motion::FORWARD_ACCEL / 2);  // Gentler acceleration
  stepper.moveTo(Motion::HOME_OFFSET * Motion::STEPS_PER_INCH);
  
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  // Add settle time after reaching home offset
  delay(Timing::HOME_SETTLE_TIME);
  
  logMessage("Home offset position reached. Current position: " + 
             String(stepper.currentPosition() / Motion::STEPS_PER_INCH) + " inches");

  // Now that we're at home position, release the clamps
  digitalWrite(Pins::LEFT_CLAMP, HIGH);
  digitalWrite(Pins::RIGHT_CLAMP, HIGH);
  
  // Add settle time after releasing clamps
  delay(Timing::CLAMP_RELEASE_TIME);

  isHomed = true;
  // Serial.println("✅ Homing complete"); // DO NOT DELETE
  logMessage("✅ Homing complete");
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
    performHomingSequence();
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
    performHomingSequence();
    if (isHomed) {
      currentState = SystemState::READY;  // Should be set by
                                          // performHomingSequence if successful
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
      stepper.setMaxSpeed(Motion::RETURN_SPEED);
      stepper.setAcceleration(Motion::RETURN_ACCEL);
      stepper.moveTo(Motion::HOME_OFFSET * Motion::STEPS_PER_INCH);
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }
      
      // Deactivate transfer arm signal - return is complete
      digitalWrite(Pins::TRANSFER_ARM_SIGNAL, LOW);
      logMessage("Transfer arm signal deactivated (return complete)");

      delay(50);
      updateTransferArmStartSignalDebouncer();
      logMessage("✅ Cycle short-circuited and completed!");
      return;
    }
  } else {
    logMessage("Camera not ready (signal LOW)");
  }

  // Approach phase
  logMessage("🚀 Approach phase...");
  stepper.setMaxSpeed(Motion::APPROACH_SPEED);
  stepper.setAcceleration(Motion::FORWARD_ACCEL);
  stepper.moveTo(Motion::APPROACH_DISTANCE * Motion::STEPS_PER_INCH);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  // Cutting phase
  logMessage("🔪 Cutting phase...");
  stepper.setMaxSpeed(Motion::CUTTING_SPEED);
  stepper.setAcceleration(Motion::FORWARD_ACCEL * 2);
  stepper.moveTo((Motion::APPROACH_DISTANCE + Motion::CUTTING_DISTANCE) * Motion::STEPS_PER_INCH);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  // Handle "End" class detection
  if (analysisResultReceived && lastDetectedClass.equalsIgnoreCase("End")) {
    logMessage("======= END CLASS DETECTED =======");
    float intermediatePosition = Motion::FORWARD_DISTANCE - Motion::END_DROP_DISTANCE_OFFSET;
    logMessage("Moving to intermediate position: " + String(intermediatePosition));

    stepper.setMaxSpeed(Motion::FINISH_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
    stepper.moveTo(intermediatePosition * Motion::STEPS_PER_INCH);
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
    stepper.stop();
    delay(Timing::MOTION_SETTLE_TIME);

    logMessage("Deactivating left clamp...");
    digitalWrite(Pins::LEFT_CLAMP, HIGH);  // Deactivate left clamp
    delay(200);

    logMessage("Proceeding to final forward distance.");
    logMessage("==================================");
  }

  // Finish phase
  logMessage("🏁 Finish phase...");
  stepper.setMaxSpeed(Motion::FINISH_SPEED);
  stepper.setAcceleration(Motion::FORWARD_ACCEL);
  stepper.moveTo(Motion::FORWARD_DISTANCE * Motion::STEPS_PER_INCH);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  stepper.stop();
  delay(50);

  // Release both clamps simultaneously
  releaseClamps();
  delay(100);

  // Return phase
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

  // Slow approach to home position
  float slowHomingSpeed = Motion::HOMING_SPEED / 2;
  stepper.setMaxSpeed(slowHomingSpeed);
  stepper.setAcceleration(Motion::RETURN_ACCEL / 4);
  stepper.moveTo(0);

  unsigned long slowApproachStartTime = millis();
  unsigned long slowApproachTimeout = 20000;

  while (stepper.distanceToGo() != 0) {
    stepper.run();
    if (millis() - slowApproachStartTime > slowApproachTimeout) {
      logMessage("⚠ Slow approach timeout - stopping movement...", "warn");
      break;
    }
  }

  delay(30);

  // Move to home offset
  logMessage("Moving to home offset position...");
  stepper.setMaxSpeed(Motion::APPROACH_SPEED);
  stepper.setAcceleration(Motion::FORWARD_ACCEL);
  stepper.moveTo(Motion::HOME_OFFSET * Motion::STEPS_PER_INCH);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  
  // Deactivate transfer arm signal - return is complete
  digitalWrite(Pins::TRANSFER_ARM_SIGNAL, LOW);
  logMessage("Transfer arm signal deactivated (return complete)");

  delay(50);
  updateTransferArmStartSignalDebouncer();

  logMessage("✅ Cycle complete!");
}