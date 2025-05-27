#include <AccelStepper.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <Bounce2.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "webpage.h"
#include "cutting_cycle.h"
#include "Config/system_config.h"
#include "Config/pin_definitions.h"

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");  // Keep only dashboard WebSocket

// Create Preferences object
Preferences preferences;

// Variable to store the current forward distance
float currentForwardDistance;  // Will be initialized in setup()

// Variable to store the current home offset
float currentHomeOffset;  // Will be initialized in setup()

// System state
enum class SystemState { INITIALIZING, HOMING, READY, CYCLE_RUNNING, ERROR };

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
void moveStepperToPosition(float position, float speed, float acceleration);
void engageClamps();
void releaseClamps();
void staggeredReleaseClamps();
void notifyClients(float distance);
void notifyHomeOffsetClients(float homeOffset);
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len);
void initWebSockets();  // Updated to initialize both WebSockets
void initWebSocket();   // Keep for backward compatibility
void sendBurstRequest();
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

// Function to send log messages to Serial and WebSocket
void logMessage(const String &message, const String &level,
                bool excludeSerial) {
  if (!excludeSerial) {
    // Serial.println(message);
  }

  if (ws.count() > 0) {  // Check if there are any connected dashboard clients
    DynamicJsonDocument doc(256);  // Adjust size as needed
    doc["type"] = "log";
    doc["message"] = message;
    doc["level"] = level;

    String jsonMessage;
    serializeJson(doc, jsonMessage);
    ws.textAll(jsonMessage);
  }
}

void notifyClients(float distance) {
  // char msg[8];
  // snprintf(msg, sizeof(msg), "%.1f", distance);
  // ws.textAll(msg); // Old method

  if (ws.count() > 0) {
    DynamicJsonDocument doc(64);
    doc["type"] = "distance";
    doc["value"] = distance;
    String jsonMessage;
    serializeJson(doc, jsonMessage);
    ws.textAll(jsonMessage);
  }
}

void notifyHomeOffsetClients(float homeOffset) {
  if (ws.count() > 0) {
    DynamicJsonDocument doc(64);
    doc["type"] = "home_offset";
    doc["value"] = homeOffset;
    String jsonMessage;
    serializeJson(doc, jsonMessage);
    ws.textAll(jsonMessage);
  }
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len &&
      info->opcode == WS_TEXT) {
    data[len] = 0;
    // Check if the message is JSON or plain text
    if (data[0] == '{') {            // Assuming JSON messages start with '{'
      DynamicJsonDocument doc(256);  // Adjust size as needed
      DeserializationError error = deserializeJson(doc, (char *)data);

      if (error) {
        logMessage(
            "Failed to parse JSON from WebSocket: " + String(error.c_str()),
            "error");
        return;
      }

      const char *type = doc["type"];

      if (type && strcmp(type, "manual_control") == 0) {
        const char *command = doc["command"];
        if (command) {
          logMessage("Manual control command received: " + String(command));
          if (currentState != SystemState::READY) {
            logMessage(
                "Manual control disabled: System not ready or cycle running.",
                "warn");
            return;
          }

          if (strcmp(command, "home") == 0) {
            manualHome();
          } else if (strcmp(command, "jog_left") == 0) {
            float distance =
                doc["distance"] | 0.1f;  // Default to 0.1 if not provided
            manualJog(true, distance);
          } else if (strcmp(command, "jog_right") == 0) {
            float distance =
                doc["distance"] | 0.1f;  // Default to 0.1 if not provided
            manualJog(false, distance);
          } else if (strcmp(command, "toggle_left_clamp") == 0) {
            manualToggleLeftClamp();
          } else if (strcmp(command, "toggle_right_clamp") == 0) {
            manualToggleRightClamp();
          } else if (strcmp(command, "toggle_align_cylinder") == 0) {
            manualToggleAlignCylinder();
          } else if (strcmp(command, "start_cycle") == 0) {
            manualStartCycle();
          } else {
            logMessage("Unknown manual command: " + String(command), "warn");
          }
        }
      } else if (type && strcmp(type, "update_forward_distance") == 0) {
        float newDistance = doc["value"];
        if (newDistance >= 0 && newDistance <= 100) {  // Basic validation
          currentForwardDistance = newDistance;
          preferences.putFloat("fwd_dist", currentForwardDistance);
          notifyClients(
              currentForwardDistance);  // Notify all clients about the change
          logMessage("Forward distance updated via WebSocket to: " +
                     String(currentForwardDistance));
        } else {
          logMessage(
              "Invalid forward distance value received: " + String(newDistance),
              "warn");
        }
      } else if (type && strcmp(type, "update_home_offset") == 0) {
        float newOffset = doc["value"];
        if (newOffset >= 0 && newOffset <= 5) {  // Basic validation
          currentHomeOffset = newOffset;
          preferences.putFloat("home_offset", currentHomeOffset);
          notifyHomeOffsetClients(
              currentHomeOffset);  // Notify all clients about the change
          logMessage("Home offset updated via WebSocket to: " +
                     String(currentHomeOffset));
        } else {
          logMessage("Invalid home offset value received: " + String(newOffset),
                     "warn");
        }
      } else {  // Assume it's the old plain text distance value
        float newDistance = atof((char *)data);
        if (newDistance >= 0 && newDistance <= 100) {  // Basic validation
          currentForwardDistance = newDistance;
          preferences.putFloat("fwd_dist", currentForwardDistance);
          notifyClients(
              currentForwardDistance);  // Notify all clients about the change
          logMessage("Forward distance updated via WebSocket to: " +
                     String(currentForwardDistance));
        } else {
          logMessage("Invalid distance value received: " + String((char *)data),
                     "warn");
        }
      }
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      logMessage("WebSocket client #" + String(client->id()) +
                 " connected from " +
                 String(client->remoteIP().toString().c_str()));
      notifyClients(currentForwardDistance);
      notifyHomeOffsetClients(currentHomeOffset);
      break;
    case WS_EVT_DISCONNECT:
      logMessage("WebSocket client #" + String(client->id()) + " disconnected");
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSockets() {
  // Dashboard WebSocket
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

// For backward compatibility
void initWebSocket() { initWebSockets(); }

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  logMessage("Serial initialized at " + String(SERIAL_BAUDRATE) + " baud.");

  // Initialize preferences
  preferences.begin("tablesaw", false);  // false = R/W mode
  logMessage("Preferences initialized.");

  // Load saved forward distance or use default if not found
  currentForwardDistance =
      preferences.getFloat("fwd_dist", Motion::FORWARD_DISTANCE);
  logMessage("Loaded forward distance: " + String(currentForwardDistance));

  // Load saved home offset or use default if not found
  currentHomeOffset = preferences.getFloat("home_offset", Motion::HOME_OFFSET);
  logMessage("Loaded home offset: " + String(currentHomeOffset) + " inches (from preferences)");
  logMessage("Default home offset constant: " + String(Motion::HOME_OFFSET) + " inches");

  // Connect to Wi-Fi using the namespaced credentials
  WiFi.begin(Config::WIFI_SSID, Config::WIFI_PASSWORD);
  esp_wifi_set_ps(WIFI_PS_NONE);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    logMessage("Connecting to WiFi...");
  }
  logMessage("WiFi Connected. IP Address: " + WiFi.localIP().toString());

  // Initialize WebSockets
  initWebSockets();
  logMessage("WebSockets initialized.");

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });
  logMessage("HTTP route for / configured.");

  // Start server
  server.begin();
  logMessage("HTTP server started.");

  // Initialize hardware and perform homing sequence
  initializeHardware();
  performHomingSequence();

  currentState = SystemState::READY;
  logMessage("System Ready!");
  logMessage("Current Forward Distance: " + String(currentForwardDistance));
  logMessage("Current Home Offset: " + String(currentHomeOffset));
  printCurrentSettings();
}

void loop() {
  // Clean up disconnected clients
  ws.cleanupClients();

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
  logMessage("Using home offset: " + String(currentHomeOffset) + " inches");
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
  logMessage("Moving to home offset position: " + String(currentHomeOffset) + " inches");
  stepper.setMaxSpeed(Motion::HOMING_SPEED / 2);  // Half speed for moving to offset
  stepper.setAcceleration(Motion::FORWARD_ACCEL / 2);  // Gentler acceleration
  moveStepperToPosition(currentHomeOffset, Motion::HOMING_SPEED / 2,
                        Motion::FORWARD_ACCEL / 2);
  
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



void moveStepperToPosition(float position, float speed, float acceleration) {
  stepper.setMaxSpeed(speed);
  stepper.setAcceleration(acceleration);
  stepper.moveTo(position * Motion::STEPS_PER_INCH);

  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
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
  logMessage("- Home offset: " + String(currentHomeOffset) + " (Current Saved)",
             "info", true);
  logMessage("- Default Home offset: " + String(Motion::HOME_OFFSET) +
                 " (Default Constant)",
             "info", true);
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
  logMessage("- Forward distance: " + String(currentForwardDistance) +
                 " (Current Saved)",
             "info", true);  // Show current actual
  logMessage("- Default Forward distance: " + String(Motion::FORWARD_DISTANCE) +
                 " (Default Constant)",
             "info", true);

  // Serial.println("\nSpeed Settings (inches/sec):"); // DO NOT DELETE
  logMessage("\nSpeed Settings (inches/sec):", "info", true);
  // Serial.print("- Homing: "); Serial.println(Motion::HOMING_SPEED_IPS); // DO
  // NOT DELETE
  logMessage("- Homing: " + String(Motion::HOMING_SPEED_IPS), "info", true);
  // Serial.print("- Approach: "); Serial.println(Motion::APPROACH_SPEED_IPS);
  // // DO NOT DELETE
  logMessage("- Approach: " + String(Motion::APPROACH_SPEED_IPS), "info", true);
  // Serial.print("- Cutting: "); Serial.println(Motion::CUTTING_SPEED_IPS); //
  // DO NOT DELETE
  logMessage("- Cutting: " + String(Motion::CUTTING_SPEED_IPS), "info", true);
  // Serial.print("- Finish: "); Serial.println(Motion::FINISH_SPEED_IPS); // DO
  // NOT DELETE
  logMessage("- Finish: " + String(Motion::FINISH_SPEED_IPS), "info", true);
  // Serial.print("- Return: "); Serial.println(Motion::RETURN_SPEED_IPS); // DO
  // NOT DELETE
  logMessage("- Return: " + String(Motion::RETURN_SPEED_IPS), "info", true);

  // Serial.println("\nAcceleration Settings (inches/sec²):"); // DO NOT DELETE
  logMessage("\nAcceleration Settings (inches/sec²):", "info", true);
  // Serial.print("- Forward: "); Serial.println(Motion::FORWARD_ACCEL_IPS2); //
  // DO NOT DELETE
  logMessage("- Forward: " + String(Motion::FORWARD_ACCEL_IPS2), "info", true);
  // Serial.print("- Return: "); Serial.println(Motion::RETURN_ACCEL_IPS2); //
  // DO NOT DELETE
  logMessage("- Return: " + String(Motion::RETURN_ACCEL_IPS2), "info", true);

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

// Send burst request to webcam endpoint using Serial
void sendBurstRequest() {
  logMessage("Sending burst request via Serial...");

  // Start timing the inference
  inferenceStartTime = millis();
  inferenceTimingActive = true;

  // Send JSON burst command via serial
  sendSerialMessage("{\"command\":\"burst\"}");
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
      // Prevent jogging beyond a safe maximum (e.g., currentForwardDistance +
      // some buffer or an absolute max) For now, let's use a generous limit,
      // but this should be refined.
      if (targetPosInches > (currentForwardDistance + 10.0))
        targetPosInches = currentForwardDistance + 10.0;
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