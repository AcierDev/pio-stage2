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

// WiFi credentials - make these available to the whole program
namespace Config {
const char *WIFI_SSID = "Everwood";
const char *WIFI_PASSWORD = "Everwood-Staff";
}  // namespace Config

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");  // Keep only dashboard WebSocket

// Create Preferences object
Preferences preferences;
//hi my name is aykov
// Serial communication settings
const unsigned long SERIAL_BAUDRATE = 115200;

// Pin Configuration - Using ESP32 GPIO pins
namespace Pins {
// Input pins
constexpr int HOME_SWITCH = 22;
constexpr int START_BUTTON = 23;
constexpr int MACHINE_1TO2_START_SIGNAL =
    15;                            // Changed from 18 to 5 for Stage 1 signal
constexpr int CAMERA_SIGNAL = 34;  // New camera signal pin

// Output pins
constexpr int STEP = 18;
constexpr int DIR = 5;
constexpr int ENABLE = 27;
constexpr int LEFT_CLAMP = 13;
constexpr int RIGHT_CLAMP = 12;
constexpr int ALIGN_CYLINDER = 14;
}  // namespace Pins

// Motion Parameters
namespace Motion {
constexpr int STEPS_PER_INCH = 43;   // Halved from 63 for the 30:80 tooth ratio
constexpr float HOME_OFFSET = 0.65;  // Position value stays the same
constexpr float APPROACH_DISTANCE = 5.0;  // Position value stays the same
constexpr float CUTTING_DISTANCE = 7.3;   // Position value stays the same
constexpr float FORWARD_DISTANCE = 29.5;  // Position value stays the same
constexpr float END_DROP_DISTANCE_OFFSET =
    5;  // Distance before the forward distance

// Speed Settings (inches/second)
constexpr float HOMING_SPEED_IPS = 23.4375f;  // 750 steps/sec ÷ 32 steps/inch
constexpr float APPROACH_SPEED_IPS = 312.5f;  // 10000 steps/sec ÷ 32 steps/inch
constexpr float CUTTING_SPEED_IPS = 1.96875f;  // 63 steps/sec ÷ 32 steps/inch
constexpr float FINISH_SPEED_IPS = 390.625f;  // 12500 steps/sec ÷ 32 steps/inch
constexpr float RETURN_SPEED_IPS = 390.625f;  // 12500 steps/sec ÷ 32 steps/inch

// Speed Settings (steps/second) - For internal use
constexpr float HOMING_SPEED = HOMING_SPEED_IPS * STEPS_PER_INCH;
constexpr float APPROACH_SPEED = APPROACH_SPEED_IPS * STEPS_PER_INCH;
constexpr float CUTTING_SPEED = CUTTING_SPEED_IPS * STEPS_PER_INCH;
constexpr float FINISH_SPEED = FINISH_SPEED_IPS * STEPS_PER_INCH;
constexpr float RETURN_SPEED = RETURN_SPEED_IPS * STEPS_PER_INCH;

// Acceleration Settings (inches/second^2)
constexpr float FORWARD_ACCEL_IPS2 =
    234.375f;  // 7500 steps/sec^2 ÷ 32 steps/inch
constexpr float RETURN_ACCEL_IPS2 =
    234.375f;  // 7500 steps/sec^2 ÷ 32 steps/inch

// Acceleration Settings (steps/second^2) - For internal use
constexpr float FORWARD_ACCEL = FORWARD_ACCEL_IPS2 * STEPS_PER_INCH;
constexpr float RETURN_ACCEL = RETURN_ACCEL_IPS2 * STEPS_PER_INCH;
}  // namespace Motion

// Variable to store the current forward distance
float currentForwardDistance;  // Will be initialized in setup()

// Variable to store the current home offset
float currentHomeOffset;  // Will be initialized in setup()

// Timing Settings (milliseconds)
namespace Timing {
constexpr int CLAMP_ENGAGE_TIME = 200;
constexpr int CLAMP_RELEASE_TIME = 200;
constexpr int HOME_SETTLE_TIME = 30;
constexpr int MOTION_SETTLE_TIME = 50;
constexpr int ALIGNMENT_TIME = 300;  // Changed to 300ms for alignment cylinder
constexpr int LEFT_CLAMP_PULSE_TIME =
    100;  // New constant for left clamp pulse duration
constexpr int LEFT_CLAMP_RETRACT_WAIT = 50;
constexpr int CLAMP_RELEASE_SETTLE_TIME = 100;
}  // namespace Timing

// System state
enum class SystemState { INITIALIZING, HOMING, READY, CYCLE_RUNNING, ERROR };

// Global objects
AccelStepper stepper(AccelStepper::DRIVER, Pins::STEP, Pins::DIR);
Bounce homeSwitch = Bounce();
Bounce startButton = Bounce();
Bounce machineStartSignal = Bounce();  // Renamed from remoteStart
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
void handleSerialResponse();  // New function to handle serial responses
void sendSerialMessage(
    const String &message);  // New function to send serial messages

// Manual Control Functions
void manualHome();
void manualJog(bool jogLeft, float distance);
void manualToggleLeftClamp();
void manualToggleRightClamp();
void manualToggleAlignCylinder();
void manualStartCycle();

// Function to send log messages to Serial and WebSocket
void logMessage(const String &message, const String &level = "info",
                bool excludeSerial = false) {
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
  logMessage("Loaded home offset: " + String(currentHomeOffset));

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
  handleSerialResponse();

  // Update button states
  homeSwitch.update();
  startButton.update();
  machineStartSignal.update();
  cameraSignal.update();  // Update camera signal debouncer

  // Check for cycle start (either from button or machine start signal)
  if (startButton.fell() && currentState == SystemState::READY) {
    logMessage("Starting cycle from button...");
    currentState = SystemState::CYCLE_RUNNING;

    runCuttingCycle();
    currentState = SystemState::READY;
    logMessage("Cycle from button finished. System Ready.");
  }

  if (machineStartSignal.rose() && currentState == SystemState::READY) {
    logMessage("Starting cycle from machine start signal...");
    currentState = SystemState::CYCLE_RUNNING;

    runCuttingCycle();
    currentState = SystemState::READY;
    logMessage("Cycle from machine signal finished. System Ready.");
  }
}

void initializeHardware() {
  // Configure input pins
  pinMode(Pins::HOME_SWITCH, INPUT_PULLUP);
  pinMode(Pins::START_BUTTON, INPUT_PULLUP);
  pinMode(Pins::MACHINE_1TO2_START_SIGNAL, INPUT_PULLUP);  // Pin 5 with pullup
  pinMode(Pins::CAMERA_SIGNAL, INPUT);  // Initialize camera signal pin

  // Configure output pins
  pinMode(Pins::ENABLE, OUTPUT);
  pinMode(Pins::LEFT_CLAMP, OUTPUT);
  pinMode(Pins::RIGHT_CLAMP, OUTPUT);
  pinMode(Pins::ALIGN_CYLINDER, OUTPUT);

  // Initialize clamps to engaged state (extended)
  digitalWrite(Pins::LEFT_CLAMP, LOW);   // Start with clamps engaged
  digitalWrite(Pins::RIGHT_CLAMP, LOW);  // Start with clamps engaged

  // Initialize alignment cylinder to retracted position
  digitalWrite(Pins::ALIGN_CYLINDER, LOW);

  // Setup debouncing
  homeSwitch.attach(Pins::HOME_SWITCH);
  homeSwitch.interval(10);
  startButton.attach(Pins::START_BUTTON);
  startButton.interval(20);
  machineStartSignal.attach(Pins::MACHINE_1TO2_START_SIGNAL);
  machineStartSignal.interval(20);  // Ensure good debouncing for the signal pin
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
      stepper.setSpeed(0);
      stepper.stop();
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

  // Now move to home offset with a gentler motion - no delay
  stepper.setMaxSpeed(Motion::HOMING_SPEED /
                      2);  // Half speed for moving to offset
  stepper.setAcceleration(Motion::FORWARD_ACCEL / 2);  // Gentler acceleration
  moveStepperToPosition(currentHomeOffset, Motion::HOMING_SPEED / 2,
                        Motion::FORWARD_ACCEL / 2);

  // Now that we're at home position, release the clamps
  digitalWrite(Pins::LEFT_CLAMP, HIGH);
  digitalWrite(Pins::RIGHT_CLAMP, HIGH);

  isHomed = true;
  // Serial.println("✅ Homing complete"); // DO NOT DELETE
  logMessage("✅ Homing complete");
}

void runCuttingCycle() {
  // Reset analysis result tracking at the start of each cycle
  lastDetectedClass = "";
  analysisResultReceived = false;

  // Initial left clamp pulse and alignment cylinder extension
  digitalWrite(Pins::LEFT_CLAMP, LOW);  // Engage (extend) left clamp

  // Left clamp only extends for 100ms
  delay(100);
  digitalWrite(Pins::LEFT_CLAMP, HIGH);      // Retract left clamp
  digitalWrite(Pins::ALIGN_CYLINDER, HIGH);  // Extend alignment cylinder

  // Alignment cylinder stays extended for the remainder of the time
  delay(300);

  digitalWrite(Pins::ALIGN_CYLINDER, LOW);  // Retract alignment cylinder
  delay(125);

  // Engage clamps
  digitalWrite(Pins::RIGHT_CLAMP, LOW);  // Engage right clamp
  delay(200);
  digitalWrite(Pins::LEFT_CLAMP, LOW);  // Engage left clamp

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
      // Update WebSocket clients and handle serial responses more frequently
      ws.cleanupClients();
      handleSerialResponse();  // Process serial responses during wait
      delay(10);               // Smaller delay for more responsive checking
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

      // Return directly to home
      moveStepperToPosition(currentHomeOffset, Motion::RETURN_SPEED,
                            Motion::RETURN_ACCEL);

      // At the end of the cycle, add a small delay
      delay(50);

      // Force update the debouncer to capture the current state
      for (int i = 0; i < 5; i++) {
        machineStartSignal.update();
        delay(10);
      }

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
        currentForwardDistance - Motion::END_DROP_DISTANCE_OFFSET;
    logMessage("Moving to intermediate position: " +
               String(intermediatePosition));

    // Move to the intermediate position
    moveStepperToPosition(intermediatePosition, Motion::FINISH_SPEED,
                          Motion::FORWARD_ACCEL);
    stepper.stop();  // Ensure motor has completely stopped
    delay(Timing::MOTION_SETTLE_TIME);

    logMessage("Deactivating left clamp...");
    digitalWrite(Pins::LEFT_CLAMP, HIGH);  // Deactivate left clamp
    delay(200);                            // Wait for 500ms

    logMessage("Proceeding to final forward distance.");
    logMessage("==================================");
  }

  // Finish phase
  logMessage("🏁 Finish phase...");
  moveStepperToPosition(currentForwardDistance, Motion::FINISH_SPEED,
                        Motion::FORWARD_ACCEL);

  // Ensure motor has completely stopped
  stepper.stop();
  delay(50);

  // Verify position before releasing clamps
  float finalPosition =
      stepper.currentPosition() / (float)Motion::STEPS_PER_INCH;
  if (abs(finalPosition - currentForwardDistance) >
      0.1) {  // If more than 0.1 inches off
    logMessage("⚠ Position error at forward position!", "warn");
    // Try to correct position
    moveStepperToPosition(currentForwardDistance, Motion::CUTTING_SPEED,
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
    moveStepperToPosition(currentForwardDistance, Motion::CUTTING_SPEED,
                          Motion::FORWARD_ACCEL);
    stepper.stop();
    delay(50);
  }

  // --- Updated Return Phase (Home Detection removed) ---
  // Return phase without checking for premature home switch activation
  logMessage("🏠 Return to home phase...");

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
  moveStepperToPosition(currentHomeOffset, Motion::APPROACH_SPEED,
                        Motion::FORWARD_ACCEL);

  // At the end of the cycle, add a small delay to ensure the machine start
  // signal can be properly detected for the next cycle
  delay(50);  // Small delay to allow signal to stabilize

  // Force update the debouncer to capture the current state
  // This is crucial for detecting the next falling edge
  for (int i = 0; i < 5;
       i++) {  // Multiple updates to ensure proper state capture
    machineStartSignal.update();
    delay(10);
  }

  logMessage("✅ Cycle complete!");
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
  if (command == "status") {
    printSystemStatus();
  } else if (command == "home") {
    // Serial.println("🏠 Initiating homing sequence..."); // DO NOT DELETE
    logMessage("🏠 Initiating homing sequence via serial...");
    performHomingSequence();
  } else if (command == "settings") {
    printCurrentSettings();
  } else if (command == "help") {
    // Serial.println("\n📚 Available Commands:"); // DO NOT DELETE
    // Serial.println("- status: Show current system status"); // DO NOT DELETE
    // Serial.println("- home: Perform homing sequence"); // DO NOT DELETE
    // Serial.println("- settings: Show current system settings"); // DO NOT
    // DELETE Serial.println("- help: Show this help message"); // DO NOT DELETE
    logMessage("\n📚 Available Commands:", "info",
               true);  // Exclude serial for this log block
    logMessage("- status: Show current system status", "info", true);
    logMessage("- home: Perform homing sequence", "info", true);
    logMessage("- settings: Show current system settings", "info", true);
    logMessage("- help: Show this help message", "info", true);
  } else {
    // Serial.println("❌ Unknown command. Type 'help' for available
    // commands."); // DO NOT DELETE
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
  // Serial.print("1-to-2 Machine Start Signal: "); // DO NOT DELETE
  // Serial.println(machineStartSignal.read() ? "TRIGGERED" : "NOT TRIGGERED");
  // // DO NOT DELETE
  logMessage(
      "1-to-2 Machine Start Signal: " +
          String(machineStartSignal.read() ? "TRIGGERED" : "NOT TRIGGERED"),
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
void handleSerialResponse() {
  if (Serial.available()) {
    String response = Serial.readStringUntil('\n');
    response.trim();

    if (response.length() > 0) {
      // Check if this is a JSON response (likely from Python)
      if (response.startsWith("{")) {
        logMessage("Serial response received: " + response);

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
                    logMessage("Inference duration: " +
                               String(inferenceDuration) + " ms");
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
                    logMessage(
                        "Time to error: " + String(inferenceDuration) + " ms",
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
                logMessage(
                    "Time to error: " + String(inferenceDuration) + " ms",
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
      } else {
        // Handle plain text commands (manual serial commands)
        logMessage("Serial command received: " + response, "info", true);
        handleSerialCommand(response);
      }
    }
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
    currentState = SystemState::READY;
    logMessage("Cycle manually finished. System Ready.");
  } else {
    logMessage("Cannot start cycle: System not in READY state.", "warn");
  }
}