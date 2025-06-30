#include <FastAccelStepper.h>
#include <Arduino.h>
#include <Bounce2.h>

#include "system_states.h"
#include "Config/Config.h"
#include "Config/Pins_Definitions.h"
#include "OTA_Manager.h"
#include "CuttingCycle.h"

//* ************************************************************************
//* ************************ GLOBAL OBJECTS ***************************
//* ************************************************************************

// System objects
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
Bounce homeSwitch = Bounce();
Bounce startButton = Bounce();
Bounce transferArmStartSignal = Bounce();

// System state tracking
SystemState currentState = SystemState::INITIALIZING;
bool isHomed = false;

//* ************************************************************************
//* ************************ FUNCTION DECLARATIONS ***************************
//* ************************************************************************

void initializeHardware();
void performHomingSequence();
void handleSerialCommand(const String &command);
void printCurrentSettings();

// Manual control functions
void manualHome();
void manualJog(bool jogLeft, float distance);
void manualToggleLeftClamp();
void manualToggleRightClamp();
void manualToggleAlignCylinder();
void manualStartCycle();

//* ************************************************************************
//* ************************ SETUP ***************************
//* ************************************************************************

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

//* ************************************************************************
//* ************************ MAIN LOOP ***************************
//* ************************************************************************

void loop() {
  // Handle OTA updates
  handleOTA();

  // Handle serial communication
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() > 0) {
      handleSerialCommand(command);
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
    currentState = SystemState::READY;
  }

  if (transferArmStartSignal.read() == HIGH && currentState == SystemState::READY) {
    currentState = SystemState::CYCLE_RUNNING;
    runCuttingCycle();
    currentState = SystemState::READY;
  }
}

//* ************************************************************************
//* ************************ HARDWARE INITIALIZATION ***************************
//* ************************************************************************

void initializeHardware() {
  // Configure input pins
  pinMode(Pins::HOME_SWITCH, INPUT_PULLDOWN);
  pinMode(Pins::START_BUTTON, INPUT_PULLDOWN);
  pinMode(Pins::TRANSFER_ARM_START_SIGNAL, INPUT_PULLDOWN);

  // Configure output pins
  pinMode(Pins::ENABLE, OUTPUT);
  pinMode(Pins::LEFT_CLAMP, OUTPUT);
  pinMode(Pins::RIGHT_CLAMP, OUTPUT);
  pinMode(Pins::ALIGN_CYLINDER, OUTPUT);
  pinMode(Pins::TRANSFER_ARM_SIGNAL, OUTPUT);

  // Initialize pneumatics
  digitalWrite(Pins::LEFT_CLAMP, LOW);   // Start with clamps engaged
  digitalWrite(Pins::RIGHT_CLAMP, LOW);
  digitalWrite(Pins::ALIGN_CYLINDER, LOW);
  digitalWrite(Pins::TRANSFER_ARM_SIGNAL, LOW);

  // Setup debouncing
  homeSwitch.attach(Pins::HOME_SWITCH);
  homeSwitch.interval(10);
  startButton.attach(Pins::START_BUTTON);
  startButton.interval(20);
  transferArmStartSignal.attach(Pins::TRANSFER_ARM_START_SIGNAL);
  transferArmStartSignal.interval(50);

  // Initialize stepper
  if (stepper) {
    stepper->setSpeedInHz(Motion::APPROACH_SPEED);
    stepper->setAcceleration(Motion::FORWARD_ACCEL);
  }
}

//* ************************************************************************
//* ************************ HOMING SEQUENCE ***************************
//* ************************************************************************

void performHomingSequence() {
  currentState = SystemState::HOMING;
  if (!stepper) return;

  Serial.println("=== HOMING SEQUENCE START ===");

  // Move away from home switch first
  stepper->setSpeedInHz(Motion::HOMING_SPEED);
  stepper->setAcceleration(Motion::FORWARD_ACCEL);
  stepper->move(-10000);  // Move away from home

  // Slow down as we approach home
  float slowHomingSpeed = Motion::HOMING_SPEED / 3;

  while (stepper->isRunning()) {
    homeSwitch.update();

    // Slow down when getting close to home
    if (abs(stepper->getCurrentPosition()) < 2000) {
      stepper->setSpeedInHz(slowHomingSpeed);
    }

    if (homeSwitch.read() == HIGH) {
      stepper->forceStopAndNewPosition(0);
      break;
    }
    delay(1);
  }

  // Check if homing was successful
  if (homeSwitch.read() == LOW) {
    Serial.println("HOMING FAILED!");
    currentState = SystemState::ERROR;
    return;
  }

  // Move to home offset
  stepper->setSpeedInHz(Motion::HOMING_SPEED / 2);
  stepper->setAcceleration(Motion::FORWARD_ACCEL / 2);
  stepper->moveTo(Motion::HOME_OFFSET * Motion::STEPS_PER_INCH);

  while (stepper->isRunning()) {
    delay(1);
  }

  isHomed = true;
  Serial.println("=== HOMING COMPLETE ===");
  Serial.println("Position: " + String(stepper->getCurrentPosition() / Motion::STEPS_PER_INCH) + " inches");
}

//* ************************************************************************
//* ************************ SERIAL COMMAND HANDLING ***************************
//* ************************************************************************

void handleSerialCommand(const String &command) {
  if (command.equalsIgnoreCase("home")) {
    manualHome();
  } else if (command.startsWith("jog")) {
    // Parse jog command: "jog left 1.5" or "jog right 2.0"
    int firstSpace = command.indexOf(' ');
    int secondSpace = command.indexOf(' ', firstSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0) {
      String direction = command.substring(firstSpace + 1, secondSpace);
      float distance = command.substring(secondSpace + 1).toFloat();
      
      if (direction.equalsIgnoreCase("left")) {
        manualJog(true, distance);
      } else if (direction.equalsIgnoreCase("right")) {
        manualJog(false, distance);
      }
    }
  } else if (command.equalsIgnoreCase("leftclamp")) {
    manualToggleLeftClamp();
  } else if (command.equalsIgnoreCase("rightclamp")) {
    manualToggleRightClamp();
  } else if (command.equalsIgnoreCase("align")) {
    manualToggleAlignCylinder();
  } else if (command.equalsIgnoreCase("cycle")) {
    manualStartCycle();
  } else if (command.equalsIgnoreCase("status")) {
    printCurrentSettings();
  } else {
    Serial.println("Unknown command: " + command);
    Serial.println("Available commands: home, jog [left/right] [distance], leftclamp, rightclamp, align, cycle, status");
  }
}

void printCurrentSettings() {
  Serial.println("\n=== SYSTEM STATUS ===");
  Serial.println("State: " + String(static_cast<int>(currentState)));
  Serial.println("Homed: " + String(isHomed ? "YES" : "NO"));
  
  if (stepper) {
    float currentPos = stepper->getCurrentPosition() / Motion::STEPS_PER_INCH;
    Serial.println("Position: " + String(currentPos) + " inches");
    Serial.println("Speed: " + String(stepper->getCurrentSpeedInUs()) + " us/step");
  }
  
  Serial.println("Home Switch: " + String(homeSwitch.read() ? "ACTIVE" : "INACTIVE"));
  Serial.println("Start Button: " + String(startButton.read() ? "PRESSED" : "RELEASED"));
  Serial.println("Transfer Arm Signal: " + String(transferArmStartSignal.read() ? "ACTIVE" : "INACTIVE"));
  Serial.println("===================\n");
}

//* ************************************************************************
//* ************************ MANUAL CONTROL FUNCTIONS ***************************
//* ************************************************************************

void manualHome() {
  if (currentState != SystemState::READY) {
    Serial.println("Cannot home - system not ready");
    return;
  }
  performHomingSequence();
  currentState = SystemState::READY;
}

void manualJog(bool jogLeft, float distance) {
  if (!stepper || currentState != SystemState::READY) {
    Serial.println("Cannot jog - stepper not available or system not ready");
    return;
  }

  float currentPos = stepper->getCurrentPosition() / Motion::STEPS_PER_INCH;
  float targetPos = jogLeft ? currentPos - distance : currentPos + distance;
  
  Serial.println("Jogging " + String(jogLeft ? "left" : "right") + " " + String(distance) + " inches");
  Serial.println("From " + String(currentPos) + " to " + String(targetPos));

  stepper->setSpeedInHz(Motion::HOMING_SPEED);
  stepper->setAcceleration(Motion::FORWARD_ACCEL);
  stepper->moveTo(targetPos * Motion::STEPS_PER_INCH);

  while (stepper->isRunning()) {
    delay(1);
  }

  Serial.println("Jog complete. Position: " + String(stepper->getCurrentPosition() / Motion::STEPS_PER_INCH) + " inches");
}

void manualToggleLeftClamp() {
  static bool leftClampEngaged = true;
  leftClampEngaged = !leftClampEngaged;
  digitalWrite(Pins::LEFT_CLAMP, leftClampEngaged ? LOW : HIGH);
  Serial.println("Left clamp: " + String(leftClampEngaged ? "ENGAGED" : "RELEASED"));
}

void manualToggleRightClamp() {
  static bool rightClampEngaged = true;
  rightClampEngaged = !rightClampEngaged;
  digitalWrite(Pins::RIGHT_CLAMP, rightClampEngaged ? LOW : HIGH);
  Serial.println("Right clamp: " + String(rightClampEngaged ? "ENGAGED" : "RELEASED"));
}

void manualToggleAlignCylinder() {
  static bool alignExtended = false;
  alignExtended = !alignExtended;
  digitalWrite(Pins::ALIGN_CYLINDER, alignExtended ? HIGH : LOW);
  Serial.println("Alignment cylinder: " + String(alignExtended ? "EXTENDED" : "RETRACTED"));
}

void manualStartCycle() {
  if (currentState != SystemState::READY) {
    Serial.println("Cannot start cycle - system not ready");
    return;
  }
  
  Serial.println("Starting manual cutting cycle...");
  currentState = SystemState::CYCLE_RUNNING;
  runCuttingCycle();
  currentState = SystemState::READY;
}