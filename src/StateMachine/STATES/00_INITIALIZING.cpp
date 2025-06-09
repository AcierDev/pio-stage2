#include "StateMachine/STATES/00_INITIALIZING.h"
#include "system_states.h"
#include "config/Config.h"
#include "config/Pins_Definitions.h"

//* ************************************************************************
//* ************************ INITIALIZING ***************************
//* ************************************************************************
// This state handles the initial hardware setup and system initialization
// before proceeding to homing and operations

void executeInitializingState() {
    //! Execute complete initialization sequence
    logMessage("✅ Starting hardware initialization...");
    initializeHardware();
    logMessage("✅ Hardware initialized");
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
}

bool isInitializationComplete() {
    //! Check if initialization sequence is complete
    return true;  // Hardware initialization is synchronous
}

void resetInitializingState() {
    //! Reset initialization state to initial conditions
    // Reset any initialization flags if needed
} 