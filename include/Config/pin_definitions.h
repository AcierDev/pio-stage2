#pragma once

// Pin Configuration - Using ESP32 GPIO pins
namespace Pins {
// Input pins
extern const int HOME_SWITCH;
extern const int START_BUTTON;
extern const int TRANSFER_ARM_START_SIGNAL;  // Transfer arm start signal
extern const int CAMERA_SIGNAL;             // Camera signal pin

// Output pins
extern const int STEP;
extern const int DIR;
extern const int ENABLE;
extern const int LEFT_CLAMP;
extern const int RIGHT_CLAMP;
extern const int ALIGN_CYLINDER;
extern const int TRANSFER_ARM_SIGNAL;  // Signal to transfer arm to prevent Z-axis lowering during return
}  // namespace Pins 