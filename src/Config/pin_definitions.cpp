#include "Config/pin_definitions.h"

// Pin Configuration - Using ESP32 GPIO pins
namespace Pins {
// Input pins
const int HOME_SWITCH = 22;
const int START_BUTTON = 23;
const int TRANSFER_ARM_START_SIGNAL = 15;  // Transfer arm start signal
const int CAMERA_SIGNAL = 34;             // Camera signal pin

// Output pins
const int STEP = 18;
const int DIR = 5;
const int ENABLE = 27;
const int LEFT_CLAMP = 13;
const int RIGHT_CLAMP = 12;
const int ALIGN_CYLINDER = 14;
const int TRANSFER_ARM_SIGNAL = 2;  // Signal to transfer arm to prevent Z-axis lowering during return
}  // namespace Pins 