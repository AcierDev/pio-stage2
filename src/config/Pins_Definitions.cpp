#include "config/Pins_Definitions.h"

// Pin Configuration - Using ESP32-S3 GPIO pins (Freenove ESP32-S3 board)
// Avoiding strapping pins (0,3,45,46), USB pins (19,20), PSRAM pins (35,36,37), JTAG pins (39,40,41,42)
namespace Pins {
// Input pins
const int HOME_SWITCH = 1;                  // Was 22 on ESP32, now using GPIO1 on S3 (ADC1_CH0)
const int START_BUTTON = 2;                 // Was 23 on ESP32, now using GPIO2 on S3 (ADC1_CH1) 
const int TRANSFER_ARM_START_SIGNAL = 47;  // Was 15 on ESP32, now using GPIO47 on S3 ( safe to use)

// Output pins
const int STEP = 38;                       // Was 18 on ESP32, now using GPIO38 on S3 (safe to use)
const int DIR = 37;                        // Was 5 on ESP32, now using GPIO37 on S3 (safe to use)
const int ENABLE = 8;                      // Was 27 on ESP32, now using GPIO8 on S3 (safe to use)
const int LEFT_CLAMP = 46;                  // Was 12 on ESP32, now using GPIO46 on S3 (safe to use)
const int RIGHT_CLAMP = 10;                // Was 13 on ESP32, now using GPIO10 on S3 (safe to use)
const int ALIGN_CYLINDER = 3;             // Was 14 on ESP32, now using GPIO11 on S3 (safe to use)
const int TRANSFER_ARM_SIGNAL = 48;        // Was 2 on ESP32, now using GPIO12 on S3 (safe to use)
}  // namespace Pins 