#pragma once

// Pin Configuration - Using ESP32-S3 GPIO pins (Freenove ESP32-S3 board)
namespace Pins {
// Input pins
inline const int HOME_SWITCH = 1;                  // GPIO1 on S3 (ADC1_CH0)
inline const int START_BUTTON = 2;                 // GPIO2 on S3 (ADC1_CH1) 
inline const int TRANSFER_ARM_START_SIGNAL = 47;  // GPIO47 on S3 (safe to use)

// Output pins
inline const int STEP = 38;                        // GPIO38 on S3 (safe to use)
inline const int DIR = 37;                         // GPIO37 on S3 (safe to use)
inline const int ENABLE = 8;                       // GPIO8 on S3 (safe to use)
inline const int LEFT_CLAMP = 46;                  // GPIO46 on S3 (safe to use)
inline const int RIGHT_CLAMP = 10;                 // GPIO10 on S3 (safe to use)
inline const int ALIGN_CYLINDER = 3;               // GPIO3 on S3 (safe to use)
inline const int TRANSFER_ARM_SIGNAL = 48;         // GPIO48 on S3 (safe to use)
}  // namespace Pins 