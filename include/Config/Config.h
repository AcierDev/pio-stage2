#pragma once

// WiFi credentials and board identification
namespace Config {
inline const char *WIFI_SSID = "Everwood";
inline const char *WIFI_PASSWORD = "Everwood-Staff";
inline const char *BOARD_ID = "STAGE2_001";
inline const char *BOARD_DESCRIPTION = "Stage 2 Cutting Machine";
}  // namespace Config

// Motion Parameters
namespace Motion {
inline const float STEPS_PER_INCH = 42.33;   // 200 steps/rev ÷ (60 teeth × 2mm ÷ 25.4mm/inch) = 42.33 steps/inch
inline const float HOME_OFFSET = 1.0;        // Position offset from home switch
inline const float APPROACH_DISTANCE = 8.0;  // Distance to approach cutting position
inline const float CUTTING_DISTANCE = 4.0;   // Distance to cut through material
inline const float FORWARD_DISTANCE = 14.0;  // Total forward distance

// Speed Settings (steps/second)
inline const float HOMING_SPEED = 423;     // ~10 IPS
inline const float APPROACH_SPEED = 12000; // Original high speed
inline const float CUTTING_SPEED = 127;    // ~3 IPS
inline const float FINISH_SPEED = 14000;   // Original high speed
inline const float RETURN_SPEED = 14000;   // Original high speed

// Acceleration Settings (steps/second²)
inline const float FORWARD_ACCEL = 2115;   // ~50 IPS²
inline const float RETURN_ACCEL = 2115;    // ~50 IPS²
}  // namespace Motion

// Timing Settings (milliseconds)
namespace Timing {
inline const int CLAMP_ENGAGE_TIME = 200;
inline const int CLAMP_RELEASE_TIME = 200;
inline const int HOME_SETTLE_TIME = 100;
inline const int MOTION_SETTLE_TIME = 50;
inline const int ALIGNMENT_TIME = 300;
}  // namespace Timing

// Serial communication settings
inline const unsigned long SERIAL_BAUDRATE = 115200; 