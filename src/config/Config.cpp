#include "config/Config.h"

// WiFi credentials and board identification
namespace Config {
const char *WIFI_SSID = "Everwood";
const char *WIFI_PASSWORD = "Everwood-Staff";
// Board identification
const char *BOARD_ID = "STAGE_2_001";
const char *BOARD_DESCRIPTION = "Stage 2 Cutting Controller";
}  // namespace Config

// Motion Parameters
namespace Motion {
const float STEPS_PER_INCH = 42.33f;   // 200 steps/rev ÷ (60 teeth × 2mm ÷ 25.4mm/inch) = 42.33 steps/inch
const float HOME_OFFSET = 0.9f;  // Position value stays the same
const float APPROACH_DISTANCE = 5.0f;  // Position value stays the same
const float CUTTING_DISTANCE = 7.3f;   // Position value stays the same
const float FORWARD_DISTANCE = 26.8f;  // Position value stays the same

// Speed Settings (steps/second) - Fixed for 42.33 steps/inch
const float HOMING_SPEED = 423;         // ~10 IPS (42.33 * 10)
const float APPROACH_SPEED = 1270;      // ~30 IPS (42.33 * 30) - much more reasonable
const float CUTTING_SPEED = 85;         // ~2 IPS - keep slow for cutting
const float FINISH_SPEED = 2115;        // ~50 IPS (42.33 * 50)
const float RETURN_SPEED = 2115;        // ~50 IPS (42.33 * 50)

// Acceleration Settings (steps/second^2) - Fixed for 42.33 steps/inch
const float FORWARD_ACCEL = 2115;       // ~50 IPS^2 (42.33 * 50)
const float RETURN_ACCEL = 2115;        // ~50 IPS^2 (42.33 * 50)
}  // namespace Motion

// Timing Settings (milliseconds)
namespace Timing {
const int CLAMP_EXTEND_TIME = 200;
const int CLAMP_RETRACT_TIME = 200;
const int CLAMP_ENGAGE_TIME = 200;
const int CLAMP_RELEASE_TIME = 200;
const int HOME_SETTLE_TIME = 30;
const int MOTION_SETTLE_TIME = 50;
const int ALIGNMENT_TIME = 300;  // Changed to 300ms for alignment cylinder
const int LEFT_CLAMP_PULSE_TIME = 100;  // New constant for left clamp pulse duration
const int LEFT_CLAMP_RETRACT_WAIT = 50;
const int CLAMP_RETRACT_SETTLE_TIME = 100;
}  // namespace Timing

// Serial communication settings
const unsigned long SERIAL_BAUDRATE = 115200; 