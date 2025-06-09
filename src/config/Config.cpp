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
const int STEPS_PER_INCH = 43;   // Halved from 63 for the 30:80 tooth ratio
const float HOME_OFFSET = 1.5f;  // Position value stays the same
const float APPROACH_DISTANCE = 5.0f;  // Position value stays the same
const float CUTTING_DISTANCE = 7.4f;   // Position value stays the same
const float FORWARD_DISTANCE = 27.0f;  // Position value stays the same
const float END_DROP_DISTANCE_OFFSET = 5.0f;  // Distance before the forward distance

// Speed Settings (steps/second)
const float HOMING_SPEED = 1000;    // Reduced for reliable homing
const float APPROACH_SPEED = 8000;    // Reduced from 14000 for stability  
const float CUTTING_SPEED = 70;    // Doubled for better cutting motion
const float FINISH_SPEED = 10000;    // Reduced from 17000 for stability
const float RETURN_SPEED = 12000;    // Different speed for return

// Acceleration Settings (steps/second^2)
const float FORWARD_ACCEL = 10000;  // Doubled for better responsiveness
const float RETURN_ACCEL = 10000;   // Higher for return movement
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