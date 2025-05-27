#include "Config/system_config.h"

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
const float HOME_OFFSET = 3.1f;  // Position value stays the same
const float APPROACH_DISTANCE = 5.0f;  // Position value stays the same
const float CUTTING_DISTANCE = 7.3f;   // Position value stays the same
const float FORWARD_DISTANCE = 29.5f;  // Position value stays the same
const float END_DROP_DISTANCE_OFFSET = 5.0f;  // Distance before the forward distance

// Speed Settings (inches/second)
const float HOMING_SPEED_IPS = 23.4375f;  // 750 steps/sec ÷ 32 steps/inch
const float APPROACH_SPEED_IPS = 312.5f;  // 10000 steps/sec ÷ 32 steps/inch
const float CUTTING_SPEED_IPS = 1.96875f;  // 63 steps/sec ÷ 32 steps/inch
const float FINISH_SPEED_IPS = 390.625f;  // 12500 steps/sec ÷ 32 steps/inch
const float RETURN_SPEED_IPS = 390.625f;  // 12500 steps/sec ÷ 32 steps/inch

// Speed Settings (steps/second) - For internal use
const float HOMING_SPEED = HOMING_SPEED_IPS * STEPS_PER_INCH;
const float APPROACH_SPEED = APPROACH_SPEED_IPS * STEPS_PER_INCH;
const float CUTTING_SPEED = CUTTING_SPEED_IPS * STEPS_PER_INCH;
const float FINISH_SPEED = FINISH_SPEED_IPS * STEPS_PER_INCH;
const float RETURN_SPEED = RETURN_SPEED_IPS * STEPS_PER_INCH;

// Acceleration Settings (inches/second^2)
const float FORWARD_ACCEL_IPS2 = 234.375f;  // 7500 steps/sec^2 ÷ 32 steps/inch
const float RETURN_ACCEL_IPS2 = 234.375f;   // 7500 steps/sec^2 ÷ 32 steps/inch

// Acceleration Settings (steps/second^2) - For internal use
const float FORWARD_ACCEL = FORWARD_ACCEL_IPS2 * STEPS_PER_INCH;
const float RETURN_ACCEL = RETURN_ACCEL_IPS2 * STEPS_PER_INCH;
}  // namespace Motion

// Timing Settings (milliseconds)
namespace Timing {
const int CLAMP_ENGAGE_TIME = 200;
const int CLAMP_RELEASE_TIME = 200;
const int HOME_SETTLE_TIME = 30;
const int MOTION_SETTLE_TIME = 50;
const int ALIGNMENT_TIME = 300;  // Changed to 300ms for alignment cylinder
const int LEFT_CLAMP_PULSE_TIME = 100;  // New constant for left clamp pulse duration
const int LEFT_CLAMP_RETRACT_WAIT = 50;
const int CLAMP_RELEASE_SETTLE_TIME = 100;
}  // namespace Timing

// Serial communication settings
const unsigned long SERIAL_BAUDRATE = 115200; 