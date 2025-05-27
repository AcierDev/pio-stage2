#pragma once

// WiFi credentials and board identification
namespace Config {
extern const char *WIFI_SSID;
extern const char *WIFI_PASSWORD;
// Board identification
extern const char *BOARD_ID;
extern const char *BOARD_DESCRIPTION;
}  // namespace Config

// Motion Parameters
namespace Motion {
extern const int STEPS_PER_INCH;   // Halved from 63 for the 30:80 tooth ratio
extern const float HOME_OFFSET;  // Position value stays the same
extern const float APPROACH_DISTANCE;  // Position value stays the same
extern const float CUTTING_DISTANCE;   // Position value stays the same
extern const float FORWARD_DISTANCE;  // Position value stays the same
extern const float END_DROP_DISTANCE_OFFSET;  // Distance before the forward distance

// Speed Settings (inches/second)
extern const float HOMING_SPEED_IPS;  // 750 steps/sec ÷ 32 steps/inch
extern const float APPROACH_SPEED_IPS;  // 10000 steps/sec ÷ 32 steps/inch
extern const float CUTTING_SPEED_IPS;  // 63 steps/sec ÷ 32 steps/inch
extern const float FINISH_SPEED_IPS;  // 12500 steps/sec ÷ 32 steps/inch
extern const float RETURN_SPEED_IPS;  // 12500 steps/sec ÷ 32 steps/inch

// Speed Settings (steps/second) - For internal use
extern const float HOMING_SPEED;
extern const float APPROACH_SPEED;
extern const float CUTTING_SPEED;
extern const float FINISH_SPEED;
extern const float RETURN_SPEED;

// Acceleration Settings (inches/second^2)
extern const float FORWARD_ACCEL_IPS2;  // 7500 steps/sec^2 ÷ 32 steps/inch
extern const float RETURN_ACCEL_IPS2;   // 7500 steps/sec^2 ÷ 32 steps/inch

// Acceleration Settings (steps/second^2) - For internal use
extern const float FORWARD_ACCEL;
extern const float RETURN_ACCEL;
}  // namespace Motion

// Timing Settings (milliseconds)
namespace Timing {
extern const int CLAMP_ENGAGE_TIME;
extern const int CLAMP_RELEASE_TIME;
extern const int HOME_SETTLE_TIME;
extern const int MOTION_SETTLE_TIME;
extern const int ALIGNMENT_TIME;  // Changed to 300ms for alignment cylinder
extern const int LEFT_CLAMP_PULSE_TIME;  // New constant for left clamp pulse duration
extern const int LEFT_CLAMP_RETRACT_WAIT;
extern const int CLAMP_RELEASE_SETTLE_TIME;
}  // namespace Timing

// Serial communication settings
extern const unsigned long SERIAL_BAUDRATE; 