// Hexapod Mk1 rev B2 - onboard controller configuration.
// Board: Adafruit Feather ESP32-S3 (#5323 / #5477), Arduino core "Adafruit Feather ESP32-S3".
// Geometry, limits and gait numbers mirror src/robots/hexapod.config.js: keep both in step.
#pragma once
#include <stdint.h>

namespace cfg {

// ---------------------------------------------------------------- geometry (m)
// Frame = the simulator's body frame: +X left, +Y up, +Z forward, origin at the
// coxa-joint height in the middle of the body.
constexpr float COXA = 0.045f, FEMUR = 0.075f, TIBIA = 0.120f;
constexpr float STAND_HEIGHT = 0.090f;      // coxa-joint height above the ground
constexpr float FOOT_REACH = 0.100f;        // neutral foot distance from the coxa axis
constexpr float FOOT_REACH_MIN = 0.080f;    // load-check stance window (cad/load_calc.py)
constexpr float FOOT_REACH_MAX = 0.115f;
// legs: 0 FR, 1 MR, 2 RR, 3 FL, 4 ML, 5 RL ; mounts [x, z]; splay deg
constexpr float MOUNT[6][2] = {{-0.06f, 0.11f}, {-0.08f, 0.0f}, {-0.06f, -0.11f},
                               {0.06f, 0.11f},  {0.08f, 0.0f},  {0.06f, -0.11f}};
constexpr float SPLAY_DEG[6] = {35, 0, -35, 35, 0, -35};

// ---------------------------------------------------------------- joint limits (sim degrees)
constexpr float COXA_LIM[2] = {-45, 45};
constexpr float FEMUR_LIM[2] = {-10, 70};
constexpr float TIBIA_LIM[2] = {-135, 30};
constexpr float TIBIA_FOLD_WHEN_FEMUR_DOWN = -132;

// ---------------------------------------------------------------- servos
constexpr uint8_t BUS_ID[6][3] = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}, {13, 14, 15}, {16, 17, 18}};
constexpr uint8_t AUX_ID[5] = {21, 22, 23, 24, 25};
constexpr int8_t DIR[6][3] = {{1, -1, -1}, {1, -1, -1}, {1, -1, -1}, {1, 1, 1}, {1, 1, 1}, {1, 1, 1}};
constexpr int8_t TRIM[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
constexpr uint32_t BUS_BAUD = 1000000;
// STS position-loop settings (registers 21 P, 22 D, 23 I on ST/STS firmware;
// confirm on your units - see CONTROL.md section 3)
constexpr uint8_t KP_STIFF = 32, KD_STIFF = 32, KI_STIFF = 0;     // factory default, walking on flat ground
constexpr uint8_t KP_SOFT = 16, KD_SOFT = 24;                      // terrain / climbing: softer, absorbs contact errors
constexpr uint16_t SPEED_SOFTSTART = 600, SPEED_RUN = 2400, ACC_RUN = 60;   // steps/s, steps/s^2/100

// ---------------------------------------------------------------- gait (sim defaults)
constexpr float SPEED_WALK = 0.26f, SPEED_SPRINT = 0.48f, TURN_RATE = 1.6f;
constexpr float STEP_HZ_WALK = 1.7f, STEP_HZ_SPRINT = 2.6f;
constexpr float STEP_HEIGHT = 0.025f, STEP_HEIGHT_TERRAIN = 0.045f;    // lift is also capped by the tibia fold limit
constexpr float TERRAIN_HEIGHT = 0.105f;                                 // body height in terrain mode (more lift, CONTROL.md 8)
constexpr float CMD_SMOOTH = 8.0f;                                  // 1/s, first-order command filter (same as the sim)

// ---------------------------------------------------------------- body pose limits
constexpr float POSE_ROLL_MAX = 15.0f, POSE_PITCH_MAX = 15.0f, POSE_YAW_MAX = 20.0f;   // deg
constexpr float HEIGHT_MIN = 0.060f, HEIGHT_MAX = 0.110f;           // m
constexpr float SHIFT_MAX = 0.025f;                                 // m

// ---------------------------------------------------------------- balance (IMU leveling)
constexpr float BAL_KP = 0.6f, BAL_KI = 2.0f, BAL_KD = 0.05f;      // deg of body correction per deg of error (per s for KI)
constexpr float BAL_I_MAX = 15.0f;                                  // deg, integrator contribution clamp (anti-windup)
constexpr float BAL_D_LPF_HZ = 8.0f;
constexpr float BAL_RATE_MAX = 60.0f;                               // deg/s slew limit on the correction
constexpr float MARGIN_MIN = 0.025f;                                // m, static stability margin floor

// ---------------------------------------------------------------- terrain
constexpr float TD_SEARCH = 0.050f;      // max reach down below the planned touchdown height (step-down limit)
constexpr float TD_SEARCH_SPEED = 0.15f; // m/s descent while searching
constexpr float STEP_UP_MAX = 0.040f;    // geometric step-up limit in terrain mode (CONTROL.md 8)
constexpr float SLOPE_MAX_DEG = 20.0f;
constexpr float TIP_DEG = 35.0f;         // fallen / tipped threshold

// ---------------------------------------------------------------- power and safety
constexpr float VBAT_LVC = 10.2f, VBAT_RECOVER = 11.0f;
constexpr uint32_t LVC_DEBOUNCE_MS = 3000;
constexpr uint32_t HOST_TIMEOUT_MS = 500, RC_TIMEOUT_MS = 250;
constexpr uint8_t SERVO_TEMP_WARN = 65, SERVO_TEMP_STOP = 75;      // deg C

// ---------------------------------------------------------------- loop rates
constexpr uint32_t CONTROL_HZ = 100;     // IK + gait + pose + balance + servo sync-write
constexpr uint32_t IMU_HZ = 200, FEET_HZ = 500, TELEM_HZ = 10, SERVO_READ_HZ = 20;

}  // namespace cfg

// ---------------------------------------------------------------- pins (Feather ESP32-S3 labels)
#ifdef ARDUINO
namespace pins {
constexpr int BUS_TX = TX, BUS_RX = RX;           // UART1 -> Waveshare bus bridge (GPIO21/22 on that board)
constexpr int CRSF_TX = 11, CRSF_RX = 12;         // UART2 -> ELRS receiver (D11 -> RX pad, D12 <- TX pad)
constexpr int IMU_INT = MOSI, IMU_RST = MISO;     // BNO085 on I2C (STEMMA QT), interrupt + reset
constexpr int FOOT[6] = {5, 6, 9, 10, SCK, A4};   // foot switches to GND, internal pull-ups
constexpr int J1_AUX1 = A2, J1_AUX2 = A3;         // weapon fire / aux, or UART TX/RX to the brain module
constexpr int SONAR_TRIG = A0, SONAR_ECHO = A1;   // HC-SR04 (ECHO via 10k/20k divider)
constexpr int STATUS_LED = 13;                    // red LED; A5 is spare (J1 / future sensor)
}  // namespace pins
#endif
