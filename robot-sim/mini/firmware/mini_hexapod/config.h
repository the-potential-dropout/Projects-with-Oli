// Hexapod Mini - build configuration.  Geometry matches cad/mini_hexapod_fusion.py
// (the user parameters there have the same names) and docs/DESIGN.md.
#pragma once
#include <stdint.h>

// ---------------------------------------------------------------- geometry (metres)
// Sim conventions (robot-sim/core/src/ik.rs, gait.rs): body frame x = robot LEFT,
// y = UP, z = FORWARD; leg frame x = out, y = up, z = "forward" of the leg.
// Leg order: 0 front-right, 1 middle-right, 2 rear-right, 3 front-left, 4 middle-left, 5 rear-left.
static const float LEG_COXA = 0.030f;    // coxa axis -> femur axis
static const float LEG_FEMUR = 0.044f;   // femur axis -> tibia axis
static const float LEG_TIBIA = 0.060f;   // tibia axis -> foot tip
// coxa axes in the body frame (x_sim = -X_fusion, z_sim = Y_fusion) and leg direction
// (Fusion degrees from +X towards +Y, as in the CAD's MOUNTS)
static const float MOUNT_X[6] = {-0.048f, -0.060f, -0.048f, 0.048f, 0.060f, 0.048f};
static const float MOUNT_Z[6] = {0.072f, 0.000f, -0.072f, 0.072f, 0.000f, -0.072f};
static const float MOUNT_DIR_DEG[6] = {40.0f, 0.0f, -40.0f, 140.0f, 180.0f, -140.0f};

// ---------------------------------------------------------------- gait
static const float STAND_HEIGHT = 0.045f;     // hip plane above the floor
static const float HEIGHT_MIN = 0.035f, HEIGHT_MAX = 0.055f;
static const float FOOT_REACH = 0.058f;       // coxa axis -> neutral foot
// Stance window (load check, docs/DESIGN.md section 4): stance feet stay this far from
// their coxa axis.  The lone middle foot of a tripod carries 44 % of the weight, so the
// middle legs get the tighter window.
static const float REACH_MIN = 0.052f;
static const float REACH_MAX_MIDDLE = 0.061f;
static const float REACH_MAX_CORNER = 0.070f;
static const float STEP_HEIGHT = 0.015f;
static const float STEP_HZ_WALK = 2.0f;
static const float STEP_HZ_FAST = 2.8f;
static const float V_MAX = 0.10f;             // m/s at full stick (the stance window may slow it)
static const float W_MAX = 0.9f;              // rad/s at full stick
static const float SIT_HEIGHT = 0.026f;       // hip plane when sitting: the battery hatch is 27 mm below it

// joint limits, sim convention (degrees): + femur = up, tibia relative to the femur
static const float LIM_COXA = 25.0f;   // neighbouring legs touch beyond this (cad sweep); the gait uses <= 16
static const float LIM_FEMUR_MIN = -35.0f, LIM_FEMUR_MAX = 55.0f;
static const float LIM_TIBIA_MIN = -130.0f, LIM_TIBIA_MAX = -40.0f;

// ---------------------------------------------------------------- servos
// PCA9685 boards: right 0x40 (legs 0-2, A0 open), left 0x41 (legs 3-5, A0 bridged).
static const uint8_t PCA_ADDR[2] = {0x40, 0x41};
// channel of (leg, joint) on its own board - must match the harness in the CAD (CHANNEL)
static const uint8_t CHANNEL[6][3] = {{0, 1, 2}, {6, 7, 8}, {15, 14, 13}, {15, 14, 13}, {9, 8, 7}, {0, 1, 2}};
// servo degrees = 90 + DIR * joint angle (coxa, femur) and 90 + DIR * (tibia + 90).
// All legs are the same printed parts turned round the body (no mirrored parts), so the
// signs are the same for every leg.  Assumes the MG90S turns counter-clockwise (seen from
// the horn) as the pulse gets longer; BUILD.md step 9 checks it - flip a sign if a joint
// moves the wrong way.
static const int8_t DIR[3] = {-1, -1, -1};
// per-servo trim in degrees, set on the stand (BUILD.md step 9)
static float TRIM[6][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
static const float SERVO_US_MIN = 500.0f;    // pulse at 0 deg
static const float SERVO_US_MAX = 2500.0f;   // pulse at 180 deg
static const float SERVO_DEG_MIN = 5.0f, SERVO_DEG_MAX = 175.0f;   // never drive into the end stops

// ---------------------------------------------------------------- pins (ESP32-DevKitC)
static const int PIN_SDA = 21;
static const int PIN_SCL = 22;
static const int PIN_OE = 4;        // PCA9685 OE on both boards: HIGH = all servo pulses off (limp)
static const int PIN_VBAT = 34;     // battery through 100k / 47k (8.4 V -> 2.69 V)
static const int PIN_LED = 2;       // on-board LED

// ---------------------------------------------------------------- battery (2S Li-ion)
static const float VBAT_DIVIDER = (100.0f + 47.0f) / 47.0f;
static const float VBAT_CAL = 1.00f;        // trim against a multimeter (BUILD.md step 7)
static const float VBAT_CUTOFF = 6.6f;      // 3.3 V/cell under load, held for 3 s -> sit and go limp
static const float VBAT_RECOVER = 7.0f;     // must read above this before it will re-arm
static const float VBAT_WARN = 6.9f;        // rumble / LED warning

// ---------------------------------------------------------------- timing
static const float LOOP_HZ = 50.0f;
static const uint32_t PAD_STOP_MS = 300;    // no gamepad data for this long -> stop walking
static const uint32_t PAD_SIT_MS = 3000;    // ... this long -> sit down and go limp
static const uint32_t USB_RELAX_MS = 500;   // USB S-frames stop -> go limp
