// Leg kinematics, gait engine, body pose and balance for Hexapod Mk1.
// Plain C++ (no Arduino calls): the same file builds on the robot and in the
// host tests (firmware/test), and mirrors robot-sim/core/src/{ik,gait}.rs so the
// robot and the simulator compute identical joint angles.
#pragma once
#include <math.h>
#include <stdint.h>

#include "config.h"

namespace hk {

constexpr float PI_F = 3.14159265358979f;
constexpr float DEG = PI_F / 180.0f;

struct Vec3 {
  float x, y, z;
};

struct LegAngles {
  float coxa, femur, tibia;  // rad, sim convention (femur + up, tibia relative, -90 deg = vertical)
  bool clamped;
};

// ---------------------------------------------------------------- IK / FK (leg frame: +X out, +Y up, +Z forward)
inline LegAngles solveLeg(float x, float y, float z, float minTibia = -PI_F) {
  using namespace cfg;
  LegAngles a{};
  a.coxa = atan2f(z, x);
  float horiz = sqrtf(x * x + z * z) - COXA;
  float d = sqrtf(horiz * horiz + y * y);
  float kMin = fmaxf(PI_F + minTibia, 0.0f);
  float foldD = sqrtf(fmaxf(FEMUR * FEMUR + TIBIA * TIBIA - 2 * FEMUR * TIBIA * cosf(kMin), 0.0f));
  float minD = fmaxf(foldD, fabsf(FEMUR - TIBIA)) + 1e-4f, maxD = FEMUR + TIBIA - 1e-4f;
  a.clamped = false;
  if (d > maxD) { d = maxD; a.clamped = true; }
  else if (d < minD) { d = minD; a.clamped = true; }
  float toFoot = atan2f(y, horiz);
  float cA = fminf(fmaxf((FEMUR * FEMUR + d * d - TIBIA * TIBIA) / (2 * FEMUR * d), -1.f), 1.f);
  a.femur = toFoot + acosf(cA);
  float cK = fminf(fmaxf((FEMUR * FEMUR + TIBIA * TIBIA - d * d) / (2 * FEMUR * TIBIA), -1.f), 1.f);
  a.tibia = acosf(cK) - PI_F;
  return a;
}

inline Vec3 forwardLeg(const LegAngles& a) {
  using namespace cfg;
  float r = COXA + FEMUR * cosf(a.femur) + TIBIA * cosf(a.femur + a.tibia);
  float y = FEMUR * sinf(a.femur) + TIBIA * sinf(a.femur + a.tibia);
  return {r * cosf(a.coxa), y, r * sinf(a.coxa)};
}

// Joint-limit clamp in sim degrees, including the coupled tibia limit.
inline void clampJoints(LegAngles& a) {
  using namespace cfg;
  auto cl = [](float v, const float* lim) { return fminf(fmaxf(v, lim[0] * DEG), lim[1] * DEG); };
  a.coxa = cl(a.coxa, COXA_LIM);
  a.femur = cl(a.femur, FEMUR_LIM);
  float tmin = (a.femur < 0) ? TIBIA_FOLD_WHEN_FEMUR_DOWN * DEG : TIBIA_LIM[0] * DEG;
  a.tibia = fminf(fmaxf(a.tibia, tmin), TIBIA_LIM[1] * DEG);
}

// Simulator angle -> servo degrees (0..180, 90 = centre); same map as legToServo() in hexapod.config.js.
inline void toServoDeg(int leg, const LegAngles& a, float out[3]) {
  using namespace cfg;
  out[0] = 90 + DIR[leg][0] * (a.coxa / DEG) + TRIM[leg][0];
  out[1] = 90 + DIR[leg][1] * (a.femur / DEG) + TRIM[leg][1];
  out[2] = 90 + DIR[leg][2] * (a.tibia / DEG + 90) + TRIM[leg][2];
  for (int k = 0; k < 3; k++) out[k] = fminf(fmaxf(out[k], 0.f), 180.f);
}

// Servo degrees -> STS position step (4096 per turn, 2048 = centre).
inline int16_t degToStep(float deg) { return (int16_t)lroundf(2048.0f + (deg - 90.0f) * 4096.0f / 360.0f); }

// ---------------------------------------------------------------- body pose
struct Pose {
  float roll = 0, pitch = 0, yaw = 0;   // rad (roll about +Z forward, pitch about +X left, yaw about +Y)
  float height = cfg::STAND_HEIGHT;     // coxa-joint height above the ground
  float shiftX = 0, shiftZ = 0;         // m, body shift (left, forward)
};

// Rotate a body-frame point by the pose (R = Ry(yaw) Rx(pitch) Rz(roll)).
inline Vec3 rotate(const Pose& p, Vec3 v) {
  float cr = cosf(p.roll), sr = sinf(p.roll), cp = cosf(p.pitch), sp = sinf(p.pitch), cy = cosf(p.yaw), sy = sinf(p.yaw);
  // roll about Z (forward): x,y
  Vec3 a{cr * v.x - sr * v.y, sr * v.x + cr * v.y, v.z};
  // pitch about X (left): y,z  (+pitch = nose up)
  Vec3 b{a.x, cp * a.y + sp * a.z, -sp * a.y + cp * a.z};
  // yaw about Y (up): z,x
  return {cy * b.x + sy * b.z, b.y, -sy * b.x + cy * b.z};
}

// ---------------------------------------------------------------- stability
// Static stability margin: shortest distance from the CoM ground projection
// (x, z) to the edges of the support polygon formed by the stance feet.
// Returns < 0 when the CoM is outside.  feet: [x, z] * n in body frame, in order round the body.
float stabilityMargin(const float feet[][2], int n, float comX, float comZ);

// ---------------------------------------------------------------- gait engine
enum class GaitType : uint8_t { Tripod = 0, Ripple = 1, Wave = 2 };

struct GaitInput {
  float vx = 0, vz = 0, yawRate = 0;    // body velocities (m/s, rad/s): +X left, +Z forward, + = turn left
  Pose pose;                            // commanded body pose (after the balance controller)
  GaitType type = GaitType::Tripod;
  float stepHz = cfg::STEP_HZ_WALK;
  float stepHeight = cfg::STEP_HEIGHT;
  bool terrain = false;                 // enable touchdown search / per-leg height adaptation
  bool contact[6] = {false, false, false, false, false, false};   // foot switches
  // measured body attitude (IMU, rad). When set, touchdown heights are taken in the true
  // level frame; otherwise a body that isn't where it was told to be learns its own tilt.
  bool hasAttitude = false;
  float attRoll = 0, attPitch = 0;
};

struct Gait {
  float phase = 0, activity = 0;
  float vel[3] = {0, 0, 0};
  Vec3 neutral[6];                      // neutral foot positions, body frame (ground at y = -height)
  Vec3 feet[6];                         // current foot targets, level (commanded) frame
  Vec3 feetBody[6];                     // last posed body-frame targets after the lift cap (sent to IK)
  float groundY[6];                     // per-leg terrain height offset learned at touchdown (m, + = higher ground)
  float terrainShift = 0;               // accumulated mean ground change; the caller adds it to the body height
  bool stance[6];
  LegAngles angles[6];
  uint8_t clampedMask = 0;
  float mountYaw[6];

  void init();
  // dt in s.  Returns the joint angles in `angles` (sim convention, limits applied).
  void update(float dt, const GaitInput& in);
  float touchdownHeight(int leg, const GaitInput& in) const;   // foot height below the origin, true level frame
  float legPhase(int leg) const;        // 0..1, swing during [0, duty) for that leg
  float duty(GaitType t) const { return t == GaitType::Tripod ? 0.5f : (t == GaitType::Ripple ? 0.333f : 0.1667f); }
  GaitType current = GaitType::Tripod;
};

// ---------------------------------------------------------------- balance controller
// Levels the body: PI on the IMU roll/pitch error plus D on the gyro rate,
// output = extra body roll/pitch added to the commanded pose (rad).
struct Balance {
  float iRoll = 0, iPitch = 0, dRoll = 0, dPitch = 0, outRoll = 0, outPitch = 0;
  bool enabled = true;
  void reset() { iRoll = iPitch = dRoll = dPitch = outRoll = outPitch = 0; }
  // target: desired world roll/pitch of the body (usually 0 = level); meas: IMU roll/pitch (rad);
  // rate: gyro roll/pitch rates (rad/s); returns the correction in `outRoll/outPitch`.
  void update(float dt, float targetRoll, float targetPitch, float measRoll, float measPitch, float rateRoll, float ratePitch);
};

}  // namespace hk
