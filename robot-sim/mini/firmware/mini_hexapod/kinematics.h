// Hexapod Mini - leg IK and tripod gait.  Plain C++ (no Arduino headers) so the
// host test (../test/test_mini.cpp) can run it on a PC.
//
// Same conventions as the simulator (robot-sim/core/src/ik.rs, gait.rs):
//   leg frame: +x out from the body, +y up, +z "forward" of the leg;
//   coxa = atan2(z, x); femur = elevation above horizontal (+ up);
//   tibia = bend relative to the femur (negative = folded down).
#pragma once
#include <stdint.h>

struct LegAngles {
  float coxa, femur, tibia;   // radians
  bool clamped;               // target out of reach (or past the tibia fold limit)
};

// Inverse kinematics (sim's solve_leg_limited): never folds the tibia past minTibia.
LegAngles solveLeg(float x, float y, float z, float coxaLen, float femurLen, float tibiaLen, float minTibia);
// Forward kinematics, for tests: foot position in the leg frame.
void forwardLeg(const LegAngles &a, float coxaLen, float femurLen, float tibiaLen, float out[3]);

// Tripod gait.  Legs 0, 2, 4 and 1, 3, 5 alternate; each foot follows
//   stance: straight line back through its neutral point, swing: eased forward with a
//   sin lift.  Body velocity (vx left, vz forward, m/s) and yaw rate (rad/s, + = turn
//   left) are scaled down so every stance foot stays inside its stance window.
class TripodGait {
 public:
  TripodGait();
  void setHeight(float h) { height_ = h; }
  float height() const { return height_; }
  void setFast(bool f) { fast_ = f; }
  // dt in s.  Writes 18 joint angles (radians, sim convention) to angles[].
  void update(float dt, float vx, float vz, float yaw, float angles[18]);
  // feet in the body frame (x, y, z) * 6, after the last update
  const float *feet() const { return feet_; }
  uint8_t stanceMask() const { return stance_; }
  float speedScale() const { return scale_; }
  bool atRest() const { return activity_ < 0.02f && restPhase(); }
  // reach of leg i's foot from its coxa axis (horizontal), last update
  float reach(int i) const { return reach_[i]; }

 private:
  bool restPhase() const;
  float neutral_[6][2];
  float yaw_[6];
  float height_ = 0.045f;
  bool fast_ = false;
  float phase_ = 0.0f;
  float activity_ = 0.0f;
  float vel_[3] = {0, 0, 0};
  float scale_ = 1.0f;
  float feet_[18];
  float reach_[6];
  uint8_t stance_ = 0x3f;
};

// joint angles (radians) -> servo degrees (0-180) with DIR and TRIM from config.h
void anglesToServo(const float angles[18], float deg[18]);
