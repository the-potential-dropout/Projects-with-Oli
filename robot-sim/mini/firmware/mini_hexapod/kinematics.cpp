// Hexapod Mini - leg IK and tripod gait (see kinematics.h).
#include "kinematics.h"
#include "config.h"
#include <math.h>

static const float PI_F = 3.14159265f;
static const float DEG = PI_F / 180.0f;

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

LegAngles solveLeg(float x, float y, float z, float c, float f, float t, float minTibia) {
  LegAngles a;
  a.coxa = atan2f(z, x);
  float horiz = sqrtf(x * x + z * z) - c;
  float d = sqrtf(horiz * horiz + y * y);
  // knee interior angle k = pi + tibia;  d^2 = f^2 + t^2 - 2 f t cos(k)
  float kMin = PI_F + minTibia;
  if (kMin < 0) kMin = 0;
  float foldD = sqrtf(fmaxf(0.0f, f * f + t * t - 2.0f * f * t * cosf(kMin)));
  float minD = fmaxf(foldD, fabsf(f - t)) + 1e-4f;
  float maxD = f + t - 1e-4f;
  a.clamped = false;
  if (d > maxD) { d = maxD; a.clamped = true; }
  else if (d < minD) { d = minD; a.clamped = true; }
  float toFoot = atan2f(y, horiz);
  float cosA = clampf((f * f + d * d - t * t) / (2.0f * f * d), -1.0f, 1.0f);
  a.femur = toFoot + acosf(cosA);
  float cosK = clampf((f * f + t * t - d * d) / (2.0f * f * t), -1.0f, 1.0f);
  a.tibia = acosf(cosK) - PI_F;
  return a;
}

void forwardLeg(const LegAngles &a, float c, float f, float t, float out[3]) {
  float r = c + f * cosf(a.femur) + t * cosf(a.femur + a.tibia);
  out[1] = f * sinf(a.femur) + t * sinf(a.femur + a.tibia);
  out[0] = r * cosf(a.coxa);
  out[2] = r * sinf(a.coxa);
}

TripodGait::TripodGait() {
  for (int i = 0; i < 6; i++) {
    // Fusion leg direction d (from +X right towards +Y forward) -> sim yaw (from +x left towards +z forward)
    yaw_[i] = PI_F - MOUNT_DIR_DEG[i] * DEG;
    neutral_[i][0] = MOUNT_X[i] + FOOT_REACH * cosf(yaw_[i]);
    neutral_[i][1] = MOUNT_Z[i] + FOOT_REACH * sinf(yaw_[i]);
    feet_[i * 3] = neutral_[i][0];
    feet_[i * 3 + 1] = -height_;
    feet_[i * 3 + 2] = neutral_[i][1];
    reach_[i] = FOOT_REACH;
  }
  height_ = STAND_HEIGHT;
}

static const float OFFS[6] = {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f};
static const float SWING = 0.5f;

bool TripodGait::restPhase() const {
  float n = phase_ / SWING;
  return (n - floorf(n)) < 1e-4f;
}

void TripodGait::update(float dt, float vx, float vz, float w, float angles[18]) {
  float hz = fast_ ? STEP_HZ_FAST : STEP_HZ_WALK;
  float target = (fabsf(vx) + fabsf(vz) > 1e-4f || fabsf(w) > 1e-3f) ? 1.0f : 0.0f;
  // first-order command filter (~0.15 s) so a jabbed stick does not jerk the legs
  float k = fminf(1.0f, dt / 0.15f);
  vel_[0] += (vx - vel_[0]) * k;
  vel_[1] += (vz - vel_[1]) * k;
  vel_[2] += (w - vel_[2]) * k;
  activity_ += (target - activity_) * fminf(1.0f, dt / 0.2f);
  float stanceTime = (1.0f - SWING) / hz;

  // stance-window rule: scale the velocity so no stance foot leaves [REACH_MIN, REACH_MAX]
  float scale = 1.0f;
  for (int i = 0; i < 6; i++) {
    float nx = neutral_[i][0], nz = neutral_[i][1];
    float hx = (vel_[0] + vel_[2] * nz) * stanceTime * 0.5f;
    float hzz = (vel_[1] - vel_[2] * nx) * stanceTime * 0.5f;
    float rmax = (i == 1 || i == 4) ? REACH_MAX_MIDDLE : REACH_MAX_CORNER;
    for (int sgn = -1; sgn <= 1; sgn += 2) {
      // largest s in [0, 1] that keeps the foot inside the window (bisection)
      float lo = 0.0f, hi = 1.0f;
      for (int it = 0; it < 20; it++) {
        float s = 0.5f * (lo + hi);
        float fx = nx + sgn * hx * s - MOUNT_X[i];
        float fz = nz + sgn * hzz * s - MOUNT_Z[i];
        float r = sqrtf(fx * fx + fz * fz);
        if (r > rmax || r < REACH_MIN) hi = s; else lo = s;
      }
      float fx = nx + sgn * hx - MOUNT_X[i], fz = nz + sgn * hzz - MOUNT_Z[i];
      float r = sqrtf(fx * fx + fz * fz);
      if (r > rmax || r < REACH_MIN) scale = fminf(scale, lo);
    }
  }
  scale_ = scale;

  // phase: keep stepping while active; when the sticks are released finish the current
  // half-cycle so every foot ends on the ground
  float next = phase_ + dt * hz;
  if (activity_ > 0.02f) {
    phase_ = next - floorf(next);
  } else if (!restPhase()) {
    float n = 1.0f / SWING;
    if (floorf(next * n) > floorf(phase_ * n)) phase_ = fmodf(floorf(next * n) / n, 1.0f);
    else phase_ = next;
  }

  float act = fminf(activity_, 1.0f);
  stance_ = 0;
  for (int i = 0; i < 6; i++) {
    float nx = neutral_[i][0], nz = neutral_[i][1];
    float hx = (vel_[0] * scale + vel_[2] * scale * nz) * stanceTime * 0.5f;
    float hzz = (vel_[1] * scale - vel_[2] * scale * nx) * stanceTime * 0.5f;
    float local = phase_ + OFFS[i];
    local -= floorf(local);
    float s, lift;
    bool swing = local < SWING;
    if (swing) {
      float u = local / SWING;
      float ease = u * u * (3.0f - 2.0f * u);
      s = ease * 2.0f - 1.0f;
      lift = sinf(PI_F * u);
    } else {
      float u = (local - SWING) / (1.0f - SWING);
      s = 1.0f - 2.0f * u;
      lift = 0.0f;
    }
    bool down = !swing || lift * STEP_HEIGHT * act < 0.001f;
    if (down) stance_ |= (1 << i);
    float fx = nx + hx * s;
    float fy = -height_ + lift * STEP_HEIGHT * act;
    float fz = nz + hzz * s;
    feet_[i * 3] = fx;
    feet_[i * 3 + 1] = fy;
    feet_[i * 3 + 2] = fz;
    // body frame -> leg frame
    float dx = fx - MOUNT_X[i], dz = fz - MOUNT_Z[i];
    float cy = cosf(yaw_[i]), sy = sinf(yaw_[i]);
    float lx = dx * cy + dz * sy;
    float lz = -dx * sy + dz * cy;
    reach_[i] = sqrtf(lx * lx + lz * lz);
    LegAngles a = solveLeg(lx, fy, lz, LEG_COXA, LEG_FEMUR, LEG_TIBIA, LIM_TIBIA_MIN * DEG);
    angles[i * 3] = clampf(a.coxa, -LIM_COXA * DEG, LIM_COXA * DEG);
    angles[i * 3 + 1] = clampf(a.femur, LIM_FEMUR_MIN * DEG, LIM_FEMUR_MAX * DEG);
    angles[i * 3 + 2] = clampf(a.tibia, LIM_TIBIA_MIN * DEG, LIM_TIBIA_MAX * DEG);
  }
}

void anglesToServo(const float angles[18], float deg[18]) {
  for (int i = 0; i < 6; i++) {
    float c = angles[i * 3] / DEG, f = angles[i * 3 + 1] / DEG, t = angles[i * 3 + 2] / DEG;
    deg[i * 3] = clampf(90.0f + DIR[0] * c + TRIM[i][0], SERVO_DEG_MIN, SERVO_DEG_MAX);
    deg[i * 3 + 1] = clampf(90.0f + DIR[1] * f + TRIM[i][1], SERVO_DEG_MIN, SERVO_DEG_MAX);
    deg[i * 3 + 2] = clampf(90.0f + DIR[2] * (t + 90.0f) + TRIM[i][2], SERVO_DEG_MIN, SERVO_DEG_MAX);
  }
}
