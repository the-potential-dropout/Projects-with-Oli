// Gait engine, pose and balance for Hexapod Mk1 (see kinematics.h, CONTROL.md).
#include "kinematics.h"

namespace hk {

// Inverse of rotate(): body pose R^T applied to a level-frame vector.
static Vec3 rotateInv(const Pose& p, Vec3 v) {
  float cr = cosf(p.roll), sr = sinf(p.roll), cp = cosf(p.pitch), sp = sinf(p.pitch), cy = cosf(p.yaw), sy = sinf(p.yaw);
  Vec3 b{cy * v.x - sy * v.z, v.y, sy * v.x + cy * v.z};            // undo yaw
  Vec3 a{b.x, cp * b.y - sp * b.z, sp * b.y + cp * b.z};            // undo pitch
  return {cr * a.x + sr * a.y, -sr * a.x + cr * a.y, a.z};          // undo roll
}

float stabilityMargin(const float feet[][2], int n, float comX, float comZ) {
  if (n < 3) return -1.0f;
  // order the stance feet by angle round their centroid (convex for a hexapod stance)
  float cx = 0, cz = 0;
  for (int i = 0; i < n; i++) { cx += feet[i][0]; cz += feet[i][1]; }
  cx /= n; cz /= n;
  int idx[6];
  float ang[6];
  for (int i = 0; i < n; i++) { idx[i] = i; ang[i] = atan2f(feet[i][1] - cz, feet[i][0] - cx); }
  for (int i = 1; i < n; i++)
    for (int j = i; j > 0 && ang[idx[j]] < ang[idx[j - 1]]; j--) { int t = idx[j]; idx[j] = idx[j - 1]; idx[j - 1] = t; }
  float m = 1e9f;
  for (int k = 0; k < n; k++) {
    const float* a = feet[idx[k]];
    const float* b = feet[idx[(k + 1) % n]];
    float ex = b[0] - a[0], ez = b[1] - a[1];
    float L = sqrtf(ex * ex + ez * ez);
    if (L < 1e-6f) continue;
    // counter-clockwise order -> inside is on the left of each edge
    float d = (ex * (comZ - a[1]) - ez * (comX - a[0])) / L;
    m = fminf(m, d);
  }
  return m;
}

// femur-to-foot distance with the tibia at its fold limit (-135 deg): 85.4 mm
static const float kMinReach = sqrtf(cfg::FEMUR * cfg::FEMUR + cfg::TIBIA * cfg::TIBIA -
                                     2 * cfg::FEMUR * cfg::TIBIA * cosf((180.0f + cfg::TIBIA_LIM[0]) * DEG)) + 2e-4f;

static const float OFFS_TRIPOD[6] = {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f};
// ripple: each side back-to-front, sides half a cycle apart; wave: one leg at a time
static const float OFFS_RIPPLE[6] = {2.f / 3, 1.f / 3, 0.0f, 1.f / 6, 5.f / 6, 0.5f};
static const float OFFS_WAVE[6] = {2.f / 6, 1.f / 6, 0.0f, 5.f / 6, 4.f / 6, 3.f / 6};

void Gait::init() {
  using namespace cfg;
  for (int i = 0; i < 6; i++) {
    float x = MOUNT[i][0], z = MOUNT[i][1];
    float splay = SPLAY_DEG[i] * DEG;
    mountYaw[i] = (x < 0) ? PI_F - splay : splay;           // same as gaitMounts() in hexapod.config.js
    neutral[i] = {x + FOOT_REACH * cosf(mountYaw[i]), -STAND_HEIGHT, z + FOOT_REACH * sinf(mountYaw[i])};
    feet[i] = neutral[i];
    groundY[i] = 0;
    stance[i] = true;
  }
  phase = 0; activity = 0;
  vel[0] = vel[1] = vel[2] = 0;
}

float Gait::legPhase(int leg) const {
  const float* o = current == GaitType::Tripod ? OFFS_TRIPOD : (current == GaitType::Ripple ? OFFS_RIPPLE : OFFS_WAVE);
  float p = phase + o[leg];
  return p - floorf(p);
}

void Gait::update(float dt, const GaitInput& in) {
  using namespace cfg;
  // first-order command filter (as the sim)
  float k = 1.0f - expf(-dt * CMD_SMOOTH);
  vel[0] += (in.vx - vel[0]) * k;
  vel[1] += (in.vz - vel[1]) * k;
  vel[2] += (in.yawRate - vel[2]) * k;
  bool moving = fabsf(vel[0]) + fabsf(vel[1]) + fabsf(vel[2]) * 0.15f > 0.004f;
  activity += ((moving ? 1.f : 0.f) - activity) * (1.0f - expf(-dt * 5.0f));

  // change gait only when every foot is down (phase boundary)
  float swingFrac = (current == GaitType::Tripod) ? 0.5f : (current == GaitType::Ripple ? 1.f / 3 : 1.f / 6);
  float stepHz = in.stepHz;
  float stanceTime = (1.0f - swingFrac) / stepHz;

  // stance-window rule: keep every stance foot between FOOT_REACH_MIN and _MAX from its
  // coxa axis (the load check). Step faster first, then slow the body down.
  float scale = 1.0f;
  for (int pass = 0; pass < 2; pass++) {
    scale = 1.0f;
    for (int i = 0; i < 6; i++) {
      float nx = neutral[i].x, nz = neutral[i].z;
      float hx = (vel[0] + vel[2] * nz) * stanceTime * 0.5f, hz = (vel[1] - vel[2] * nx) * stanceTime * 0.5f;
      for (int sg = -1; sg <= 1; sg += 2) {
        float fx = nx + sg * hx - MOUNT[i][0], fz = nz + sg * hz - MOUNT[i][1];
        float r = sqrtf(fx * fx + fz * fz);
        float r0 = FOOT_REACH;
        float dr = r - r0;
        float lim = dr > 0 ? FOOT_REACH_MAX - r0 : r0 - FOOT_REACH_MIN;
        if (fabsf(dr) > lim) scale = fminf(scale, lim / fabsf(dr));
      }
    }
    if (scale < 1.0f && pass == 0 && stepHz < STEP_HZ_SPRINT) {
      stepHz = fminf(STEP_HZ_SPRINT, stepHz / fmaxf(scale, 0.3f));
      stanceTime = (1.0f - swingFrac) / stepHz;
    } else {
      break;
    }
  }
  if (scale < 1.0f) {
    for (float& v : vel) v *= scale;
  }

  float next = phase + dt * stepHz;
  if (activity > 0.02f) {
    phase = next - floorf(next);
  } else {
    // finish the current swing, then park with all feet down
    float n = 1.0f / swingFrac;                       // phase boundaries where all feet are down (tripod: 0, 0.5)
    bool atRest = fmodf(phase * n, 1.0f) < 1e-5f;
    if (!atRest) {
      if (floorf(next * n) > floorf(phase * n)) phase = fmodf(floorf(next * n) / n, 1.0f);
      else phase = next;
    } else if (in.type != current) {
      current = in.type;                              // safe moment to switch gait
    }
  }
  if (activity > 0.02f && in.type != current && fmodf(phase / swingFrac, 1.0f) < dt * stepHz / swingFrac) current = in.type;

  clampedMask = 0;
  float meanGround = 0;
  int nStance = 0;
  for (int i = 0; i < 6; i++) {
    float nx = neutral[i].x, nz = neutral[i].z;
    float hx = (vel[0] + vel[2] * nz) * stanceTime * 0.5f, hz = (vel[1] - vel[2] * nx) * stanceTime * 0.5f;
    float local = legPhase(i);
    float s, lift;
    bool inSwing = local < swingFrac;
    if (inSwing) {
      float u = local / swingFrac;
      float ease = u * u * (3.0f - 2.0f * u);
      s = ease * 2.0f - 1.0f;
      lift = sinf(PI_F * u);
      // terrain: in the last 20 % of the swing keep lowering until the switch closes
      if (in.terrain && u > 0.8f && in.contact[i]) {
        groundY[i] = feet[i].y + in.pose.height;       // early touchdown -> remember this foot's ground height
        lift = 0;
      }
    } else {
      float u = (local - swingFrac) / (1.0f - swingFrac);
      s = 1.0f - 2.0f * u;
      lift = 0;
      if (in.terrain && !in.contact[i] && u < 0.5f) {
        // late touchdown: search down (bounded), then accept
        groundY[i] = fmaxf(groundY[i] - TD_SEARCH_SPEED * dt, -TD_SEARCH);
      }
    }
    stance[i] = !inSwing;
    if (!in.terrain) groundY[i] *= (1.0f - fminf(1.0f, dt * 2.0f));   // relax back to flat
    groundY[i] = fminf(fmaxf(groundY[i], -TD_SEARCH), STEP_UP_MAX);
    if (stance[i]) { meanGround += groundY[i]; nStance++; }

    float act = fminf(activity, 1.0f);
    feet[i] = {nx + hx * s, -in.pose.height + groundY[i] + lift * in.stepHeight * act, nz + hz * s};

    // level frame -> posed body frame -> leg frame
    Vec3 f = feet[i];
    Vec3 pb = rotateInv(in.pose, {f.x - in.pose.shiftX, f.y, f.z - in.pose.shiftZ});
    float dx = pb.x - MOUNT[i][0], dz = pb.z - MOUNT[i][1];
    float c = cosf(mountYaw[i]), sn = sinf(mountYaw[i]);
    float lx = dx * c + dz * sn, lz = -dx * sn + dz * c;
    // a foot lifted close to the body would need the tibia folded past its limit:
    // cap the lift there (the swing just stays a little lower)
    float horiz = sqrtf(lx * lx + lz * lz) - COXA;
    if (horiz < kMinReach) pb.y = fminf(pb.y, -sqrtf(kMinReach * kMinReach - horiz * horiz));
    LegAngles a = solveLeg(lx, pb.y, lz, TIBIA_LIM[0] * DEG);
    clampJoints(a);
    if (a.clamped) clampedMask |= (1u << i);
    angles[i] = a;
  }
  // terrain-following body height: drift the per-leg offsets back round zero so the
  // body follows the mean ground height of the stance feet (done by the caller via pose.height)
  if (in.terrain && nStance > 0) {
    meanGround /= nStance;
    float d = meanGround * fminf(1.0f, dt * 1.5f);
    for (int i = 0; i < 6; i++) groundY[i] -= d;
    terrainShift += d;
  }
}

void Balance::update(float dt, float tRoll, float tPitch, float mRoll, float mPitch, float rRoll, float rPitch) {
  using namespace cfg;
  if (!enabled || dt <= 0) { reset(); return; }
  float eR = tRoll - mRoll, eP = tPitch - mPitch;
  float iMax = BAL_I_MAX * DEG;
  iRoll = fminf(fmaxf(iRoll + eR * dt, -iMax / fmaxf(BAL_KI, 1e-3f)), iMax / fmaxf(BAL_KI, 1e-3f));
  iPitch = fminf(fmaxf(iPitch + eP * dt, -iMax / fmaxf(BAL_KI, 1e-3f)), iMax / fmaxf(BAL_KI, 1e-3f));
  float a = 1.0f - expf(-2.0f * PI_F * BAL_D_LPF_HZ * dt);
  dRoll += (-rRoll - dRoll) * a;                     // derivative on measurement (no kick on setpoint changes)
  dPitch += (-rPitch - dPitch) * a;
  float uR = BAL_KP * eR + BAL_KI * iRoll + BAL_KD * dRoll;
  float uP = BAL_KP * eP + BAL_KI * iPitch + BAL_KD * dPitch;
  float rate = BAL_RATE_MAX * DEG * dt;
  outRoll += fminf(fmaxf(uR - outRoll, -rate), rate);
  outPitch += fminf(fmaxf(uP - outPitch, -rate), rate);
  outRoll = fminf(fmaxf(outRoll, -POSE_ROLL_MAX * DEG), POSE_ROLL_MAX * DEG);
  outPitch = fminf(fmaxf(outPitch, -POSE_PITCH_MAX * DEG), POSE_PITCH_MAX * DEG);
}

}  // namespace hk
