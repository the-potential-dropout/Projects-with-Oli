// Host tests for the onboard controller core (no hardware needed).
//   g++ -std=c++17 -O2 -I../hexapod_controller test_core.cpp ../hexapod_controller/kinematics.cpp -o test_core && ./test_core
#include <stdio.h>
#include <stdlib.h>

#include <initializer_list>

#include "crsf.h"
#include "kinematics.h"

static int fails = 0;
#define CHECK(cond, ...)                         \
  do {                                           \
    if (!(cond)) {                               \
      fails++;                                   \
      printf("FAIL %s:%d: ", __FILE__, __LINE__); \
      printf(__VA_ARGS__);                       \
      printf("\n");                              \
    }                                            \
  } while (0)

using namespace hk;

static void test_ik_roundtrip() {
  for (float x = 0.08f; x < 0.20f; x += 0.01f)
    for (float y = -0.12f; y < -0.03f; y += 0.01f)
      for (float z = -0.06f; z <= 0.06f; z += 0.02f) {
        LegAngles a = solveLeg(x, y, z);
        if (a.clamped) continue;
        Vec3 p = forwardLeg(a);
        CHECK(fabsf(p.x - x) + fabsf(p.y - y) + fabsf(p.z - z) < 1e-4f, "IK/FK mismatch at %.3f %.3f %.3f", x, y, z);
      }
  // neutral stance: foot 0.1 out, 0.09 down -> femur slightly up, tibia about -90
  LegAngles a = solveLeg(0.10f, -0.09f, 0.0f);
  printf("neutral leg: coxa %.1f femur %.1f tibia %.1f deg\n", a.coxa / DEG, a.femur / DEG, a.tibia / DEG);
  CHECK(!a.clamped, "neutral clamped");
}

static void test_standing() {
  Gait g;
  g.init();
  GaitInput in;
  for (int k = 0; k < 200; k++) g.update(0.01f, in);
  for (int i = 0; i < 6; i++) CHECK(fabsf(g.feet[i].y + cfg::STAND_HEIGHT) < 1e-4f, "foot %d not on the ground", i);
  CHECK(g.clampedMask == 0, "clamped while standing: %x", g.clampedMask);
}

static void test_tripod_walk_and_window() {
  Gait g;
  g.init();
  GaitInput in;
  in.vz = cfg::SPEED_SPRINT;
  in.yawRate = 0.8f;
  in.stepHz = cfg::STEP_HZ_SPRINT;
  float worstR = 0, bestR = 1;
  for (int k = 0; k < 600; k++) {
    g.update(0.01f, in);
    bool aUp = false, bUp = false;
    for (int i = 0; i < 6; i++) {
      bool up = g.feet[i].y > -cfg::STAND_HEIGHT + 5e-4f;
      if (i % 2 == 0) aUp |= up; else bUp |= up;
      if (g.stance[i] && k > 100) {
        float dx = g.feet[i].x - cfg::MOUNT[i][0], dz = g.feet[i].z - cfg::MOUNT[i][1];
        float r = sqrtf(dx * dx + dz * dz);
        worstR = fmaxf(worstR, r);
        bestR = fminf(bestR, r);
      }
    }
    CHECK(!(aUp && bUp), "both tripods up at step %d", k);
  }
  printf("sprint+turn stance reach: %.1f .. %.1f mm (window %.0f .. %.0f)\n", bestR * 1e3, worstR * 1e3,
         cfg::FOOT_REACH_MIN * 1e3, cfg::FOOT_REACH_MAX * 1e3);
  CHECK(worstR <= cfg::FOOT_REACH_MAX + 1e-3f && bestR >= cfg::FOOT_REACH_MIN - 1e-3f, "stance window violated");
  CHECK(g.clampedMask == 0, "IK clamped while walking: %x", g.clampedMask);
}

static void test_ripple_wave_support() {
  for (GaitType t : {GaitType::Ripple, GaitType::Wave}) {
    Gait g;
    g.init();
    GaitInput in;
    in.type = t;
    for (int k = 0; k < 50; k++) g.update(0.01f, in);   // switch while parked
    in.vz = 0.1f;
    int minDown = 6;
    for (int k = 0; k < 600; k++) {
      g.update(0.01f, in);
      int down = 0;
      for (int i = 0; i < 6; i++) down += g.stance[i];
      if (k > 50) minDown = down < minDown ? down : minDown;
    }
    printf("%s: min feet down %d\n", t == GaitType::Ripple ? "ripple" : "wave", minDown);
    CHECK(minDown >= (t == GaitType::Ripple ? 4 : 5), "too few feet down");
  }
}

static void test_margin() {
  Gait g;
  g.init();
  float all[6][2], tri[3][2];
  for (int i = 0; i < 6; i++) { all[i][0] = g.neutral[i].x; all[i][1] = g.neutral[i].z; }
  int k = 0;
  for (int i = 0; i < 6; i += 2) { tri[k][0] = g.neutral[i].x; tri[k][1] = g.neutral[i].z; k++; }
  float m6 = stabilityMargin(all, 6, 0, 0), m3 = stabilityMargin(tri, 3, 0, 0);
  printf("static margin: 6 feet %.1f mm, tripod %.1f mm\n", m6 * 1e3, m3 * 1e3);
  CHECK(m3 > cfg::MARGIN_MIN, "tripod margin below the floor");
  CHECK(stabilityMargin(tri, 3, 0.5f, 0) < 0, "CoM outside not detected");
}

static void test_balance_on_slope() {
  // body on a 12 deg slope (pitch): measured pitch = slope + commanded correction
  Balance b;
  float slope = 8 * DEG, meas = slope;
  for (int k = 0; k < 400; k++) {
    b.update(0.01f, 0, 0, 0, meas, 0, 0);
    meas = slope + b.outPitch;
  }
  printf("balance (no gyro, no lag) on an 8 deg slope: body pitch %.2f deg after 4 s (correction %.1f deg)\n", meas / DEG, b.outPitch / DEG);
  CHECK(fabsf(meas) < 0.3f * DEG, "not levelled");
  // beyond the reach limit the correction saturates at BAL_OUT_MAX (12 deg slope -> 2 deg left)
  Balance c;
  float m12 = 12 * DEG;
  for (int k = 0; k < 400; k++) { c.update(0.01f, 0, 0, 0, m12, 0, 0); m12 = 12 * DEG + c.outPitch; }
  CHECK(fabsf(c.outPitch + cfg::BAL_OUT_MAX * DEG) < 0.01f * DEG, "12 deg slope not clamped at the levelling limit");
}

// Terrain touchdown with the real body pitched relative to the command.  Feet resting on
// flat ground under a body pitched 6 deg: the touchdown height must be the same for
// every leg (the true level frame), whereas the old command-frame height differs front
// to back by about L*sin(6 deg) and teaches the gait the tilt (positive feedback).
static void test_touchdown_attitude() {
  Gait g;
  g.init();
  GaitInput in;
  in.terrain = true;
  in.pose.height = cfg::TERRAIN_HEIGHT;
  in.hasAttitude = true;
  in.attPitch = 6 * DEG;
  Pose actual;
  actual.pitch = in.attPitch;
  // feet on the ground plane y = -height, expressed in the (pitched) body frame
  Pose inv;
  float lo = 1, hi = -1, lo0 = 1, hi0 = -1;
  for (int i = 0; i < 6; i++) {
    Vec3 w{g.neutral[i].x, -cfg::TERRAIN_HEIGHT, g.neutral[i].z};
    // body = R^T w: rotate by the inverse pitch
    actual.pitch = -in.attPitch;
    g.feetBody[i] = rotate(actual, w);
    actual.pitch = in.attPitch;
    float t = g.touchdownHeight(i, in);
    lo = fminf(lo, t); hi = fmaxf(hi, t);
    lo0 = fminf(lo0, g.feetBody[i].y); hi0 = fmaxf(hi0, g.feetBody[i].y);
  }
  printf("touchdown on flat ground, body pitched 6 deg: height spread %.2f mm in the IMU level frame, %.1f mm in the command frame (old)\n",
         (hi - lo) * 1e3, (hi0 - lo0) * 1e3);
  CHECK(hi - lo < 0.0005f, "touchdown heights still depend on the body's tilt");
  CHECK(hi0 - lo0 > 0.01f, "test doesn't distinguish the two");
}

// Balance with the real gyro rate and a 60 ms first-order servo lag (the sim's plant).
static void test_balance_gyro_lag() {
  Balance b;
  float slope = 8 * DEG, act = 0, prev = slope, tau = 0.06f, dt = 0.01f, worst = 0;
  for (int k = 0; k < 400; k++) {
    float meas = slope + act;
    float rate = (meas - prev) / dt;
    prev = meas;
    b.update(dt, 0, 0, 0, meas, 0, rate);
    act += (b.outPitch - act) * (dt / tau);
    if (k >= 300) worst = fmaxf(worst, fabsf(slope + act));
  }
  printf("balance, 8 deg slope via gyro + 60 ms servo lag: residual %.2f deg (max over the last 1 s)\n", worst / DEG);
  CHECK(worst < 0.3f * DEG, "not levelled or chattering");
}

static void test_crsf() {
  uint16_t ch[16];
  for (int i = 0; i < 16; i++) ch[i] = 172 + i * 100;
  uint8_t buf[64];
  int n = crsf::rcChannels(ch, buf);
  crsf::Parser p;
  bool ok = false;
  for (int i = 0; i < n; i++) ok |= p.feed(buf[i]);
  CHECK(ok && p.newChannels, "rc frame not parsed");
  for (int i = 0; i < 16; i++) CHECK(p.ch[i] == ch[i], "ch%d %d != %d", i, p.ch[i], ch[i]);
  buf[5] ^= 0x40;   // corrupt
  crsf::Parser q;
  for (int i = 0; i < n; i++) q.feed(buf[i]);
  CHECK(q.crcErrors == 1 && !q.newChannels, "bad crc accepted");
  n = crsf::battery(11.7f, 4.2f, 830, 71, buf);
  CHECK(n == 12 && buf[2] == crsf::T_BATTERY && crsf::crc8(&buf[2], 9) == buf[11], "battery frame");
}

int main() {
  test_ik_roundtrip();
  test_standing();
  test_tripod_walk_and_window();
  test_ripple_wave_support();
  test_margin();
  test_balance_on_slope();
  test_balance_gyro_lag();
  test_touchdown_attitude();
  test_crsf();
  printf(fails ? "%d FAILED\n" : "all tests passed\n", fails);
  return fails ? 1 : 0;
}
