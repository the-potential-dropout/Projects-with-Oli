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
  float slope = 12 * DEG, meas = slope;
  for (int k = 0; k < 400; k++) {
    b.update(0.01f, 0, 0, 0, meas, 0, 0);
    meas = slope + b.outPitch;
  }
  printf("balance on a 12 deg slope: body pitch %.2f deg after 4 s (correction %.1f deg)\n", meas / DEG, b.outPitch / DEG);
  CHECK(fabsf(meas) < 0.3f * DEG, "not levelled");
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
  test_crsf();
  printf(fails ? "%d FAILED\n" : "all tests passed\n", fails);
  return fails ? 1 : 0;
}
