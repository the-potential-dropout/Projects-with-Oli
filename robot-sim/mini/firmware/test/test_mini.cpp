// Host tests for the Hexapod Mini kinematics and gait (no hardware needed).
//   g++ -std=c++17 -O2 -I../mini_hexapod test_mini.cpp ../mini_hexapod/kinematics.cpp -o test_mini && ./test_mini
#include <math.h>
#include <stdio.h>
#include <set>
#include "config.h"
#include "kinematics.h"

static int fails = 0;
#define CHECK(c, ...)                         \
  do {                                        \
    if (!(c)) {                               \
      printf("FAIL %s:%d: ", __FILE__, __LINE__); \
      printf(__VA_ARGS__);                    \
      printf("\n");                           \
      fails++;                                \
    }                                         \
  } while (0)

static const float D2R = 3.14159265f / 180.0f;
// adjacent legs are only clear of each other up to this coxa angle (cad sweep, "pairs")
static const float COXA_GAIT_MAX = 25.0f;

int main() {
  // 1. IK round trip through FK over the reachable stance/swing box
  int n = 0;
  for (float r = 0.050f; r <= 0.072f; r += 0.004f)
    for (float y = -0.055f; y <= -0.025f; y += 0.005f)
      for (float a = -0.6f; a <= 0.6f; a += 0.3f) {
        float x = r * cosf(a), z = r * sinf(a);
        LegAngles s = solveLeg(x, y, z, LEG_COXA, LEG_FEMUR, LEG_TIBIA, -3.14159f);
        if (s.clamped) continue;
        float p[3];
        forwardLeg(s, LEG_COXA, LEG_FEMUR, LEG_TIBIA, p);
        CHECK(fabsf(p[0] - x) < 1e-4f && fabsf(p[1] - y) < 1e-4f && fabsf(p[2] - z) < 1e-4f, "FK(IK) r %.3f y %.3f", r, y);
        n++;
      }
  CHECK(n > 100, "only %d reachable test points", n);

  // 2. tibia limit: a tucked foot never folds the tibia past LIM_TIBIA_MIN
  LegAngles t = solveLeg(0.040f, -0.020f, 0.0f, LEG_COXA, LEG_FEMUR, LEG_TIBIA, LIM_TIBIA_MIN * D2R);
  CHECK(t.clamped && t.tibia >= LIM_TIBIA_MIN * D2R - 1e-3f, "tibia %.1f", t.tibia / D2R);

  // 3. standing pose is inside every joint limit and not clamped
  {
    TripodGait g;
    float ang[18];
    g.update(0.02f, 0, 0, 0, ang);
    for (int i = 0; i < 6; i++) {
      CHECK(fabsf(ang[i * 3]) < 1e-3f, "leg %d coxa %.2f deg at rest", i, ang[i * 3] / D2R);
      CHECK(ang[i * 3 + 1] > LIM_FEMUR_MIN * D2R && ang[i * 3 + 1] < LIM_FEMUR_MAX * D2R, "leg %d femur", i);
      CHECK(ang[i * 3 + 2] > LIM_TIBIA_MIN * D2R && ang[i * 3 + 2] < LIM_TIBIA_MAX * D2R, "leg %d tibia %.1f", i, ang[i * 3 + 2] / D2R);
      CHECK(fabsf(g.reach(i) - FOOT_REACH) < 1e-4f, "leg %d reach", i);
    }
    printf("stand: femur %.1f deg, tibia %.1f deg\n", ang[1] / D2R, ang[2] / D2R);
  }

  // 4. walking: tripod never has both groups up, stance feet stay in their window,
  //    every joint stays inside its limits, for forward, strafe, turn and mixes, walk + fast
  const float cmds[][3] = {{0, V_MAX, 0}, {V_MAX, 0, 0}, {0, 0, W_MAX}, {V_MAX * 0.7f, V_MAX * 0.7f, W_MAX}, {0, -V_MAX, -W_MAX}};
  for (int fast = 0; fast < 2; fast++)
    for (auto &c : cmds) {
      TripodGait g;
      g.setFast(fast);
      float ang[18];
      float rmin = 1, rmax = 0, minScale = 1;
      float travel = 0, cmax = 0;
      for (int k = 0; k < 600; k++) {
        g.update(0.02f, c[0], c[1], c[2], ang);
        uint8_t m = g.stanceMask();
        CHECK((m & 0b010101) == 0b010101 || (m & 0b101010) == 0b101010, "both tripods up, mask %02x", m);
        for (int i = 0; i < 6; i++) {
          if (k > 50 && (m & (1 << i))) {
            float lim = (i == 1 || i == 4) ? REACH_MAX_MIDDLE : REACH_MAX_CORNER;
            CHECK(g.reach(i) <= lim + 1e-4f && g.reach(i) >= REACH_MIN - 1e-4f, "leg %d stance reach %.4f", i, g.reach(i));
            rmin = fminf(rmin, g.reach(i));
            rmax = fmaxf(rmax, g.reach(i));
          }
          CHECK(fabsf(ang[i * 3]) <= COXA_GAIT_MAX * D2R + 1e-4f, "coxa %.1f deg beyond the gait limit", ang[i * 3] / D2R);
          cmax = fmaxf(cmax, fabsf(ang[i * 3]));
        }
        minScale = fminf(minScale, g.speedScale());
        travel += 0.02f * g.speedScale() * sqrtf(c[0] * c[0] + c[1] * c[1]);
      }
      printf("cmd (%.2f, %.2f, %.2f)%s: stance reach %.1f-%.1f mm, speed scale %.2f, %.2f m in 12 s, max |coxa| %.1f deg\n", c[0], c[1], c[2],
             fast ? " fast" : "", rmin * 1000, rmax * 1000, minScale, travel, cmax / D2R);
      // release the sticks: the gait must come to rest with all six feet down
      for (int k = 0; k < 200; k++) g.update(0.02f, 0, 0, 0, ang);
      CHECK(g.atRest() && g.stanceMask() == 0x3f, "did not come to rest (mask %02x)", g.stanceMask());
    }

  // 5. servo mapping: rest pose inside the servo range; channel map has no duplicates per board
  {
    TripodGait g;
    float ang[18], deg[18];
    g.update(0.02f, 0, 0, 0, ang);
    anglesToServo(ang, deg);
    for (int k = 0; k < 18; k++) CHECK(deg[k] > SERVO_DEG_MIN && deg[k] < SERVO_DEG_MAX, "servo %d %.1f", k, deg[k]);
    for (int b = 0; b < 2; b++) {
      std::set<int> used;
      for (int i = b * 3; i < b * 3 + 3; i++)
        for (int j = 0; j < 3; j++) CHECK(used.insert(CHANNEL[i][j]).second, "board %d channel %d used twice", b, CHANNEL[i][j]);
    }
  }

  printf(fails ? "%d FAILED\n" : "all tests passed\n", fails);
  return fails ? 1 : 0;
}
