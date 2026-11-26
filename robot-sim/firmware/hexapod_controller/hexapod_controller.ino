// Hexapod Mk1 rev B2 - onboard controller (Adafruit Feather ESP32-S3).
//
// Runs the robot on its own: IK, gait, body pose, IMU balance, terrain adaptation
// and safety at 100 Hz.  Commands come from three sources (CONTROL.md section 6):
//   RC     ExpressLRS receiver, CRSF on UART2               (remote / override / e-stop)
//   HOST   USB-C: the simulator or a PC                     (joint mode "S," or command mode "C,")
//   BRAIN  Raspberry Pi 5 brain module, UART on J1 AUX1/2   (autonomy, same "C," lines)
// Servos: Feetech STS3215/STS3250 bus, reached through the Waveshare servo board
// running firmware/bus_bridge (transparent 1 Mbaud UART bridge + battery ADC).
//
// Libraries (Arduino library manager / Waveshare):
//   SCServo (Waveshare ST3215 wiki, ST_Servo.zip)   - SMS_STS
//   Adafruit BNO08x                                 - BNO085 rotation vector + gyro
//   VL53L1X by Pololu                               - time-of-flight look-ahead
// Board: "Adafruit Feather ESP32-S3 No PSRAM" (#5323) or "... 2MB PSRAM" (#5477), USB CDC on boot: Enabled.
//
// STATUS: complete control structure; the controller maths (kinematics.cpp, crsf.h)
// is unit-tested on the PC (firmware/test).  Not yet run on hardware: tune on the
// stand first (CONTROL.md section 9).

#include <Adafruit_BNO08x.h>
#include <SCServo.h>
#include <VL53L1X.h>
#include <Wire.h>

#include "config.h"
#include "crsf.h"
#include "kinematics.h"

using namespace hk;

// ---------------------------------------------------------------- hardware objects
HardwareSerial& BusSerial = Serial1;   // -> bus bridge
HardwareSerial& RcSerial = Serial2;    // -> ELRS receiver
SMS_STS st;
Adafruit_BNO08x imu(pins::IMU_RST);
VL53L1X tof;
crsf::Parser rc;

// ---------------------------------------------------------------- state
enum class Mode : uint8_t { Relaxed, Stand, Walk, JointBridge };
enum class Source : uint8_t { None, RC, Host, Brain };
struct Command {
  float vx = 0, vz = 0, yaw = 0;          // m/s, rad/s
  float roll = 0, pitch = 0, bodyYaw = 0; // rad (pose request)
  float height = cfg::STAND_HEIGHT;
  GaitType gait = GaitType::Tripod;
  bool terrain = false, fire = false, sprint = false;
  uint32_t stamp = 0;
};

Gait gait;
Balance bal;
Mode mode = Mode::Relaxed;
Source source = Source::None;
Command cmdRc, cmdHost, cmdBrain;
bool armed = false, rcArmSwitch = false, rcAuto = false, rcKill = false, brainOnJ1 = false;
uint32_t lastHostMs = 0, lastBrainMs = 0, lastRcMs = 0;
float jointDeg[23];                      // joint-bridge mode target (servo degrees)
float residual[18];                      // learned-policy residuals from the brain (rad), "R," lines
uint32_t lastResidualMs = 0;
int nJoint = 0;
bool hostJointFresh = false;

// sensors
float imuRoll = 0, imuPitch = 0, imuYaw = 0, gyroRoll = 0, gyroPitch = 0;
bool imuOk = false, tofOk = false;
uint16_t tofMm = 0;
bool foot[6];
float vbat = 12.0f, ibat = 0.0f, mahUsed = 0;
uint8_t hottest = 0;
bool lvc = false;
uint32_t lowSince = 0;
float margin = 0.1f;
const char* fault = "OK";

// ---------------------------------------------------------------- servo bus
static uint8_t ids[18];
static int16_t pos[18];
static uint16_t spd[18];
static uint8_t acc[18];

void servoTorque(bool on) {
  for (int l = 0; l < 6; l++)
    for (int j = 0; j < 3; j++) st.EnableTorque(cfg::BUS_ID[l][j], on ? 1 : 0);
}

void servoGains(bool soft) {
  // ST/STS EEPROM-backed P/D at addresses 21/22: unlock (55) -> write -> lock
  for (int l = 0; l < 6; l++)
    for (int j = 0; j < 3; j++) {
      uint8_t id = cfg::BUS_ID[l][j];
      st.unLockEprom(id);
      st.writeByte(id, 21, soft ? cfg::KP_SOFT : cfg::KP_STIFF);
      st.writeByte(id, 22, soft ? cfg::KD_SOFT : cfg::KD_STIFF);
      st.LockEprom(id);
    }
}

void writeLegs(const LegAngles a[6], uint16_t speed) {
  for (int l = 0; l < 6; l++) {
    float d[3];
    toServoDeg(l, a[l], d);
    for (int j = 0; j < 3; j++) {
      int k = l * 3 + j;
      ids[k] = cfg::BUS_ID[l][j];
      pos[k] = degToStep(d[j]);
      spd[k] = speed;
      acc[k] = cfg::ACC_RUN;
    }
  }
  st.SyncWritePosEx(ids, 18, pos, spd, acc);
}

void writeJoints(const float* deg, int n, uint16_t speed) {
  for (int k = 0; k < 18 && k < n; k++) {
    ids[k] = cfg::BUS_ID[k / 3][k % 3];
    pos[k] = degToStep(deg[k]);
    spd[k] = speed;
    acc[k] = cfg::ACC_RUN;
  }
  st.SyncWritePosEx(ids, n < 18 ? n : 18, pos, spd, acc);
  for (int k = 18; k < n && k < 23; k++) st.WritePosEx(cfg::AUX_ID[k - 18], degToStep(deg[k]), speed, cfg::ACC_RUN);
}

// battery voltage / current from the bridge (local packet A5 5A 01 -> A5 5A 01 mv_hi mv_lo ma_hi ma_lo sum)
void pollBridgeAdc() {
  while (BusSerial.available()) BusSerial.read();
  const uint8_t q[3] = {0xA5, 0x5A, 0x01};
  BusSerial.write(q, 3);
  uint32_t t0 = micros();
  uint8_t r[8];
  int n = 0;
  while (n < 8 && micros() - t0 < 3000)
    if (BusSerial.available()) r[n++] = BusSerial.read();
  if (n == 8 && r[0] == 0xA5 && r[1] == 0x5A && r[2] == 0x01 && (uint8_t)(r[3] + r[4] + r[5] + r[6]) == r[7]) {
    vbat = ((r[3] << 8) | r[4]) / 1000.0f;
    ibat = (int16_t)((r[5] << 8) | r[6]) / 1000.0f;
  }
}

// ---------------------------------------------------------------- sensors
void readImu() {
  sh2_SensorValue_t v;
  while (imu.getSensorEvent(&v)) {
    if (v.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qr = v.un.gameRotationVector.real, qi = v.un.gameRotationVector.i, qj = v.un.gameRotationVector.j,
            qk = v.un.gameRotationVector.k;
      // BNO085 mounted with x forward, y left, z up (ImuPod) -> sim roll about forward, pitch about left
      imuRoll = atan2f(2 * (qr * qi + qj * qk), 1 - 2 * (qi * qi + qj * qj));
      imuPitch = asinf(fminf(fmaxf(2 * (qr * qj - qk * qi), -1.f), 1.f));
      imuYaw = atan2f(2 * (qr * qk + qi * qj), 1 - 2 * (qj * qj + qk * qk));
    } else if (v.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      gyroRoll = v.un.gyroscope.x;
      gyroPitch = v.un.gyroscope.y;
    }
  }
}

void readFeet() {
  for (int i = 0; i < 6; i++) foot[i] = digitalRead(pins::FOOT[i]) == LOW;   // switch closes to GND when loaded
}

// ---------------------------------------------------------------- host / brain line protocol
// in : S,<deg x18..23>          joint bridge (as rev B)
//      C,vx,vz,yaw,roll,pitch,height,gait,flags   command mode (m/s, rad/s, deg, m; flags bit0 terrain, bit1 fire, bit2 sprint)
//      R,<18 deg>               learned-policy residuals added to the gait's joint angles (brain or host, 50 Hz)
//      A,0|1                    disarm / arm (host only; ignored while an RC link is up - the radio arms)
// out: B,<mV>,<mA>,<degC>,<status>             2 Hz  (unchanged from rev B)
//      T,roll,pitch,yaw,feet,margin_mm,mode,source,lq   10 Hz
struct LineIn {
  char buf[200];
  int n = 0;
};
LineIn hostLine, brainLine;

bool parseCmd(char* s, Command& c) {
  float v[8] = {0, 0, 0, 0, 0, cfg::STAND_HEIGHT, 0, 0};
  int k = 0;
  for (char* t = strtok(s + 2, ","); t && k < 8; t = strtok(nullptr, ",")) v[k++] = atof(t);
  if (k < 3) return false;
  c.vx = fminf(fmaxf(v[0], -cfg::SPEED_SPRINT), cfg::SPEED_SPRINT);
  c.vz = fminf(fmaxf(v[1], -cfg::SPEED_SPRINT), cfg::SPEED_SPRINT);
  c.yaw = fminf(fmaxf(v[2], -cfg::TURN_RATE), cfg::TURN_RATE);
  c.roll = fminf(fmaxf(v[3], -cfg::POSE_ROLL_MAX), cfg::POSE_ROLL_MAX) * DEG;
  c.pitch = fminf(fmaxf(v[4], -cfg::POSE_PITCH_MAX), cfg::POSE_PITCH_MAX) * DEG;
  c.height = fminf(fmaxf(v[5], cfg::HEIGHT_MIN), cfg::HEIGHT_MAX);
  c.gait = (GaitType)((int)v[6] % 3);
  int f = (int)v[7];
  c.terrain = f & 1; c.fire = f & 2; c.sprint = f & 4;
  c.stamp = millis();
  return true;
}

void handleLine(char* s, bool fromHost) {
  if (s[0] == 'C' && s[1] == ',') {
    if (parseCmd(s, fromHost ? cmdHost : cmdBrain)) (fromHost ? lastHostMs : lastBrainMs) = millis();
  } else if (fromHost && s[0] == 'S' && s[1] == ',') {
    nJoint = 0;
    for (char* t = strtok(s + 2, ","); t && nJoint < 23; t = strtok(nullptr, ",")) jointDeg[nJoint++] = atof(t);
    hostJointFresh = nJoint >= 18;
    lastHostMs = millis();
  } else if (s[0] == 'R' && s[1] == ',') {
    // residual joint offsets (deg) from the RL policy, added on top of the gait's IK (clamped +-15 deg)
    int k = 0;
    for (char* t = strtok(s + 2, ","); t && k < 18; t = strtok(nullptr, ",")) residual[k++] = fminf(fmaxf(atof(t), -15.f), 15.f) * DEG;
    if (k == 18) lastResidualMs = millis();
  } else if (fromHost && s[0] == 'A' && s[1] == ',') {
    if (millis() - lastRcMs > cfg::RC_TIMEOUT_MS) armed = atoi(s + 2) == 1;
  }
}

void pumpLines(Stream& in, LineIn& L, bool fromHost) {
  while (in.available()) {
    char ch = in.read();
    if (ch == '\n' || ch == '\r') {
      if (L.n) { L.buf[L.n] = 0; handleLine(L.buf, fromHost); L.n = 0; }
    } else if (L.n < (int)sizeof(L.buf) - 1) {
      L.buf[L.n++] = ch;
    }
  }
}

// ---------------------------------------------------------------- RC mapping (radio/README.md)
// ch1 roll stick = strafe vx, ch2 pitch stick = forward vz, ch3 throttle = body height,
// ch4 yaw stick = turn, ch5 SA arm, ch6 SB gait (tripod/ripple/wave), ch7 SC mode
// (manual / pose / auto), ch8 SD terrain, ch9 SE fire (momentary), ch10 SF kill (e-stop).
void readRc() {
  while (RcSerial.available()) {
    if (rc.feed(RcSerial.read()) && rc.newChannels) {
      rc.newChannels = false;
      lastRcMs = millis();
      float roll = crsf::norm(rc.ch[0]), fwd = crsf::norm(rc.ch[1]), thr = crsf::norm(rc.ch[2]), yaw = crsf::norm(rc.ch[3]);
      rcArmSwitch = rc.ch[4] > 1500;
      int gaitSw = rc.ch[5] < 600 ? 0 : (rc.ch[5] < 1400 ? 1 : 2);
      int modeSw = rc.ch[6] < 600 ? 0 : (rc.ch[6] < 1400 ? 1 : 2);
      rcKill = rc.ch[9] > 1500;
      rcAuto = modeSw == 2;
      Command& c = cmdRc;
      c.gait = (GaitType)gaitSw;
      c.terrain = rc.ch[7] > 1500;
      c.fire = rc.ch[8] > 1500;
      c.height = cfg::HEIGHT_MIN + (thr + 1) * 0.5f * (cfg::HEIGHT_MAX - cfg::HEIGHT_MIN);
      if (modeSw == 1) {          // pose mode: sticks tilt / twist the body, no walking
        c.vx = c.vz = c.yaw = 0;
        c.roll = -roll * cfg::POSE_ROLL_MAX * DEG;
        c.pitch = fwd * cfg::POSE_PITCH_MAX * DEG;
        c.bodyYaw = -yaw * cfg::POSE_YAW_MAX * DEG;
      } else {
        auto dz = [](float x) { return fabsf(x) < 0.05f ? 0.f : x; };
        c.vx = -dz(roll) * cfg::SPEED_WALK;
        c.vz = dz(fwd) * cfg::SPEED_WALK;
        c.yaw = -dz(yaw) * cfg::TURN_RATE;
        c.roll = c.pitch = c.bodyYaw = 0;
      }
      c.stamp = millis();
    }
  }
}

void sendRcTelemetry() {
  static uint8_t k = 0;
  uint8_t buf[32];
  int n = 0;
  switch (k++ % 3) {
    case 0: {
      float pct = fminf(fmaxf((vbat - 9.9f) / (12.6f - 9.9f), 0.f), 1.f) * 100;
      n = crsf::battery(vbat, ibat, (uint32_t)mahUsed, (uint8_t)pct, buf);
      break;
    }
    case 1: n = crsf::attitude(imuPitch, imuRoll, imuYaw, buf); break;
    case 2: {
      static char m[16];
      snprintf(m, sizeof m, "%s%s", lvc ? "LOWBAT" : (!armed ? "SAFE" : (source == Source::Brain ? "AUTO" : "MAN")),
               strcmp(fault, "OK") ? "!" : "");
      n = crsf::flightMode(m, buf);
      break;
    }
  }
  RcSerial.write(buf, n);
}

// ---------------------------------------------------------------- arbitration + safety (CONTROL.md 6, 7)
const Command* arbitrate() {
  uint32_t now = millis();
  bool rcUp = now - lastRcMs < cfg::RC_TIMEOUT_MS;
  bool hostUp = now - lastHostMs < cfg::HOST_TIMEOUT_MS;
  bool brainUp = now - lastBrainMs < cfg::HOST_TIMEOUT_MS;
  if (rcUp) {
    armed = rcArmSwitch && !rcKill;
    if (!armed) { source = Source::RC; return nullptr; }
    bool sticksActive = fabsf(cmdRc.vx) + fabsf(cmdRc.vz) + fabsf(cmdRc.yaw) > 0.01f;
    if (rcAuto && !sticksActive) {                 // autonomy, RC sticks override instantly
      if (brainUp) { source = Source::Brain; return &cmdBrain; }
      if (hostUp) { source = Source::Host; return &cmdHost; }
    }
    source = Source::RC;
    return &cmdRc;
  }
  // radio lost after being used: stop, crouch, and go limp after 2 s (failsafe)
  if (lastRcMs && now - lastRcMs > 2000) armed = false;
  // no radio: tethered testing from the USB host only (brain autonomy needs the radio as e-stop)
  if (hostUp && armed) { source = Source::Host; return &cmdHost; }
  source = Source::None;
  return nullptr;
}

void safety() {
  uint32_t now = millis();
  if (vbat < cfg::VBAT_LVC) { if (!lowSince) lowSince = now; if (now - lowSince > cfg::LVC_DEBOUNCE_MS) lvc = true; }
  else { lowSince = 0; if (vbat > cfg::VBAT_RECOVER) lvc = false; }
  bool tipped = imuOk && (fabsf(imuRoll) > cfg::TIP_DEG * DEG || fabsf(imuPitch) > cfg::TIP_DEG * DEG);
  fault = lvc ? "LOWBAT" : (tipped ? "TIPPED" : (hottest >= cfg::SERVO_TEMP_STOP ? "HOT" : (margin < 0 ? "UNSTABLE" : "OK")));
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);                                            // USB CDC (host)
  BusSerial.begin(cfg::BUS_BAUD, SERIAL_8N1, pins::BUS_RX, pins::BUS_TX);
  st.pSerial = &BusSerial;
  RcSerial.begin(420000, SERIAL_8N1, pins::CRSF_RX, pins::CRSF_TX);
  // J1 role: the brain's UART idles high on AUX2; weapon modules tie AUX2 to GND through 10k
  pinMode(pins::J1_AUX2, INPUT);
  delay(5);
  brainOnJ1 = digitalRead(pins::J1_AUX2) == HIGH;
  if (brainOnJ1) Serial0.begin(921600, SERIAL_8N1, pins::J1_AUX2, pins::J1_AUX1);
  else { pinMode(pins::J1_AUX1, OUTPUT); digitalWrite(pins::J1_AUX1, LOW); }
  for (int i = 0; i < 6; i++) pinMode(pins::FOOT[i], INPUT_PULLUP);
  pinMode(pins::STATUS_LED, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);
  imuOk = imu.begin_I2C(0x4A, &Wire);
  if (imuOk) {
    imu.enableReport(SH2_GAME_ROTATION_VECTOR, 1000000 / cfg::IMU_HZ);   // no magnetometer: no indoor heading drift jumps
    imu.enableReport(SH2_GYROSCOPE_CALIBRATED, 1000000 / cfg::IMU_HZ);
  }
  tof.setTimeout(20);
  tofOk = tof.init();
  if (tofOk) { tof.setDistanceMode(VL53L1X::Short); tof.setMeasurementTimingBudget(20000); tof.startContinuous(25); }
  gait.init();
  servoTorque(false);                                              // limp until armed
}

void loop() {
  static uint32_t tCtl = 0, tTel = 0, tB = 0, tRcT = 0, tAdc = 0, tTemp = 0, armedAt = 0;
  static bool wasArmed = false;
  uint32_t now = millis();

  readRc();
  pumpLines(Serial, hostLine, true);
  if (brainOnJ1) pumpLines(Serial0, brainLine, false);
  if (imuOk) readImu();
  readFeet();
  if (tofOk && tof.dataReady()) tofMm = tof.read(false);

  if (now - tCtl >= 1000 / cfg::CONTROL_HZ) {
    float dt = (now - tCtl) / 1000.0f;
    tCtl = now;
    if (dt > 0.05f) dt = 0.01f;
    safety();
    const Command* c = arbitrate();
    bool stop = !armed || lvc || strcmp(fault, "TIPPED") == 0;
    if (armed && !wasArmed) { servoTorque(true); armedAt = now; bal.reset(); }
    if (stop) {
      if (wasArmed || mode != Mode::Relaxed) servoTorque(false);
      mode = Mode::Relaxed;
      wasArmed = false;
    } else if (source == Source::Host && hostJointFresh && now - lastHostMs < cfg::HOST_TIMEOUT_MS && c == &cmdHost &&
               cmdHost.stamp + cfg::HOST_TIMEOUT_MS < now) {
      // joint-bridge mode: the simulator drives the servos directly (rev B behaviour)
      mode = Mode::JointBridge;
      writeJoints(jointDeg, nJoint, now - armedAt < 2000 ? cfg::SPEED_SOFTSTART : cfg::SPEED_RUN);
      wasArmed = true;
    } else {
      static Command hold;
      if (c) hold = *c; else { hold.vx = hold.vz = hold.yaw = 0; hold.height = cfg::HEIGHT_MIN; }   // no source: stop and crouch
      GaitInput in;
      in.vx = hold.vx; in.vz = hold.vz; in.yawRate = hold.yaw;
      in.type = hold.gait;
      in.terrain = hold.terrain;
      in.stepHz = hold.sprint ? cfg::STEP_HZ_SPRINT : cfg::STEP_HZ_WALK;
      in.stepHeight = hold.terrain ? cfg::STEP_HEIGHT_TERRAIN : cfg::STEP_HEIGHT;
      for (int i = 0; i < 6; i++) in.contact[i] = foot[i];
      // balance: level the body in the world (target = requested pose), feed-forward = requested pose
      bal.enabled = imuOk;
      bal.update(dt, hold.roll, hold.pitch, imuRoll, imuPitch, gyroRoll, gyroPitch);
      in.pose.roll = hold.roll + bal.outRoll;
      in.pose.pitch = hold.pitch + bal.outPitch;
      in.pose.yaw = hold.bodyYaw;
      in.pose.height = (hold.terrain ? fmaxf(hold.height, cfg::TERRAIN_HEIGHT) : hold.height) + gait.terrainShift;
      // stability: shift the body towards the support polygon and slow down when the margin is thin
      float sf[6][2];
      int n = 0;
      for (int i = 0; i < 6; i++) if (gait.stance[i]) { sf[n][0] = gait.feet[i].x; sf[n][1] = gait.feet[i].z; n++; }
      float comX = in.pose.height * tanf(in.pose.roll), comZ = -in.pose.height * tanf(in.pose.pitch);   // CoM shift from tilt
      margin = stabilityMargin(sf, n, comX, comZ);
      if (margin < cfg::MARGIN_MIN && hold.gait == GaitType::Tripod) in.type = GaitType::Ripple;
      gait.update(dt, in);
      if (now - lastResidualMs < 100 && source != Source::RC) {      // policy residuals (stale after 100 ms -> pure classical gait)
        for (int l = 0; l < 6; l++) {
          gait.angles[l].coxa += residual[l * 3];
          gait.angles[l].femur += residual[l * 3 + 1];
          gait.angles[l].tibia += residual[l * 3 + 2];
          clampJoints(gait.angles[l]);
        }
      }
      writeLegs(gait.angles, now - armedAt < 2000 ? cfg::SPEED_SOFTSTART : cfg::SPEED_RUN);
      mode = (fabsf(hold.vx) + fabsf(hold.vz) + fabsf(hold.yaw) > 0.01f) ? Mode::Walk : Mode::Stand;
      wasArmed = true;
      if (!brainOnJ1) digitalWrite(pins::J1_AUX1, hold.fire);           // weapon fire line
    }
  }

  if (now - tAdc >= 100) { tAdc = now; pollBridgeAdc(); mahUsed += ibat * 1000.0f * 0.1f / 3600.0f; }
  if (now - tTemp >= 110) {                      // one servo per pass (~0.3 ms on the bus): all 18 every 2 s
    tTemp = now;
    static uint8_t k = 0, temps[18] = {0};
    int t = st.ReadTemper(cfg::BUS_ID[k / 3][k % 3]);
    if (t > 0) temps[k] = t;
    k = (k + 1) % 18;
    uint8_t h = 0;
    for (int i = 0; i < 18; i++) h = temps[i] > h ? temps[i] : h;
    hottest = h;
  }
  if (now - tRcT >= 50 && now - lastRcMs < cfg::RC_TIMEOUT_MS) { tRcT = now; sendRcTelemetry(); }
  if (now - tTel >= 1000 / cfg::TELEM_HZ) {
    tTel = now;
    uint8_t feet = 0;
    for (int i = 0; i < 6; i++) feet |= foot[i] << i;
    char line[120];
    snprintf(line, sizeof line, "T,%.1f,%.1f,%.1f,%u,%d,%d,%d,%u\n", imuRoll / DEG, imuPitch / DEG, imuYaw / DEG, feet,
             (int)(margin * 1000), (int)mode, (int)source, rc.link.lq);
    Serial.print(line);
    if (brainOnJ1) Serial0.print(line);
  }
  if (now - tB >= 500) {
    tB = now;
    char line[64];
    snprintf(line, sizeof line, "B,%d,%d,%u,%s\n", (int)(vbat * 1000), (int)(ibat * 1000), hottest,
             lvc ? "LOW" : (now - lastHostMs > cfg::HOST_TIMEOUT_MS ? "NOLINK" : "OK"));
    Serial.print(line);
  }
  digitalWrite(pins::STATUS_LED, armed ? ((now / 100) % 2) : ((now / 1000) % 2));
}
