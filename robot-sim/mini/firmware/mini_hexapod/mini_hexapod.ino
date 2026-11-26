// Hexapod Mini - remote-control firmware.
//
// Board:   ESP32-DevKitC (ESP32-WROOM-32E, classic ESP32: needed for Bluetooth Classic
//          gamepads).  Arduino IDE: add the Bluepad32 board package
//          (https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json)
//          and pick  Tools > Board > esp32_bluepad32 > "ESP32 Dev Module".
// Library: "Adafruit PWM Servo Driver Library" (Library Manager).  Bluepad32 comes with
//          the board package.
// Gamepad: any Bluepad32 gamepad (8BitDo Lite 2 in Switch mode, DualShock 4, DualSense,
//          Switch Pro, Xbox Wireless).  Pair: power the robot, hold the pad's pairing
//          button; the robot takes the first pad that connects.
//
// Controls (docs/DESIGN.md section 6):
//   A (Cross)          arm: servos on, stand up            B (Circle)   sit down, servos off
//   left stick         walk (up = forward) and strafe      right stick X  turn
//   d-pad up / down    body height                         R1           hold for fast steps
//   Failsafe: no pad data for 0.3 s -> stop; for 3 s -> sit and go limp.  Battery below
//   6.6 V for 3 s -> sit and go limp (re-arm needs > 7.0 V).
//
// USB serial 115200 - the Robot Sim protocol (same as robot-sim/firmware/hexapod_bridge):
//   sim -> robot   S,<a0>,...,<a17>\n   servo degrees 0-180, legs 0..5 x (coxa, femur, tibia)
//                  A,1 / A,0            arm / disarm USB mode (only while no gamepad is armed)
//   robot -> sim   B,<battery mV>,0,0,<OK|LOW|NOLINK>\n   at 2 Hz
//   In USB mode the servos follow the S lines and go limp 0.5 s after the last one.
//   The sim must use this robot's directions (all -1) and geometry - see firmware/README.md.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Bluepad32.h>
#include "config.h"
#include "kinematics.h"

Adafruit_PWMServoDriver pca[2] = {Adafruit_PWMServoDriver(PCA_ADDR[0], Wire), Adafruit_PWMServoDriver(PCA_ADDR[1], Wire)};
ControllerPtr pad = nullptr;
TripodGait gait;

enum Mode { LIMP, STANDING_UP, WALK, SITTING_DOWN, USB_DRIVE };
Mode mode = LIMP;
float angles[18], servoDeg[18], sitAngles[18], standAngles[18];
float rampT = 0.0f;
uint32_t lastPadMs = 0, lastUsbMs = 0, lowSinceMs = 0, lastTeleMs = 0, lastLoopUs = 0, lastRumbleMs = 0;
float vbat = 0.0f;
bool batteryLow = false;
char line[200];
int lineLen = 0;

// ------------------------------------------------------------------ servo output
void outputsEnabled(bool on) { digitalWrite(PIN_OE, on ? LOW : HIGH); }

void writeServo(int leg, int joint, float deg) {
  int board = leg < 3 ? 0 : 1;
  float us = SERVO_US_MIN + (SERVO_US_MAX - SERVO_US_MIN) * deg / 180.0f;
  pca[board].writeMicroseconds(CHANNEL[leg][joint], (uint16_t)us);
}

void writeAll(const float deg[18]) {
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 3; j++) writeServo(i, j, deg[i * 3 + j]);
}

// a pose with every foot at its neutral point and the hip plane at height h
void poseAt(float h, float out[18]) {
  TripodGait g;
  g.setHeight(h);
  g.update(0.02f, 0, 0, 0, out);
}

// ------------------------------------------------------------------ gamepad
void onConnected(ControllerPtr c) {
  if (pad == nullptr) {
    pad = c;
    lastPadMs = millis();
    c->setColorLED(0, 0, 255);
  } else {
    c->disconnect();      // one pad at a time
  }
}

void onDisconnected(ControllerPtr c) {
  if (c == pad) pad = nullptr;
}

float stick(int32_t v) {             // Bluepad32 axes are -511..512; 12 % dead zone
  float x = v / 512.0f;
  if (fabsf(x) < 0.12f) return 0.0f;
  return (x - (x > 0 ? 0.12f : -0.12f)) / 0.88f;
}

// ------------------------------------------------------------------ battery
float readBattery() {
  uint32_t mv = analogReadMilliVolts(PIN_VBAT);
  return mv / 1000.0f * VBAT_DIVIDER * VBAT_CAL;
}

// ------------------------------------------------------------------ USB protocol
void handleLine(char *s) {
  if (s[0] == 'A' && s[1] == ',') {
    bool padArmed = mode == WALK || mode == STANDING_UP;
    if (s[2] == '1' && !padArmed && !batteryLow) {
      mode = USB_DRIVE;
      lastUsbMs = millis();
    } else if (s[2] == '0' && mode == USB_DRIVE) {
      mode = LIMP;
      outputsEnabled(false);
    }
    return;
  }
  if (s[0] != 'S' || s[1] != ',' || mode != USB_DRIVE) return;
  float deg[18];
  int n = 0;
  char *p = s + 2;
  while (n < 18 && *p) {
    deg[n++] = strtof(p, &p);
    if (*p == ',') p++;
  }
  if (n < 18) return;
  for (int k = 0; k < 18; k++) {
    float d = deg[k] + TRIM[k / 3][k % 3];
    servoDeg[k] = d < SERVO_DEG_MIN ? SERVO_DEG_MIN : (d > SERVO_DEG_MAX ? SERVO_DEG_MAX : d);
  }
  writeAll(servoDeg);
  outputsEnabled(true);
  lastUsbMs = millis();
}

void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) {
        line[lineLen] = 0;
        handleLine(line);
      }
      lineLen = 0;
    } else if (lineLen < (int)sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}

// ------------------------------------------------------------------ setup / loop
void setup() {
  pinMode(PIN_OE, OUTPUT);
  outputsEnabled(false);                 // limp until armed
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  for (auto &b : pca) {
    b.begin();
    b.setOscillatorFrequency(25000000);  // trim if the measured period is off (BUILD.md step 9)
    b.setPWMFreq(50);
  }
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
  BP32.setup(&onConnected, &onDisconnected);
  BP32.forgetBluetoothKeys();            // pair fresh each power-up: no surprise pads
  poseAt(SIT_HEIGHT, sitAngles);
  poseAt(STAND_HEIGHT, standAngles);
  vbat = readBattery();
  lastLoopUs = micros();
}

void loop() {
  BP32.update();
  pollSerial();
  uint32_t now = millis();
  uint32_t us = micros();
  float dt = (us - lastLoopUs) * 1e-6f;
  if (dt < 1.0f / LOOP_HZ) return;
  lastLoopUs = us;
  if (dt > 0.1f) dt = 0.1f;

  // battery (filtered) and low-voltage cutoff
  vbat += (readBattery() - vbat) * 0.05f;
  bool bench = vbat < 3.0f;            // no pack: on USB or a bench supply (servo calibration)
  if (vbat < VBAT_CUTOFF && !bench) {
    if (!lowSinceMs) lowSinceMs = now;
    if (now - lowSinceMs > 3000) batteryLow = true;
  } else {
    lowSinceMs = 0;
    if (vbat > VBAT_RECOVER) batteryLow = false;
  }

  // gamepad
  float vx = 0, vz = 0, w = 0;
  bool armPress = false, sitPress = false;
  if (pad && pad->isConnected() && pad->hasData()) {
    lastPadMs = now;
    vz = -stick(pad->axisY()) * V_MAX;        // stick up = forward
    vx = -stick(pad->axisX()) * V_MAX;        // stick right = strafe right (body x = left)
    w = -stick(pad->axisRX()) * W_MAX;        // right stick right = turn right
    armPress = pad->a();
    sitPress = pad->b();
    gait.setFast(pad->r1());
    uint8_t dp = pad->dpad();
    float h = gait.height();
    if (dp & DPAD_UP) h += 0.02f * dt;
    if (dp & DPAD_DOWN) h -= 0.02f * dt;
    gait.setHeight(h < HEIGHT_MIN ? HEIGHT_MIN : (h > HEIGHT_MAX ? HEIGHT_MAX : h));
    if (vbat < VBAT_WARN && now - lastRumbleMs > 5000) {   // low-battery warning every 5 s
      lastRumbleMs = now;
      pad->playDualRumble(0, 150, 0x40, 0x40);
    }
  }
  uint32_t padAge = now - lastPadMs;
  if (!pad || padAge > PAD_STOP_MS) vx = vz = w = 0;     // failsafe: stop

  switch (mode) {
    case LIMP:
      outputsEnabled(false);
      if (armPress && !batteryLow && pad && padAge < PAD_STOP_MS) {
        rampT = 0;
        anglesToServo(sitAngles, servoDeg);   // limp servos jump to the sit pose (belly on the floor)
        writeAll(servoDeg);
        outputsEnabled(true);
        mode = STANDING_UP;
      }
      break;
    case STANDING_UP:
    case SITTING_DOWN: {
      rampT += dt / 1.5f;              // 1.5 s from sitting to standing or back
      float u = rampT > 1 ? 1 : rampT;
      u = u * u * (3 - 2 * u);
      for (int k = 0; k < 18; k++)
        angles[k] = mode == STANDING_UP ? sitAngles[k] + (standAngles[k] - sitAngles[k]) * u
                                        : standAngles[k] + (sitAngles[k] - standAngles[k]) * u;
      anglesToServo(angles, servoDeg);
      writeAll(servoDeg);
      if (rampT >= 1) {
        if (mode == STANDING_UP) {
          gait.setHeight(STAND_HEIGHT);
          mode = WALK;
        } else {
          outputsEnabled(false);
          mode = LIMP;
        }
      }
      break;
    }
    case WALK:
      gait.update(dt, vx, vz, w, angles);
      anglesToServo(angles, servoDeg);
      writeAll(servoDeg);
      if ((sitPress || batteryLow || !pad || padAge > PAD_SIT_MS)) {
        // finish the step first so the sit starts from all feet down
        if (gait.atRest() || padAge > PAD_SIT_MS) {
          poseAt(gait.height(), standAngles);
          rampT = 0;
          mode = SITTING_DOWN;
        }
      }
      break;
    case USB_DRIVE:
      if (now - lastUsbMs > USB_RELAX_MS || batteryLow) {
        outputsEnabled(false);
        mode = LIMP;
      }
      break;
  }
  if (mode == LIMP) poseAt(STAND_HEIGHT, standAngles);

  digitalWrite(PIN_LED, mode == LIMP ? ((now / 500) & 1) : HIGH);
  if (now - lastTeleMs > 500) {
    lastTeleMs = now;
    const char *st = batteryLow ? "LOW" : ((pad || mode == USB_DRIVE) ? "OK" : "NOLINK");
    Serial.printf("B,%d,0,0,%s\n", (int)(vbat * 1000), st);
  }
}
