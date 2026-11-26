// Hexapod bridge (Mk1 rev B): receives servo angles from the Robot Sim over USB
// and drives the Feetech STS3215 / STS3250 serial-bus servos.
//
// Board:   Waveshare "Servo Driver with ESP32" (ESP32-WROOM, USB-C). In the
//          Arduino IDE pick "ESP32 Dev Module".
// Library: Feetech / Waveshare "SCServo" (SMS_STS class), from the ST3215 wiki:
//          https://www.waveshare.com/wiki/ST3215_Servo  (ST_Servo.zip)
// Wiring:  servo bus UART on GPIO18 (RX) / GPIO19 (TX) at 1 Mbaud (on the board).
//          GPIO35 <- ACS724 output through a 10k/20k divider (0-5 V -> 0-3.3 V)
//          GPIO34 <- battery voltage through a 100k/22k divider (12.6 V -> 2.3 V)
//
// USB protocol - UNCHANGED from the PCA9685 version, so the simulator needs no
// change to drive the robot:
//   sim -> robot, 25 lines/s:  S,<a0>,<a1>,...,<aN>\n   whole degrees 0-180,
//   order: legs 0..5 x (coxa, femur, tibia), then attachment servos.
//   90 = the servo's centre. The sim already applies servos.direction / trim.
// New, robot -> sim, 2 lines/s (the sim may ignore it):
//   B,<battery mV>,<current mA>,<hottest servo degC>,<state>\n
//   state: OK, LOW (below cutoff, servos relaxed) or NOLINK.
//
// Servo IDs (set each servo once with Waveshare's / Feetech's ID tool before
// assembly; write the ID on the case):
//   leg L joint J -> ID 1 + 3*L + J     (legs 0-5 as in hexapod.config.js,
//                                       joints 0 coxa, 1 femur, 2 tibia)
//   attachment servo k (0-4)        -> ID 21 + k
// Angle -> position: the STS3215 has 4096 steps per turn, 2048 = centre.

#include <SCServo.h>

SMS_STS st;

#define S_RXD 18
#define S_TXD 19
#define PIN_CURRENT 35
#define PIN_VBAT 34

const uint8_t N_LEG = 18;
const uint8_t N_AUX = 5;
const uint8_t N_ALL = N_LEG + N_AUX;
uint8_t ids[N_ALL];

const float STEPS_PER_DEG = 4096.0f / 360.0f;
const u16 SPEED_SOFT = 600;     // steps/s for the first 2 s after power-up / relink
const u16 SPEED_RUN = 2400;
const u8 ACC = 50;
const unsigned long TIMEOUT_MS = 500;

// battery: 3S LiPo. Cut off at 3.4 V/cell under load, recover above 3.65 V/cell.
const float VBAT_CUTOFF = 10.2f;
const float VBAT_RECOVER = 11.0f;
const float VBAT_DIV = (100.0f + 22.0f) / 22.0f;
const float ACS_MV_PER_A = 66.0f;       // ACS724LLCTR-30AB at 5 V
const float ACS_DIV = (10.0f + 20.0f) / 20.0f;

char line[256];
uint8_t lineLen = 0;
unsigned long lastFrame = 0, linkStart = 0, lowSince = 0, lastTelemetry = 0;
bool relaxed = true;
bool lowBattery = false;
s16 pos[N_ALL];
u16 spd[N_ALL];
u8 acc[N_ALL];

float readVbat() {
  // ESP32 ADC is not very linear; calibrate once against a multimeter if needed.
  return analogReadMilliVolts(PIN_VBAT) / 1000.0f * VBAT_DIV;
}

float readCurrent() {
  float v = analogReadMilliVolts(PIN_CURRENT) * ACS_DIV;   // mV at the sensor
  return (v - 2500.0f) / ACS_MV_PER_A;                      // A, + = discharge
}

void relaxAll() {
  for (uint8_t k = 0; k < N_ALL; k++) st.EnableTorque(ids[k], 0);
  relaxed = true;
}

void handleLine(char *s) {
  if (s[0] != 'S' || s[1] != ',') return;
  if (lowBattery) return;                          // ignore motion until the battery is swapped / charged
  if (relaxed) linkStart = millis();
  bool soft = millis() - linkStart < 2000;
  uint8_t n = 0;
  char *tok = strtok(s + 2, ",");
  while (tok && n < N_ALL) {
    int deg = constrain(atoi(tok), 0, 180);
    pos[n] = (s16)(2048 + (deg - 90) * STEPS_PER_DEG);
    spd[n] = soft ? SPEED_SOFT : SPEED_RUN;
    acc[n] = ACC;
    n++;
    tok = strtok(NULL, ",");
  }
  if (n == 0) return;
  st.SyncWritePosEx(ids, n, pos, spd, acc);        // also re-enables torque
  lastFrame = millis();
  relaxed = false;
}

void telemetry() {
  float v = readVbat();
  float a = readCurrent();
  int hottest = 0;
  static uint8_t probe = 0;                        // read one servo's temperature per call
  int t = st.ReadTemper(ids[probe]);
  probe = (probe + 1) % N_LEG;
  if (t > hottest) hottest = t;
  const char *state = lowBattery ? "LOW" : (relaxed ? "NOLINK" : "OK");
  Serial.printf("B,%d,%d,%d,%s\n", (int)(v * 1000), (int)(a * 1000), hottest, state);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  st.pSerial = &Serial1;
  analogReadResolution(12);
  for (uint8_t k = 0; k < N_LEG; k++) ids[k] = 1 + k;
  for (uint8_t k = 0; k < N_AUX; k++) ids[N_LEG + k] = 21 + k;
  delay(500);
  relaxAll();
  Serial.println("hexapod_bridge (bus servos) ready");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line[lineLen] = 0;
      handleLine(line);
      lineLen = 0;
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineLen = 0;  // line too long, drop it
    }
  }
  if (!relaxed && millis() - lastFrame > TIMEOUT_MS) relaxAll();

  // low-voltage cutoff with 3 s debounce (servo current spikes sag the pack briefly)
  float v = readVbat();
  if (!lowBattery && v < VBAT_CUTOFF) {
    if (lowSince == 0) lowSince = millis();
    if (millis() - lowSince > 3000) {
      lowBattery = true;
      relaxAll();
      Serial.println("LOW BATTERY: servos relaxed. Switch off and charge the pack.");
    }
  } else if (v >= VBAT_CUTOFF) {
    lowSince = 0;
  }
  if (lowBattery && v > VBAT_RECOVER) lowBattery = false;

  if (millis() - lastTelemetry > 500) {
    lastTelemetry = millis();
    telemetry();
  }
}
