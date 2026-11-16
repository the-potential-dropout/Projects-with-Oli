// Hexapod bridge: receives servo angles from the Robot Sim over USB serial and
// drives the real servos through two PCA9685 boards.
//
// Board: ESP32 (or Arduino Mega). Library: "Adafruit PWM Servo Driver Library".
// Wiring: PCA9685 #1 at 0x40 (channels 0-15), #2 at 0x41 (channels 16-31),
//         SDA/SCL to the board's I2C pins, servo power from a 5-6 V UBEC.
//
// Protocol (from the simulator, 25 lines per second):
//   S,<a0>,<a1>,...,<aN>\n     whole degrees 0-180, order:
//   legs 0..5 x (coxa, femur, tibia), then attachment servos.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver boardA(0x40);
Adafruit_PWMServoDriver boardB(0x41);

// Must match servos.channels in src/robots/hexapod.config.js
const uint8_t LEG_CHANNELS[18] = {
  0, 1, 2,    // front right
  4, 5, 6,    // middle right
  8, 9, 10,   // rear right
  16, 17, 18, // front left
  20, 21, 22, // middle left
  24, 25, 26, // rear left
};
// Attachment servos (turret pitch, or arm base/shoulder/elbow/wrist/gripper)
const uint8_t AUX_CHANNELS[5] = {12, 13, 14, 15, 28};

const uint16_t SERVO_MIN_US = 500;  // pulse width at 0 degrees
const uint16_t SERVO_MAX_US = 2500; // pulse width at 180 degrees
const unsigned long TIMEOUT_MS = 500;

char line[256];
uint8_t lineLen = 0;
unsigned long lastFrame = 0;
bool relaxed = true;

void writeServo(uint8_t channel, int degrees) {
  degrees = constrain(degrees, 0, 180);
  uint16_t us = map(degrees, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  // PCA9685 at 50 Hz: 4096 ticks per 20000 us
  uint16_t ticks = (uint32_t)us * 4096 / 20000;
  if (channel < 16) boardA.setPWM(channel, 0, ticks);
  else boardB.setPWM(channel - 16, 0, ticks);
}

void relaxAll() {
  // Stop sending pulses so servos go limp (safe if the link drops)
  for (uint8_t c = 0; c < 16; c++) {
    boardA.setPWM(c, 0, 0);
    boardB.setPWM(c, 0, 0);
  }
  relaxed = true;
}

void handleLine(char *s) {
  if (s[0] != 'S' || s[1] != ',') return;
  int index = 0;
  char *tok = strtok(s + 2, ",");
  while (tok && index < 18 + 5) {
    int deg = atoi(tok);
    if (index < 18) writeServo(LEG_CHANNELS[index], deg);
    else writeServo(AUX_CHANNELS[index - 18], deg);
    index++;
    tok = strtok(NULL, ",");
  }
  lastFrame = millis();
  relaxed = false;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  boardA.begin();
  boardB.begin();
  boardA.setPWMFreq(50);
  boardB.setPWMFreq(50);
  relaxAll();
  Serial.println("hexapod_bridge ready");
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
      lineLen = 0; // line too long, drop it
    }
  }
  if (!relaxed && millis() - lastFrame > TIMEOUT_MS) relaxAll();
}
