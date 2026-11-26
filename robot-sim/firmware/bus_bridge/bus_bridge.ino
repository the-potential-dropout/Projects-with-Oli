// Hexapod Mk1 rev B2 - servo-bus bridge for the Waveshare "Servo Driver with ESP32".
//
// The board keeps the job it is good at: its half-duplex circuit and DC jack drive
// the Feetech serial bus.  The Feather ESP32-S3 main controller talks to it over a
// 3-wire UART on the board's I2C/OLED header (GPIO21 = RX from the Feather,
// GPIO22 = TX to the Feather, GND); this sketch passes bytes both ways at 1 Mbaud,
// so the Feather's SCServo library behaves exactly as if it owned the bus.
//
// It also measures the battery (VBAT divider 100k/22k on GPIO34, ACS724 66 mV/A via
// 10k/20k on GPIO35) and answers a 3-byte local query that can't be confused with a
// servo packet (those start with FF FF):
//   Feather -> bridge  A5 5A 01
//   bridge  -> Feather A5 5A 01 mV_hi mV_lo mA_hi mA_lo sum   (sum = low byte of the four data bytes)
//
// Board: "ESP32 Dev Module".  The OLED is not used (its pins carry the UART).

#include <Arduino.h>

constexpr int BUS_RX = 18, BUS_TX = 19;      // to the servo bus circuit (as the Waveshare examples)
constexpr int LINK_RX = 21, LINK_TX = 22;    // to the Feather (header pins SDA / SCL)
constexpr int VBAT_PIN = 34, ISENSE_PIN = 35;
constexpr float VBAT_DIV = (100.0f + 22.0f) / 22.0f;
constexpr float IS_DIV = (10.0f + 20.0f) / 20.0f, ACS_MV_PER_A = 66.0f;

HardwareSerial& Bus = Serial1;
HardwareSerial& Link = Serial2;
float iZeroMv = 2500.0f;

static float readMv(int pin) {
  uint32_t s = 0;
  for (int i = 0; i < 8; i++) s += analogReadMilliVolts(pin);
  return s / 8.0f;
}

void setup() {
  Bus.begin(1000000, SERIAL_8N1, BUS_RX, BUS_TX);
  Link.begin(1000000, SERIAL_8N1, LINK_RX, LINK_TX);
  analogSetAttenuation(ADC_11db);
  delay(200);
  iZeroMv = readMv(ISENSE_PIN) * IS_DIV;     // switch the robot on with the servos limp: ~0 A
}

void loop() {
  static uint8_t q[3];
  static int qn = 0;
  static uint32_t lastByteUs = 0;
  while (Link.available()) {
    uint8_t b = Link.read();
    uint32_t now = micros();
    bool idle = now - lastByteUs > 150;      // a local query always starts after a quiet gap
    lastByteUs = now;
    if (qn == 0 && b == 0xA5 && idle) { q[qn++] = b; continue; }
    if (qn > 0) {
      q[qn++] = b;
      if (qn == 2 && b != 0x5A) { Bus.write(q, 2); qn = 0; continue; }
      if (qn == 3) {
        qn = 0;
        if (q[2] == 0x01) {
          uint16_t mv = (uint16_t)(readMv(VBAT_PIN) * VBAT_DIV);
          int16_t ma = (int16_t)((readMv(ISENSE_PIN) * IS_DIV - iZeroMv) / ACS_MV_PER_A * 1000.0f);
          uint8_t r[8] = {0xA5, 0x5A, 0x01, (uint8_t)(mv >> 8), (uint8_t)mv, (uint8_t)((uint16_t)ma >> 8), (uint8_t)ma, 0};
          r[7] = r[3] + r[4] + r[5] + r[6];
          Link.write(r, 8);
        } else {
          Bus.write(q, 3);
        }
      }
      continue;
    }
    Bus.write(b);
  }
  while (Bus.available()) Link.write(Bus.read());
}
