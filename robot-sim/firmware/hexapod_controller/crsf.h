// CRSF (Crossfire / ExpressLRS) receiver protocol: RC-channel parser and
// telemetry frames back to the radio.  Header-only, no Arduino calls, so it is
// unit-tested on the PC (firmware/test).  Protocol reference:
// https://github.com/crsf-wg/crsf/wiki and the ExpressLRS docs (CRSF over UART at 420000 baud).
#pragma once
#include <stdint.h>
#include <string.h>

namespace crsf {

constexpr uint8_t ADDR_FC = 0xC8;               // frames to / from the flight controller (us)
constexpr uint8_t T_RC_CHANNELS = 0x16;          // 16 x 11-bit channels, 22 bytes
constexpr uint8_t T_LINK_STATS = 0x14;
constexpr uint8_t T_BATTERY = 0x08;
constexpr uint8_t T_ATTITUDE = 0x1E;
constexpr uint8_t T_FLIGHT_MODE = 0x21;
constexpr uint16_t CH_MIN = 172, CH_MID = 992, CH_MAX = 1811;

inline uint8_t crc8(const uint8_t* p, int n) {    // poly 0xD5 (DVB-S2)
  uint8_t c = 0;
  for (int i = 0; i < n; i++) {
    c ^= p[i];
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1);
  }
  return c;
}

// -1 .. +1 stick value from a raw channel
inline float norm(uint16_t v) {
  float x = ((int)v - (int)CH_MID) / (float)(CH_MAX - CH_MID);
  return x < -1 ? -1 : (x > 1 ? 1 : x);
}

struct LinkStats {
  uint8_t rssi1 = 0, rssi2 = 0, lq = 0;
  int8_t snr = 0;
  uint8_t rfMode = 0, txPower = 0;
};

class Parser {
 public:
  uint16_t ch[16] = {};
  LinkStats link;
  uint32_t frames = 0, crcErrors = 0;
  bool newChannels = false;

  // feed one byte; returns true when a complete valid frame was handled
  bool feed(uint8_t b) {
    if (n_ == 0 && b != ADDR_FC && b != 0xEE && b != 0xEA) return false;   // wait for a sync / address byte
    buf_[n_++] = b;
    if (n_ == 2 && (buf_[1] < 2 || buf_[1] > 62)) { n_ = 0; return false; }
    if (n_ >= 2 && n_ == (unsigned)buf_[1] + 2) {
      int len = buf_[1];
      bool ok = crc8(&buf_[2], len - 1) == buf_[len + 1];
      if (ok) handle(buf_[2], &buf_[3], len - 2);
      else crcErrors++;
      n_ = 0;
      return ok;
    }
    if (n_ >= sizeof(buf_)) n_ = 0;
    return false;
  }

 private:
  uint8_t buf_[64];
  unsigned n_ = 0;

  void handle(uint8_t type, const uint8_t* p, int n) {
    frames++;
    if (type == T_RC_CHANNELS && n >= 22) {
      uint32_t acc = 0;
      int bits = 0, k = 0;
      for (int i = 0; i < 22 && k < 16; i++) {
        acc |= (uint32_t)p[i] << bits;
        bits += 8;
        while (bits >= 11 && k < 16) {
          ch[k++] = acc & 0x7FF;
          acc >>= 11;
          bits -= 11;
        }
      }
      newChannels = true;
    } else if (type == T_LINK_STATS && n >= 10) {
      link.rssi1 = p[0]; link.rssi2 = p[1]; link.lq = p[2]; link.snr = (int8_t)p[3];
      link.rfMode = p[5]; link.txPower = p[6];
    }
  }
};

// ---------------------------------------------------------------- telemetry frame builders
// Each returns the number of bytes written to out (sync .. crc).
inline int frame(uint8_t type, const uint8_t* payload, int n, uint8_t* out) {
  out[0] = ADDR_FC;
  out[1] = (uint8_t)(n + 2);
  out[2] = type;
  memcpy(&out[3], payload, n);
  out[3 + n] = crc8(&out[2], n + 1);
  return n + 4;
}

inline int battery(float volts, float amps, uint32_t mahUsed, uint8_t percent, uint8_t* out) {
  uint8_t p[8];
  uint16_t v = (uint16_t)(volts * 10.0f + 0.5f), a = (uint16_t)(amps * 10.0f + 0.5f);
  p[0] = v >> 8; p[1] = v & 0xFF; p[2] = a >> 8; p[3] = a & 0xFF;
  p[4] = (mahUsed >> 16) & 0xFF; p[5] = (mahUsed >> 8) & 0xFF; p[6] = mahUsed & 0xFF; p[7] = percent;
  return frame(T_BATTERY, p, 8, out);
}

inline int attitude(float pitchRad, float rollRad, float yawRad, uint8_t* out) {
  int16_t v[3] = {(int16_t)(pitchRad * 10000.0f), (int16_t)(rollRad * 10000.0f), (int16_t)(yawRad * 10000.0f)};
  uint8_t p[6];
  for (int i = 0; i < 3; i++) { p[2 * i] = (uint16_t)v[i] >> 8; p[2 * i + 1] = v[i] & 0xFF; }
  return frame(T_ATTITUDE, p, 6, out);
}

inline int flightMode(const char* s, uint8_t* out) {
  uint8_t p[16];
  int n = 0;
  while (s[n] && n < 14) { p[n] = (uint8_t)s[n]; n++; }
  p[n++] = 0;
  return frame(T_FLIGHT_MODE, p, n, out);
}

// Pack 16 channels into an RC-channels frame (used by the tests and the sim bridge).
inline int rcChannels(const uint16_t ch[16], uint8_t* out) {
  uint8_t p[22] = {};
  uint32_t acc = 0;
  int bits = 0, k = 0;
  for (int i = 0; i < 16; i++) {
    acc |= (uint32_t)(ch[i] & 0x7FF) << bits;
    bits += 11;
    while (bits >= 8) { p[k++] = acc & 0xFF; acc >>= 8; bits -= 8; }
  }
  return frame(T_RC_CHANNELS, p, 22, out);
}

}  // namespace crsf
