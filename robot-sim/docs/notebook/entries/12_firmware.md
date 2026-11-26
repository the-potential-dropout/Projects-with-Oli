# 12 · Firmware and protocol
*2026-11-25*

- **Board.** Waveshare Servo Driver with ESP32. Servo bus on GPIO18/19 at 1 Mbaud (pins confirmed from Waveshare's example source).
- **Library.** Feetech SCServo (`SMS_STS`).
- **IDs.** Leg L, joint J → ID 1 + 3L + J. Attachment servos are IDs 21–25.

**Protocol unchanged.** The sim still sends `S,<deg>…` at 25 Hz. The firmware converts 0–180° to 4096-step positions (2048 = centre) and sends one `SyncWritePosEx` per frame.

**New telemetry.** `B,<mV>,<mA>,<°C>,<state>` at 2 Hz.

**Safety logic.**
- 0.5 s link timeout → all servos relax.
- Soft-start speed for 2 s after a relink.
- Low-voltage cut-off at 10.2 V (3 s debounce), recovering above 11.0 V.
