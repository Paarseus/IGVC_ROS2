# Firmware

| Folder | Status | What it is |
|---|---|---|
| `teensy_diff_drive_v2/` | **Live (v2d), FINAL — flashed + ground-verified 2026-10-08** | Teensy 4.1 USB-serial ↔ CAN bridge for 2 × REV SPARK MAX on firmware 26.1.5. Protocol: `PROTOCOL.md`. Reviews: `REVIEW.md`. Boot banner confirmed on the hardware: `# avros diff-drive bridge v2d ready (SPARK MAX FW 26.1.5)`. |
| `teensy_diff_drive/` | Legacy (v1) | The previous bridge (SPARK FW 25). Kept for rollback |
| `teensy_diag/`, `neopixel_test/`, `safety_light_test/` | Bench utilities | Diagnostics and safety-light tests |

**Build and flash (on the Jetson; stop actuator_node / the web UI first):**
```bash
export PATH=$HOME/bin:$PATH
arduino-cli compile --fqbn teensy:avr:teensy41 --output-dir ~/fw_v2d ~/IGVC_ROS2/firmware/teensy_diff_drive_v2
teensy_loader_cli --mcu=TEENSY41 -s -w ~/fw_v2d/teensy_diff_drive_v2.ino.hex
```
After boot the Teensy prints `CHK OK` or `CHK FAIL <reasons>`. Do not drive on `CHK FAIL`.
