# IMU USB Latency Fix (2026-09-30)

## Summary
The Xsens IMU reaches the Jetson through an FTDI USB-serial chip (the "MTi USB Converter", USB ID 2639:0301, Linux driver `ftdi_sio`).

**Problem.** The chip's latency timer was at its default of 16 ms, so IMU messages arrived in bursts. The timer is now 1 ms, set permanently by a udev rule, and the IMU arrives evenly every 10 ms.

## Why it matters
An FTDI chip does not pass data to the computer as it arrives. It sends a USB packet only when one of these happens:
- its 64-byte buffer is full;
- a serial status line changes;
- a set event character arrives;
- the **latency timer** expires (default 16 ms).

Source: FTDI application note AN232B-04, `research/topics/L5_time_sync` (L5-S39).

Each Xsens message is shorter than the buffer. The tail of each message therefore waits in the chip for up to 16 ms, and several messages come out together. At 100 Hz this means:
- **Late data.** The EKF and the heading-hold loop in `actuator_node` see IMU data up to 16 ms older than necessary. The odometry budget in `GROUND_TEST_PLAN.md` is 50 ms end to end, so 16 ms is a third of it.
- **Uneven data.** Messages arrive in clumps with gaps up to 38 ms. A consumer that runs on a timer (the 30 Hz EKF, the 50 Hz heading-hold) sometimes gets no new IMU sample and sometimes gets three.
- **Arrival times are unusable as timestamps.** Any timing done from arrival time, such as the ground-test analysis and host-time logging, inherits the 0–16 ms error.

## Measurement
`/imu/data` at 100 Hz, 20 s stationary, on the Jetson with the full sensor launch running. Tool: `docs/drive_tuning_2026_09_28/tools/ground/imu_timing.py`.

| Latency timer | Mean interval | Spread (sd) | 1st / 99th percentile | Max gap | Messages < 2 ms apart |
|---|---|---|---|---|---|
| 16 ms (before) | 10.00 ms | **12.85 ms** | 0.87 / 36.67 ms | 37.96 ms | **65 %** |
| 1 ms (after) | 10.01 ms | **0.71 ms** | 8.54 / 11.84 ms | 19.45 ms | **0 %** |

The driver kept running through the change, with no errors in its log.

## Why the chip waits, and why the Xsens driver does not fix it

**Why the chip waits.**
- USB is polled by the host: the chip can only hand data over when the host asks, in packets of up to 64 bytes (2 status bytes + 62 data bytes).
- The serial link is a plain byte stream, so the chip cannot tell where one Xsens message ends.
- FTDI's rule is therefore: send when the 64-byte packet is full; otherwise wait for more bytes until the latency timer runs out (AN232B-04 §3.1, local copy `research/topics/L5_time_sync/sources/ftdi_2006_an232b04_latency.pdf`).
- FTDI chose 16 ms "so that we could make advantage of 64 byte packets to fill large buffers" (§3.1). That is a throughput choice for bulk data (modems, printers), not for 100 Hz sensor messages.
- With our IMU at 921 600 baud, the first 62 bytes of a message leave at once. The rest waits. Usually the next message arrives 10 ms later and fills the packet first, so the tail of one message and the head of the next arrive together. That is the 65 % "< 2 ms apart" we measured.

**Where the setting lives.**
- The timer is a register in the FTDI chip. The Linux driver `ftdi_sio` writes it when the device is plugged in, from `priv->latency` (default 16).
- There are two ways to change it:
  - the sysfs file `latency_timer`, which is what the udev rule writes;
  - the `ASYNC_LOW_LATENCY` serial flag (`setserial low_latency`, or the `TIOCSSERIAL` ioctl). With that flag set, `write_latency_timer()` uses 1 ms (`drivers/usb/serial/ftdi_sio.c`, Linux 5.15, the Jetson's kernel 5.15.148-tegra).
- Either way the value is lost when the device is re-plugged or the Jetson reboots, unless something sets it again.

**What the Xsens driver does.**
- The Xsens ROS 2 driver opens the port with plain `termios` settings (`lib/xspublic/xscontroller/serialinterface.cpp:409-465`, `VMIN 0`, `VTIME` timeout) and polls it with `select()`.
- It never sets `ASYNC_LOW_LATENCY` or the timer: a search of its source finds no `LOW_LATENCY`, `latency_timer` or `TIOCSSERIAL`.
- Xsens' own guidance for Linux is to enable low-latency mode at the system level with `setserial <port> low_latency` (Xsens / element14 "Interfacing MTi devices with the NVIDIA Jetson").

**Options compared.**

| Fix | How | Survives reboot / replug | Needs a code change | Verdict |
|---|---|---|---|---|
| **udev rule** (chosen) | sets `latency_timer=1` whenever the device appears | yes | no | the common Linux practice (Granite Devices, rosserial, ROBOTIS); nothing to maintain |
| Patch the Xsens driver | set `ASYNC_LOW_LATENCY` with `TIOCSSERIAL` after `open()` | yes, while the driver starts it | yes: a patch on a vendor package that is re-cloned by `vcs import` (like the kiwicampus patch) | works; only worth it as a second safety net |
| `setserial low_latency` at boot | a systemd unit or script | only if the script runs after every replug | no | weaker than udev (misses replugs) |
| FTDI event character | chip flushes when a chosen byte arrives | — | yes (libftdi, not `ftdi_sio`) | not usable: Xsens messages start with a known byte (0xFA) but do not end with one |
| Different USB-serial adapter | a converter without a 16 ms timer | — | hardware | not needed |

The udev rule stays. A driver patch is only worth adding if the IMU is ever moved to a machine without the rule. In that case, check `latency_timer` in the session preflight (plan test 0.6).

## Permanent fix
The rule is in the repo at `scripts/udev/99-avros-xsens-latency.rules`:

```
ACTION=="add|change", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTRS{idVendor}=="2639", ATTR{latency_timer}="1"
```

- It matches only the Xsens converter (vendor 2639).
- It runs at boot, on every replug, and after a `udevadm trigger`.

**Install** (done on the Jetson 2026-09-30):

```bash
sudo cp ~/IGVC_ROS2/scripts/udev/99-avros-xsens-latency.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change --subsystem-match=usb-serial
cat /sys/bus/usb-serial/devices/ttyUSB*/latency_timer      # expect 1
```

**Check after a reboot or replug:**
```bash
cat /sys/bus/usb-serial/devices/ttyUSB*/latency_timer      # 1
python3 ~/IGVC_ROS2/docs/drive_tuning_2026_09_28/tools/ground/imu_timing.py 20
# sensors running; expect sd < 2 ms, 0 % < 2 ms apart
```

**Undo:**
```bash
sudo rm /etc/udev/rules.d/99-avros-xsens-latency.rules
sudo udevadm control --reload-rules
echo 16 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

**Cost.** At 1 ms the chip sends more, smaller USB packets. At 100 Hz of IMU data this is negligible on the Jetson.

## Related finding (not fixed): IMU and GPS timestamps before a GPS fix
While measuring, `/imu/data` and `/gnss` had header stamps of about **644 s**: time since the Xsens powered on, not the real clock (about 1.79 × 10⁹ s).

**Cause.** `xsens.yaml` does not set `time_option`, so it defaults to 0 ("UTC from the MTi"). The driver copies the MTi's UTC field without checking that it is valid (`xsens_time_handler.cpp:45-68`). Until the GNSS receiver has set the MTi clock, that field counts up from 1970.

**Effect:**
- Until GPS time is valid, IMU and GPS messages are about 56 years older than every other message in ROS.
- robot_localization is likely to reject them as too old.
- When GPS time becomes valid, the stamps **jump** by about 1.79 × 10⁹ s.
- The Sept 2026 audit measured a normal IMU age (median 25.8 ms), which fits: that was taken with a GPS fix.

**Options** (a decision for later; not changed):

| `time_option` | Stamp | Pro | Con |
|---|---|---|---|
| 0 (now) | MTi UTC | exact sample time once GPS time is valid | wrong until GPS time is valid, then jumps |
| 1 | MTi sample counter, anchored to the Jetson clock at the first message | even spacing, valid from start, no jump | drifts slowly against the Jetson clock (ppm) |
| 2 | Jetson clock at arrival | always consistent with other nodes | includes USB and processing delay (now small, with the 1 ms timer) |

**Check before navigation:** compare `ros2 topic echo --once /imu/data` header stamp with `date +%s`. They must agree to within about 0.1 s.
