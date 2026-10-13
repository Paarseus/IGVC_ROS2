# avros_webrtc

Low-latency teleop video from the ZED X to a browser, isolated from the control path.

```
ZED wrapper ─ raw image ─► webrtc_node ─ NVENC H.264 ─► MediaMTX ─ WebRTC ─► browser
```

- Encodes on the Jetson hardware encoder (NVENC), not the CPU.
- Subscribes to the raw image only (never `/compressed` or `/theora`).
- MediaMTX runs as-is: official v1.21.1 binary, stock `mediamtx.yml`.

## Setup (Jetson, once)

```bash
bash src/avros_webrtc/scripts/install_deps.sh
colcon build --symlink-install --packages-select avros_webrtc avros_bringup
```

## Run

```bash
ros2 launch avros_bringup sensors.launch.py enable_zed_front:=true
ros2 launch avros_bringup webrtc.launch.py
```

## View

Open in Chrome/Edge on any Tailscale machine:

```
http://100.93.121.3:8889/zed_front
```

## Config

`avros_bringup/config/webrtc_params.yaml`

| Param | Default |
|---|---|
| `image_topic` | `/zed_front/zed_node/rgb/color/rect/image` |
| `rtsp_url` | `rtsp://127.0.0.1:8554/zed_front` (last segment = URL path) |
| `frame_rate` | `8.0` (match `zed_front.yaml` `pub_frame_rate`) |
| `bitrate_bps` | `1500000` |
| `idr_interval` | `8` |
| `bitrate_scale_topic` | `/webrtc/bitrate_scale` (live bitrate = `bitrate_bps` × scale; `''` = fixed) |
| `pausable` | `false` (side cameras: `true`, see below) |
| `paused_interval_s` | `2.0` |

## Adaptive bitrate (cellular)

Fixed CBR has no congestion control: on cellular an overloaded uplink
bufferbloats, so video and drive commands arrive seconds late instead
of being dropped. `rate_controller` fixes that for all encoders at once:

```bash
ros2 run avros_webrtc rate_controller --ros-args \
  -p target:=<viewer tailscale ip> -p base_total_bps:=1200000 -p front_bps:=500000
```

- Pings the viewer 5×/s over Tailscale (same path as the video).
  Queue delay = median RTT (last 1 s) − min RTT (last 120 s).
- Delay > 200 ms or > 40% pings lost → scale × 0.7 (once per second);
  > 800 ms → floor. Delay < 80 ms for 2 s while video is actually
  flowing → +2 %/s. Range 0.25–1.5, start 0.6.
- Publishes `/webrtc/bitrate_scale` (Float32). NVENC (`nvv4l2h264enc`)
  takes the new bitrate live within ~1 s on JetPack 6, though
  gst-inspect says READY-only.
- Still clogged after 3 s at the floor → `/webrtc/sides_enabled` false:
  `pausable` encoders send one frame per `paused_interval_s` (keeps the
  WebRTC session up), the front gets the link. Resume after 15 s calm
  once the front alone carries 1.3× what all cameras need at the floor.
- NVENC overshoots small CBR targets (0.25 × 1.2 Mbps sent ~480 kbps),
  which is why pausing the sides works where lowering the floor doesn't.

Field results: SimChair_AVL `docs/RELAY_TELEOP_2026-10-10.md`.

Ports: 8889/tcp, 8189/udp (browser), 8554/tcp (local).

Testing: see [TESTING.md](TESTING.md).
