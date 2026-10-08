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

Ports: 8889/tcp, 8189/udp (browser), 8554/tcp (local).

Testing: see [TESTING.md](TESTING.md).
