# Testing avros_webrtc

Run in order. Each step isolates one layer. Viewer = sim chair PC browser.

## 1. Encoder + MediaMTX (no ROS)

```bash
mediamtx /usr/local/etc/mediamtx.yml &
gst-launch-1.0 videotestsrc is-live=true pattern=ball \
  ! video/x-raw,format=BGRx,width=480,height=300,framerate=8/1 \
  ! nvvidconv ! 'video/x-raw(memory:NVMM),format=NV12' \
  ! nvv4l2h264enc profile=0 control-rate=1 bitrate=1500000 iframeinterval=8 idrinterval=8 \
    insert-sps-pps=true poc-type=2 preset-level=1 maxperf-enable=true \
  ! h264parse config-interval=-1 ! rtspclientsink location=rtsp://127.0.0.1:8554/test protocols=tcp
```

Pass: bouncing ball at `http://100.93.121.3:8889/test`.

## 2. Node with a fake image (no ZED)

```bash
ros2 run image_tools cam2image --ros-args -p burger_mode:=true -r image:=/test_image
ros2 run avros_webrtc webrtc_node --ros-args \
  -p image_topic:=/test_image -p rtsp_url:=rtsp://127.0.0.1:8554/test
```

Pass: burger at `/test`; log shows `Streaming ... bgr8`.

## 3. ZED X

```bash
ros2 launch avros_bringup sensors.launch.py enable_zed_front:=true
ros2 topic echo --once --field encoding /zed_front/zed_node/rgb/color/rect/image   # bgra8
ros2 launch avros_bringup webrtc.launch.py
```

Pass: camera at `http://100.93.121.3:8889/zed_front`.

## 4. Network

| Check | Command | Pass |
|---|---|---|
| Direct path | `tailscale ping 100.93.121.3` (sim chair) | `via <ip>:41641`, not DERP |
| Packet size | `sudo tcpdump -i tailscale0 udp port 8189 -c 50 -v` (Jetson) | length < 1280 |
| Bandwidth | `iperf3 -s` (Jetson), `iperf3 -c 100.93.121.3 -R -u -b 3M -t 20` (sim chair) | loss < 1% |

## 5. Performance

- `tegrastats` while streaming: NVENC active, CPU about the same as without the stream.
- Run with navigation: MPPI loop rate unchanged.
- Latency: point the ZED at a millisecond stopwatch on the sim chair screen, photograph screen + stream, subtract. Expect ~150–250 ms.

## 6. Recovery

| Action | Expected |
|---|---|
| Kill + restart MediaMTX | Node retries every 2 s, stream returns |
| Kill `webrtc_node` | ZED and perception unaffected |
| Reload browser / brief network drop | Video back within ~1 s |
