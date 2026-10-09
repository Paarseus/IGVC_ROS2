# GPS Datum History

The `datum` field in `src/avros_bringup/config/navsat.yaml` anchors the map frame's (0, 0) to a real-world GPS coordinate. It must match the deployment location for the EKF map → odom transform to be accurate.

| Date | Datum (lat, lon, alt) | Location | Notes |
|---|---|---|---|
| original | `34.059270, -117.820934, 0.0` | CPP campus | original development site, committed in early bringup |
| 2026-05-13 | `42.401640686, -86.283409651, 0.0` | SW Michigan | sample mean of 707 GPS fixes; Jetson-local override, never committed to main |
| 2026-05-27 | `42.658430417, -83.241993772, 247.578` | SE Michigan (Detroit metro / Oakland County area) | sampled 2026-05-27 P0.17; mean of 200 GNSS fixes, ~1m spread (sub-meter — SBAS likely active despite driver reporting status=0); Jetson-local override |

## How to sample a new datum

1. Stop webui: `sudo systemctl stop avros-webui`
2. Launch sensors only: `ros2 launch avros_bringup sensors.launch.py enable_ntrip:=false`
3. Wait ~5 s for `/gnss` to publish
4. Sample 200+ fixes (50+ s at 4 Hz): `ros2 topic echo /gnss --field "[latitude, longitude, altitude]" > /tmp/gnss_fixes.txt`
5. Compute mean: `awk -F, '{lat+=$1; lon+=$2; alt+=$3; n++} END {printf "lat=%.9f lon=%.9f alt=%.3f n=%d\n", lat/n, lon/n, alt/n, n}' /tmp/gnss_fixes.txt`
6. Edit `src/avros_bringup/config/navsat.yaml` line containing `datum:` with the new tuple
7. Commit
