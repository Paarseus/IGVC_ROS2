# IGVC 2026 AutoNav — Surveyed GPS Waypoints

**Source:** Official IGVC 2026 slide — `gl-systems-technology.net/uploads/3/4/7/2/34727963/2026-igvc-gps-waypoints-2_orig.jpg`
**Decoded:** 2026-05-30. Emergency # on slide: 248-370-3331.

## ⚠️ Coordinate format — read this first

IGVC publishes the coordinates in **DMS packed `DD.MMSSssss`** (degrees, then 2-digit minutes, 2-digit seconds, then fractional seconds) — **NOT** decimal degrees and **NOT** degrees-decimal-minutes. They **must be decoded** before use:

```
decimal_degrees = DD + MM/60 + SS.ssss/3600
```

Example — `42.400510946` (5004 Practice Mid latitude):
`42° + 40'/60 + 05.10946"/3600 = 42 + 0.666667 + 0.001419 = 42.66808596°`

Using the raw value directly puts you ~30 km off; decimal-minutes puts you ~60 m off. The **decimal-degrees** column below is the correct, usable form (verified in the field: sent via `/fromLL` → robot reached the true marker).

## Waypoints

### Practice (3 points, ~14 m N↔S spacing)

| Code | Name | Raw (DMS `DD.MMSSssss`) | **Decimal degrees (USE THIS)** |
|------|------|--------------------------|-------------------------------|
| 5002 | Practice North | `42.400556255, -83.130645144` | **42.66821182, -83.21845873** |
| 5004 | Practice Mid    | `42.400510946, -83.130640432` | **42.66808596, -83.21844564** |
| 5003 | Practice South  | `42.400465621, -83.130635756` | **42.66796006, -83.21843266** |

### Main Course (4 points, ~75 m WEST of Practice)

| Code | Name | Raw (DMS `DD.MMSSssss`) | **Decimal degrees (USE THIS)** |
|------|------|--------------------------|-------------------------------|
| 5001 | Main North     | `42.400577100, -83.130962509` | **42.66826972, -83.21934030** |
| 5006 | Main North-Mid | `42.400523432, -83.130969819` | **42.66812064, -83.21936061** |
| 5005 | Main South-Mid | `42.400507588, -83.130969297` | **42.66807663, -83.21935916** |
| 5000 | Main South     | `42.400453974, -83.130957951` | **42.66792771, -83.21932764** |

## Ready-to-use (decimal degrees)

```yaml
# IGVC 2026 surveyed waypoints — decoded decimal degrees [latitude, longitude]
practice:
  north: [42.66821182, -83.21845873]   # 5002
  mid:   [42.66808596, -83.21844564]   # 5004
  south: [42.66796006, -83.21843266]   # 5003
main_course:
  north:     [42.66826972, -83.21934030]   # 5001
  north_mid: [42.66812064, -83.21936061]   # 5006
  south_mid: [42.66807663, -83.21935916]   # 5005
  south:     [42.66792771, -83.21932764]   # 5000
```

## How these are sent as Nav2 goals

With RTK FIXED + absolute GPS fusion (`global_frame: map`), convert lat/lon → map via the `/fromLL` service, then send a map-frame `NavigateToPose` goal:

```bash
# lat/lon -> map (x,y)
ros2 service call /fromLL robot_localization/srv/FromLL \
  "{ll_point: {latitude: 42.66808596, longitude: -83.21844564, altitude: 0.0}}"
# then send NavigateToPose with frame_id: map at the returned (x,y)
```

Field-verified 2026-05-30: robot reached Practice Mid / South / North within goal tolerance (0.5 m xy, position-only). See `memory/project_gps_waypoint_nav_works.md` for the full recipe.
