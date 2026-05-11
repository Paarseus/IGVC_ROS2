# avros_control — Review

## Summary

`avros_control` is a single-file ROS 2 Python node (`actuator_node.py`, 426 LOC) that bridges
`/cmd_vel` and `/avros/actuator_command` to a Teensy 4.1 over `/dev/ttyACM0` at 115200 baud, with
diff-drive inverse kinematics, asymmetric slew-rate limiting, IMU heading-hold, gyro-stabilized
turning, and a `/wheel_odom` integrator for EKF fusion. The package layout is professional
(format-3 manifest, `setup.cfg`, resource marker, LICENSE present, three boilerplate linters in
`test/`) and the control-side architecture is conceptually correct: dt-aware slew limits, separate
accel/decel caps, midpoint pose integration, latest-wins command priority with a 500 ms watchdog,
and a serial-write lock for thread safety.

**The node has multiple P0 defects that should be fixed before any field driving.** The
`destroy_node()` shutdown sends one `S` line then closes the serial port, but the SIGINT path goes
through `KeyboardInterrupt` → `node.destroy_node()` → `rclpy.try_shutdown()`, and during a noisy
`KeyboardInterrupt` (e.g. operator presses Ctrl-C twice, or rclpy logger raises during shutdown)
the `S` may never reach the Teensy. There is **no `try/except` around `serial.Serial(...)` in
`__init__`**: if `/dev/ttyACM0` is missing on launch the node raises `SerialException` and dies,
which is acceptable, but if the cable is unplugged at runtime the `_serial_reader` thread hits
`SerialException` in `read()` and **silently re-spins forever logging at full rate while the
control loop continues happily writing into a dead handle**. There is no reconnect path; there is
no "serial dead → publish stop on rosout / set local e-stop flag" failsafe. `_estop` is a Python
bool that can only be **set** by an `ActuatorCommand` (no auto-clear), which means the only way to
clear an e-stop in software is to publish a non-estop `ActuatorCommand` — there is no concept of a
latched fault that requires explicit re-arm, but also no concept of "watchdog tripped, motors
already cleared, refuse new motion until operator confirms." Slew on e-stop is set to **zero
instantly (line 258-259)**, which is *infinite* deceleration applied to the SparkMAX setpoint —
this is the opposite of the asymmetric-decel-cap discipline described in the comment immediately
above and in the standards doc; it relies entirely on the SparkMAX's own brake-mode behavior to
absorb the kinetic energy and is jerk-unbounded. Finally, the IMU-fresh flag (`_imu_fresh`) is
**set true on the first IMU message and never reset**: if the Xsens dies mid-run, the node keeps
applying heading-hold against a stale yaw forever.

**Tests are boilerplate-only.** `test_copyright.py` is `pytest.mark.skip`, `test_flake8.py` and
`test_pep257.py` run but cover style, not behavior. There is **no unit test** for `wrap_angle`,
`yaw_from_quaternion`, the diff-drive inverse, the slew limiter, the heading-hold gate, or the
midpoint-pose integrator. For a 426-line safety-critical control node one month from competition
this is a P1.

Severity buckets below; full table in *Safety analysis*. Recommended P0 fixes (none of them
require firmware changes) can be done in a single afternoon and would meaningfully reduce the
risk of the node misbehaving on the field.

## Per-file findings

### actuator_node.py — safety / failure modes

**`_estop` flag has no auto-clear and no latch (lines 131, 211-217, 244-245).** `_on_actuator_cmd`
sets `self._estop = True` when `msg.estop` is true, and sets `self._estop = False` whenever any
non-estop `ActuatorCommand` arrives (line 217). Two consequences:

1. Any non-estop `ActuatorCommand` (e.g. webui phone joystick re-connecting after a momentary
   disconnect) **silently clears the e-stop** without operator confirmation. There is no concept
   of "latched fault — explicit re-arm required."
2. If the e-stop source is `cmd_vel` only (Nav2 BT firing a stop pose), there is no path to set
   `_estop` at all — only `ActuatorCommand` can. Nav2 has no idea this flag exists.

For a competition vehicle the safer design is: any time `_estop` is set, latch it; require either
a service call (`/avros/clear_estop`) or a specific `ActuatorCommand` flag (`clear_estop=True`) to
clear it; never auto-clear on a fresh non-estop command. **P0.**

**`_estop = True` produces zero-jerk slew (lines 256-259).** Comment claims "Emergency path:
allow fastest decel to zero regardless of caps" but the implementation is `self._slew_v = 0.0;
self._slew_w = 0.0` — i.e. the *commanded RPM* steps to zero in one tick. The SparkMAX's own PID
will then compute whatever current it needs to track that zero setpoint, capped only by the
SparkMAX hardware current limit and the brake-idle behavior. This is fine if the chassis is
moving slowly, but at 1.5 m/s top speed the resulting decel can saturate the PID, regen-spike the
12 V rail (CLAUDE.md notes brown-out reboots have already happened), and snap drivetrain backlash.
The standards doc (§6) is explicit that asymmetric *decel* should still respect a cap. Recommended
fix: on e-stop, set `v_req = 0` and let the slew limiter use `_decel_v` (or a separate
`_estop_decel_v` parameter, e.g. 3.0 m/s² for an emergency profile). **P0.**

**No serial reconnect path (lines 376-398).** `_serial_reader` runs in a daemon thread; the
`except Exception` swallows `SerialException` (cable unplug, USB device gone, Teensy reset) and
calls `time.sleep(0.1)` then loops. There is no attempt to `self._serial.close(); self._serial =
serial.Serial(self._port, self._baud, timeout=0.1)` — the dead handle stays dead, and the control
loop in the foreground keeps calling `_serial_write` which catches the exception, logs at full
50 Hz rate, and returns. **Net effect: the chassis can be moving with the operator believing the
node is "fine" because there is no top-level "serial dead" indicator on `/avros/actuator_state`.**
Recommended: detect the dead handle, set a `_serial_dead` flag, force `_estop = True`, log a
single ERROR (not a stream), and attempt re-open every 1 s with backoff. The state publisher
should reflect this so webui/Nav2 knows. **P0.**

**No serial-write rate limiting on errors (lines 369-374).** If the serial cable is unplugged, the
control loop fires at 50 Hz and each call hits the `except Exception` branch, logging at 50 Hz to
`/rosout`. This will spam the logs and (more critically) flood DDS with logger output during the
exact moment you want the system observable. Use `get_logger().error(..., throttle_duration_sec=2.0)`.
**P1.**

**Shutdown ordering (lines 401-410).** `destroy_node` sets `_running = False` and writes one `S`,
sleeps 100 ms, then closes the serial port. There is a race: the control-loop timer can fire
between the `S` write and the `serial.close()`, writing a fresh `L<rpm> R<rpm>` setpoint *after*
the stop. This is rare but possible because rclpy timers are not cancelled by `_running` (it's a
flag the reader thread reads, not the timer). Recommended: cancel the control timer first
(`self._control_timer.cancel()` — but the code doesn't store the timer handle), then write S,
then close. **P1.**

**`KeyboardInterrupt` only — no SIGTERM handling (lines 413-422).** `main()` catches
`KeyboardInterrupt` (which `rclpy.spin` translates from SIGINT). On SIGTERM (e.g. systemd stop,
launch file kill) there is no handler — `rclpy.spin` returns, `try/except` falls through to the
`finally`, `destroy_node` runs, but the order isn't deterministic. For a competition vehicle this
is acceptable; for a node controlling a moving chassis it would be more robust to install a signal
handler that explicitly sends `S` and joins the reader thread before `rclpy.shutdown()`. **P2.**

**No bounded-time response check from Teensy (lines 369-398).** The host writes commands fire-and-
forget; the only feedback is the `E` line from the reader thread. There is **no check that an `E`
line has been seen recently** — if the Teensy reset, hung, or lost USB enumeration, the control
loop keeps writing setpoints into a dead pipe and the publisher keeps reporting `/avros/actuator_state`
with **stale** measured RPM (whatever was last received). The standards doc (§3) calls this out:
"Silently dropping a command is the worst failure mode — the host can't distinguish 'command
accepted' from 'command lost in noise' from 'firmware crashed'." Recommended: timestamp the last
`E` line; if `> 200 ms` without one, set `_serial_dead = True` and the same auto-stop path. **P0.**

**No IMU staleness check (lines 137-141, 225-229, 274-289).** `_imu_fresh = True` is set on the
first IMU message and is **never reset**. If the Xsens hardware fails or the driver crashes
mid-run, `_current_yaw` stays at whatever value it last had, and `_heading_locked = True` keeps
applying `heading_kp * (heading_target - stale_yaw)` corrections to ω indefinitely. On a long
straight this can manifest as a persistent ω bias the operator can't explain. Recommended:
timestamp the last IMU message; treat IMU as stale after `> 200 ms` (Xsens runs at 100 Hz so this
is 20 missed samples, comfortably tolerant); if stale, disable heading-hold and gyro-rate
correction, log throttled warning. **P0.**

**No bus-voltage monitoring (none).** CLAUDE.md flags brown-outs as a known cause of "Jetson
crashes randomly during motor testing." The Teensy DIAG line (per `firmware/teensy_diff_drive`)
includes bus voltage, but `actuator_node.py` only parses `E` lines (line 379) — there is no path
to ingest bus voltage and publish it on `/avros/actuator_state` or trigger a controlled
deceleration when it drops below e.g. 11 V. **P1.**

**`_target_v` / `_target_w` are not protected by a lock (lines 124-125, 204-207, 209-223,
247-248).** Three callbacks (`_on_cmd_vel`, `_on_actuator_cmd`, `_control_loop`) read/write these
fields without synchronization. With the default `SingleThreadedExecutor` and rclpy timers, this
is *almost* safe because callbacks serialize on the executor — but `_serial_reader` is a *separate
Python thread* and could (if mis-used in a future change) write `self._target_v` and corrupt a
read-modify-write. Pure floats are likely atomic on CPython under GIL, but the whole pattern is
fragile. Recommended: add a single `self._cmd_lock` and use it in all three places, or document
the GIL-ish invariant explicitly. **P2** (currently safe under SingleThreadedExecutor).

### actuator_node.py — control loop discipline

**Fixed-rate timer used correctly (line 198).** `self.create_timer(self._ctrl_dt, self._control_loop)`
runs at 50 Hz, matches `cmd_timeout_s = 0.5` (25× margin), matches the firmware's universal-
heartbeat 50 Hz cadence. Good.

**`dt` for slew-rate is taken from the parameter, not measured (lines 197, 264-269).** `self._ctrl_dt
= 1.0 / control_rate` is computed once at init and used as `dt` in the slew limiter. If the
SingleThreadedExecutor is busy and a tick is delayed by, say, 30 ms, the limiter still uses
`dt = 0.02` and under-applies acceleration that frame. Conversely, if a tick fires twice quickly
(rare but possible under timer drift), it applies *less* slew per real-time-second than intended.
The professional pattern is to record `last_tick = self.get_clock().now()` and compute `dt = (now -
last_tick).nanoseconds / 1e9`, clamped to a sensible range to handle pause/resume. **P1.**

**Asymmetric accel/decel logic is *almost* right but has an edge case (lines 263-266).** Code:

```python
if abs(v_req) > abs(self._slew_v) and v_req * self._slew_v >= 0:
    max_dv = self._accel_v * self._ctrl_dt
else:
    max_dv = self._decel_v * self._ctrl_dt
```

This says "if the magnitude is increasing **and** signs agree, use accel cap; else use decel
cap." The intent (from the comment at lines 252-255) is correct — accelerating away from zero =
accel cap, decelerating toward zero or reversing direction = decel cap. **But: when `v_req = 0`
and `_slew_v = 0.5` (decelerating to a stop), `abs(0) > abs(0.5)` is false → decel cap applied.
Correct. When `v_req = -0.5` and `_slew_v = 0.5` (forward → reverse), `0.5 > 0.5` is false (not
strictly greater) and `-0.5 * 0.5 < 0` so the second clause kicks in → decel cap. Correct.** When
`v_req = 0.6` and `_slew_v = 0.5` (continuing to accelerate), `abs(0.6) > abs(0.5)` and signs
agree → accel cap. Correct. The logic is fine, but the *first* test (`abs(v_req) > abs(self._slew_v)`)
is **strict greater-than**, so when `v_req == _slew_v` (already at target) the else branch picks
decel cap; this doesn't matter because `dv = 0`. Net: logic is correct, comment is accurate, but
it took me three reads to convince myself. Add unit tests; you will not remember this in three
months. **P1 (test gap).**

**No saturation flag exposed (lines 268-269).** `clamp(dv, -max_dv, max_dv)` silently truncates;
when the controller asks for more accel than the chassis can deliver, the slew limiter quietly
truncates and the operator has no signal that they're saturated. Common pattern: publish
`v_request - v_slew` as `/avros/diagnostics/slew_lag` so it's visible on Foxglove. **P2.**

**Angular slew has no separate decel cap (line 267).** `max_dw = self._accel_w * self._ctrl_dt`
uses the same cap symmetrically. Per § 6 of the standards doc, robotics convention treats braking
as asymmetric. For ω this matters less (rotational inertia is much smaller than translational on
a 50 kg chassis) but consistency would be nice. **P2.**

**No blocking I/O in callbacks** — the callbacks (`_on_cmd_vel`, `_on_actuator_cmd`, `_on_imu`)
only set fields. Good.

**Serial write blocks while holding the lock (lines 370-372).** `serial.Serial.write()` blocks
until the bytes are queued in the OS buffer. At 115200 baud and ~20-byte commands, this is
~1.7 ms per write best case. The control loop holds `_serial_lock` during the write; if the OS
buffer is full (rare on USB-CDC), `write()` can block on the kernel side. Under SingleThreadedExecutor,
*the entire executor blocks* — IMU callbacks queue up, `_on_cmd_vel` queue up, etc. This is
unlikely to matter at 50 Hz × 20 bytes (= 1000 B/s, 1% of bandwidth) but if a future revision
streams DIAG telemetry it could. The standards doc calls this out as anti-pattern §6 ("blocking
I/O in callback"). Mitigation: a small write queue + dedicated writer thread. **P2** (acceptable
at current rates).

**Two timers, one executor (lines 198-199).** `_control_loop` at 50 Hz and `_publish_state` at
20 Hz both run on the SingleThreadedExecutor. They serialize, which means `_publish_state` can
delay `_control_loop` by up to ~1 ms (it's mostly arithmetic + one publish). Acceptable, but
worth noting if the encoder-integration or Odometry creation gets heavier. **P2** (note only).

**`time.sleep(0.3)` in `__init__` after opening the serial port (line 108).** This is in
`__init__`, not in a callback, so it doesn't block the executor — it blocks before the executor
even starts. Acceptable per the standards doc (§ 1: "delay() is acceptable in setup()..."). The
0.3 s gives the Teensy time to come out of bootloader. Good.

**`time.sleep(0.2)` in PID-push loop (line 117).** Also in `__init__`, four iterations × 0.2 s =
0.8 s of node-startup latency. Acceptable for one-time init; would be a smell in steady state.

### actuator_node.py — IMU heading-hold

**`yaw_from_quaternion` is correct (lines 39-43).** Standard ZYX-order yaw extraction; safe
against gimbal lock at +/- 90° pitch (uses `atan2`).

**`wrap_angle` is correct (lines 46-48).** `atan2(sin(a), cos(a))` is the textbook idiom for
wrapping to `[-π, π]`.

**Heading-hold gating is reasonable (lines 274-289).**

```python
if self._imu_fresh:
    if abs(w) < self._hh_deadband and abs(v) > 0.02:
        # Straight-line intent → heading hold
        ...
    else:
        # Turning → release hold, optionally close loop on ω via IMU
        ...
```

The double condition (`|w| < 0.05 rad/s` AND `|v| > 0.02 m/s`) avoids engaging hold while
stationary, which would otherwise lock the heading at whatever yaw was current when the operator
released the joystick. Good.

**Heading-target captured on lock entry (lines 278-280).** Correct: `if not self._heading_locked:
self._heading_target = self._current_yaw`. The hold engages on the *current* yaw the moment the
operator releases the steering input. **However:** if the chassis has any residual ω at that
moment (operator was turning slightly, then snapped to dead center), the heading will be captured
mid-rotation and the hold will then fight to return to that snapshot — this can feel like the
chassis "snaps" to a pose when steering centers. Consider waiting one tick after `|w| < deadband`
before locking, or low-passing the captured yaw. **P2.**

**Hold correction is **proportional only** (line 282).** No integral, no derivative — pure P. For
a track-drive chassis this is OK at low speeds, but on slope or in soft terrain the chassis will
exhibit a steady-state error proportional to the disturbance / `heading_kp`. With `heading_kp =
1.5` rad/s per rad of error, a 5° (0.087 rad) bias produces 0.13 rad/s of correction; this is
within the `_max_w * 0.5 = 0.5 rad/s` cap. Good. Adding a small integrator (e.g. `kI = 0.1` with
windup limit) would tighten straightness but is not urgent. **P2.**

**Yaw-rate correction (line 287-289).** The else-branch when `|w| ≥ deadband`:

```python
w_err = w - self._current_yaw_rate
w = w + self._yaw_rate_kp * w_err
```

This is a **positive feedback** form: it *adds* `kp * (commanded - measured)` to the commanded
ω. With `yaw_rate_kp = 0.3` and assuming the IMU yaw rate matches commanded ω (steady state),
`w_err → 0` and the correction is zero. If the chassis under-rotates (`measured < commanded`),
`w_err > 0` and the loop adds positive correction → commanded ω grows → chassis turns harder.
Correct *direction*, but it's a feed-forward gain on top of the open-loop command, **not a closed
loop on ω.** A true closed loop would replace `w` with `kp * w_err`, not add it. The current
form is a "boost when under-rotating" heuristic; it works at small `kp` (0.3) because it can't
diverge. At `kp = 1.0` this would turn into an instability — the comment "closes the loop on ω"
overstates what it does. **P1 — clarify comment, ideally rewrite to a true closed loop with
explicit anti-windup.**

**No max-correction clamp on yaw-rate branch (line 289).** The straight-line branch caps
correction at `0.5 * max_w` (line 284). The turning branch does not — `w + 0.3 * (w - yaw_rate)`
has no clamp before going into diff-drive inverse. With `_max_w = 1.0 rad/s` commanded and a
yaw-rate measurement of 0 (e.g. tire slip), the corrected `w` becomes `1.0 + 0.3 * 1.0 = 1.3 rad/s`,
which then bypasses the `_max_w` cap because that cap was applied earlier in `_on_cmd_vel`. The
diff-drive inverse will compute wheel speeds for 1.3 rad/s and the SparkMAXes will accept them.
**Recommended: clamp the corrected ω to `_max_w` before going into diff-drive inverse.** **P1.**

**No fallback when `_imu_fresh` is false.** Lines 274-289 are wrapped in `if self._imu_fresh:`,
but as noted in the safety section, this flag is set once and never reset. If the IMU dies, the
flag stays true forever and the node uses stale yaw. There is no "soft fallback to open-loop" path
for a real IMU outage. **P0** (covered above).

**Quaternion validation absent (line 226).** `yaw_from_quaternion(msg.orientation)` is called
without checking whether the quaternion is normalized or contains NaN. A faulted IMU driver that
publishes `(0, 0, 0, 0)` quaternions will produce `atan2(0, 1) = 0` (siny_cosp = 0, cosy_cosp = 1,
because the formula uses `1 - 2*(y² + z²)`, and at all zeros that's 1). So the failure mode is
"yaw stuck at 0" rather than "NaN propagates," which is *less* bad but still wrong. **P2** —
ideally check `|q|` is in [0.5, 1.5] before trusting it.

### actuator_node.py — serial protocol

**Open with timeout (line 107).** `serial.Serial(self._port, self._baud, timeout=0.1)` — read
timeout 100 ms is reasonable for a 256-byte read at 115200 baud. Good.

**No try/except around serial open (lines 107-109).** If `/dev/ttyACM0` is missing on launch (USB
not yet enumerated, Teensy not powered, wrong port), the node crashes with `SerialException`
during `__init__`. For a launch-file environment that's actually OK — the launch system reports
the failure and the rest of the stack doesn't come up. But it would be more user-friendly to
catch, log a clear error ("expected Teensy on /dev/ttyACM0; ls /dev/tty* says... — is the cable
plugged in?"), and `sys.exit(1)` cleanly. **P2.**

**`_serial_lock` only (line 110, 371).** The lock is held during `write()`, not during `read()`.
The reader thread in `_serial_reader` calls `self._serial.read(256)` without acquiring the lock
(line 383). PySerial's `Serial` object is **not documented as thread-safe across read/write
concurrently** — most platforms it works because read and write are independent kernel buffers,
but the `Serial` instance itself can have shared state (notably `read_buffer` / `_in_waiting` on
some backends). Recommended: keep the current pattern (read+write are usually safe in pyserial)
but document the assumption with a comment. **P2.**

**Reader uses `read(256)` blocking with timeout (line 383).** Returns up to 256 bytes within
100 ms. Buffer accumulation in `buf` (line 386) is correct — newlines are split and consumed
incrementally. Good.

**`E_RE` regex is compiled inside the thread function (line 379).** `import re` and the regex
compile happen inside the function body, executed once when the thread enters. Acceptable since
this is one-time, but stylistically `re.compile` is module-level. **P2 (style only).**

**Regex is fragile (line 379).** `r"E L(-?\d+) (-?[\d.]+) R(-?\d+) (-?[\d.]+)"` matches
`E L<rpm> <pos> R<rpm> <pos>`. If the Teensy ever changes its DIAG format (e.g. adds bus voltage,
swaps order, prints scientific notation), the regex silently stops matching and **`E` lines are
silently dropped** — the operator sees `_l_meas_rpm` frozen at zero with no log. **P1 —
recommend logging on parse failure (throttled), and parsing key=value pairs instead of positional
fields.**

**No PID-write retry / verification (lines 113-121).** The node writes `KF<val>`, `KP<val>`,
`KI<val>`, `KD<val>` to the Teensy on startup, with 0.2 s spacing, and **never verifies the
firmware accepted them.** If the Teensy isn't out of bootloader yet, or the lines are split
across a USB packet boundary, or the firmware silently rejects the value (out of range), the
gains stay at the firmware-default (or last-burned) value and the chassis behaves with whatever
is in flash. Per the standards doc § 3 ("Bidirectional handshaking"), every host command should
produce *some* firmware response within bounded time. Recommended: after each `KF/KP/KI/KD`,
parse the next D/DIAG line and assert the new value was applied. **P1.**

**No `BURN` command issued, no SparkMAX flash write at startup.** The PID gains are pushed in RAM
each boot (lines 114-117). This is correct per the standards doc § 9 ("don't `BURN` on every
command — only after characterization"). Good. The fact that the gains live in `actuator_params.yaml`
and get re-pushed every boot means the Teensy doesn't need to remember them — also good for
single-source-of-truth.

**No DIAG ingestion (lines 376-398).** The reader only matches `E` lines. The Teensy emits `D`
lines per CLAUDE.md's firmware section (tx/rx counts, watchdog state, mode, bus voltage). None of
these reach ROS. This means: **bus voltage is invisible**, **firmware's own watchdog state is
invisible**, **firmware's own command-loss timeout state is invisible**. For a competition vehicle
those should all be on `/diagnostics` or at minimum `/avros/actuator_state`. **P1.**

**Serial-write retry has no rate cap (lines 369-374).** Already covered in the safety section
(P1). One more note: `except Exception` is too broad — it catches `KeyboardInterrupt` too on some
Python builds (in pre-3.10 / certain interpreters), masking shutdown signals. Use `except
(serial.SerialException, OSError) as e:`. **P2.**

**`fb_lock` correctly used for measured RPM (lines 149, 306-308, 391-395).** Reader thread holds
the lock during the four assignments, publisher holds the lock during the two reads. Good.

**Floats `_l_pos_prev` / `_r_pos_prev` initialized to None but never used (lines 147-148).** Dead
code — the position deltas could be useful for an alternative odom integrator, but currently
position is computed from RPM × dt, not from delta-position. Either delete or wire up. **P2.**

### actuator_node.py — rclpy idioms

**Parameter declaration is professional (lines 58-84).** Every parameter is declared with a
default before being read. No `allow_undeclared_parameters`. Reads use the verbose form `p('name').value`.
Good.

**No `add_on_set_parameters_callback` (none).** Parameters can be changed at runtime
(`ros2 param set ...`), but the node will not see the change — the values are read once into
`self._max_v` etc. in `__init__` and never refreshed. Anyone changing `max_linear_mps` at runtime
expecting it to apply will be surprised. Either: (a) document that parameters are launch-time only
and recommend the user restart the node, or (b) add a SetParametersCallback that revalidates
ranges and rebinds the locals. The standards doc (§ 2) calls this out. **P1.**

**No QoS specified on subscribers (lines 158-163).** All three subscribers use the default
`qos_profile_default` (reliable, volatile, depth 10). For `/imu/data` from the Xsens at 100 Hz,
this is the **wrong choice** — the standards doc § 3 says explicitly: "Default QoS (reliable,
depth 10) on a 100 Hz IMU stream — the reliability buffer fills, publisher blocks, latency spikes.
Use `qos_profile_sensor_data`." For `/cmd_vel` from Nav2, default QoS is acceptable (Nav2 publishes
reliable). For `/avros/actuator_command` from webui, it depends on how webui publishes — but if
webui ever drops to best_effort, the actuator node will silently miss commands. **P1 — at minimum
switch `/imu/data` to `qos_profile_sensor_data`.**

**No QoS specified on publishers (lines 166-169).** `qos_profile_default` (reliable, depth 10).
For `/avros/actuator_state` at 20 Hz this is fine. For `/wheel_odom` at 20 Hz consumed by EKF,
also fine. **No issue.**

**Subscriber depth 20 for IMU (line 163).** `create_subscription(Imu, '/imu/data', ..., 20)` —
the depth is set to 20 explicitly, which is more than the default-10. With reliable QoS at 100 Hz
this means a one-second window of buffered messages, ridiculous for a control input. With
`sensor_data` QoS this would matter less. **P2 (cleaned up by switching to sensor_data).**

**No callback group specified (none).** Default is a single MutuallyExclusiveCallbackGroup. With
`SingleThreadedExecutor` (the default in `rclpy.spin`), all callbacks serialize. This is
**correct** for this node — no concurrency footguns. Good.

**Logger usage is clean (lines 111, 118-121, 201, 215, 374, 397, 402).** Uses
`self.get_logger().info/warn/error/...` consistently. No `print()`. No throttling on the
high-frequency error paths (line 374 in the per-tick serial-write error log) — already noted as
P1.

**Subscriber handles not stored (lines 158-163).** `self.create_subscription(...)` returns the
subscription object; this code does **not** assign it to `self.<something>`, so the handle is held
only by the rclpy node internals. Per the standards doc § 6: "Constructing a node-internal
subscriber without storing it on `self` — `self.create_subscription(...)` returns the subscription
handle; assigning it to a local variable lets it be garbage-collected and silently die. Always
`self.sub = self.create_subscription(...)`."

**Empirically, rclpy's Node retains the subscription internally (in `Node._subscriptions`), so it
does not get GC'd**, and the standards doc's anti-pattern is actually a "nice to have" rather than
a hard bug. The same applies to timers (lines 198-199). The current code works. But for
consistency with upstream demos and to make introspection easier, save the handles. **P2 (style).**

**`destroy_node` overridden correctly with `super().destroy_node()` (line 410).** Good.

**`rclpy.try_shutdown()` instead of `rclpy.shutdown()` (line 422).** `try_shutdown()` is the safer
form — it doesn't raise if rclpy is already shut down. Good.

**`rclpy.init(args=args)` plumbed correctly (line 414).** Good — picks up `--ros-args`.

**No exception handling around node construction (lines 414-415).** If `ActuatorNode.__init__`
raises (e.g. serial open fails), `node` is undefined when the `finally` block tries to call
`node.destroy_node()`. **NameError shadowing the original exception.** Fix:

```python
node = None
try:
    node = ActuatorNode()
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    if node is not None:
        node.destroy_node()
    rclpy.try_shutdown()
```

**P2.**

**No executor instantiation — uses `rclpy.spin(node)` directly (line 417).** This is the standard
single-threaded path. Good for this node.

### actuator_node.py — odometry math

**Midpoint integration is correct (lines 333-352).** The comment claims trapezoidal/midpoint
integration; the implementation does:

```python
yaw_delta = w * dt
yaw_avg = wrap_angle(self._odom_yaw + yaw_delta / 2.0)
self._odom_yaw = wrap_angle(self._odom_yaw + yaw_delta)
self._odom_x += v * math.cos(yaw_avg) * dt
self._odom_y += v * math.sin(yaw_avg) * dt
```

This is *midpoint* (use yaw at the midpoint of the interval for the position update), which is
2nd-order accurate — strictly more accurate than the forward-Euler that would just use the start-
of-interval yaw. Comment matches code. Good.

**dt sanity-check is reasonable (line 335).** `if dt <= 0 or dt > 0.5: return` — guards against
clock skew and against the first call where `_odom_last_t` was set in `__init__`. Good.

**No covariance scaling with speed (lines 175-188).** The covariances are constant — `0.001` on
x/y position regardless of speed. In reality, wheel-odom uncertainty grows linearly with distance
travelled and quadratically with rotation. EKF will receive an over-confident pose at high
speeds. For competition purposes this is probably OK because EKF fuses GNSS heavily; for log
analysis it can throw off NEES diagnostics. **P2.**

**`_publish_odom` runs at the same 20 Hz as `_publish_state` (called from inside it, line 331).**
20 Hz is acceptable for EKF input but lower than the typical 50 Hz wheel-odom rate. The function
docstring on line 23 says "@ 50 Hz" but the actual rate is `state_pub_rate_hz` (20 Hz). **P2 —
docstring/comment mismatch.**

**`_publish_state` claims to be "Legacy ActuatorState contract (for webui compatibility)" (line
310).** Approximating throttle/brake/steer from measured wheel RPMs is a reasonable round-trip:
the webui sees the actual state, not the command. **One bug:**

```python
diff_mps = ((r_rpm - l_rpm)) * self._m_per_rev / 60.0
...
steer = max(-1.0, min(1.0, (diff_mps / self._track_w) / self._max_w)) if self._max_w > 0 else 0.0
```

The conversion `diff_mps / track_w` gives ω in rad/s. Correct. Divided by `_max_w` to get a
fraction in `[-1, 1]`. Correct. `_max_w > 0` guard prevents divide-by-zero. Good. **No bug —
correct.**

**Throttle/brake confusion (lines 319-320).** When `avg_mps < 0` (reversing), `throttle = 0` and
`brake = -avg_mps / _max_v` — i.e. the webui will display "brake = 1.0" while the chassis is
**driving in reverse**. This is wrong terminology — there should be a "reverse" indicator, not
"brake." The webui side may already account for this, but if not, the operator sees the brake bar
swing while reversing and gets confused. **P2 — clarify webui contract or change the field.**

**Mode field hard-coded (line 326).** `msg.mode = 'D' if not self._estop else 'N'` — only ever
'D' or 'N'. The webui supports N/D/S/R per CLAUDE.md but the actuator node never reports 'S'
(sport) or 'R' (reverse). Either delete the field from the contract or wire it up. **P2.**

**Watchdog flag hard-coded false (line 327).** `msg.watchdog_active = False` — the field exists
in the message but is never set true. If the cmd-vel watchdog fires (no command for 500 ms), this
flag should be **true** so the operator can see "stale command, holding stop." **P1 — wire this
up; the field is exactly the right place to surface watchdog state.**

### package.xml / setup.py / setup.cfg / __init__.py

**`package.xml` is format 3 (line 3).** `<package format="3">` — current standard. Good.

**Build type declared correctly (line 23).** `<build_type>ament_python</build_type>` in
`<export>`. Good.

**License is `MIT` (line 8).** Matched by `LICENSE` file at package root (1023 bytes). Good. Note:
ROS 2 default is Apache-2.0 (per the standards doc), but MIT is a perfectly valid choice; just
ensure the rest of the repo is consistent.

**Dependencies use `<depend>` (lines 10-15).** The standards doc § 1 prefers `<exec_depend>` for
ament_python runtime deps for precision, but `<depend>` (= build + build_export + exec) is a
common shortcut. Not a defect.

**`python3-serial` listed (line 15).** Correct rosdep key for `pyserial`. Good. **However, in
`setup.py` line 14 also lists `pyserial` in `install_requires`** — this triggers pip to install
pyserial again at `pip install` time. For an ament_python package consumed via colcon + rosdep,
listing in `package.xml` only is preferred; the `setup.py` `install_requires` is for when someone
`pip install`s the package directly. Acceptable but redundant. **P2.**

**`<test_depend>` block correct (lines 17-20).** Standard set: `ament_copyright`, `ament_flake8`,
`ament_pep257`, `python3-pytest`. Matches the boilerplate `test/` files. Good.

**Maintainer email is `avlab@cpp.edu` (line 7).** Real address; good.

**`description` is single-line and accurate (line 6).** "Diff-drive actuator bridge: cmd_vel to
Teensy serial (SparkMAX)". Good.

**Version is `0.0.0` (line 5).** Acceptable for a pre-1.0 internal package. **P2** — bump to e.g.
`0.1.0` once safety issues are fixed.

**`setup.py`** (lines 1-30):

- `find_packages(exclude=['test'])` — correct.
- `data_files` installs `package.xml` and the resource marker. **No launch / config installation
  here — those live in `avros_bringup` per CLAUDE.md, which is the correct pattern.**
- `entry_points['console_scripts']` resolves to `avros_control.actuator_node:main` — matches
  the `main()` function on line 413. Good.
- `install_requires=['setuptools', 'pyserial']` — see note above re duplication.
- `extras_require={'test': ['pytest']}` — fine.
- `zip_safe=True` — okay for pure Python without resource files.
- **No `tests_require=` (deprecated anyway in modern setuptools).** Fine.

**`setup.cfg` (3-line file)** correctly sets `script_dir`/`install_scripts` to `lib/avros_control`
so `ros2 run avros_control actuator_node` finds the executable. Standard. Good.

**`__init__.py` is empty (0 bytes).** Standard ament_python idiom. Good.

**Resource marker `resource/avros_control` is empty.** Correct.

**LICENSE file present at package root.** Good.

### tests

**Boilerplate-only.** Three files, all standard ament-generated, no real coverage of the node.

- `test_copyright.py` (line 20): `@pytest.mark.skip(reason='No copyright header has been placed
  in the generated source file.')`. Skipped because source files lack copyright headers. Either
  add the headers (`ament_copyright --add-missing 'AV Lab' mit`) or accept the skip. The standards
  doc § 5 notes this is acceptable. **P2.**

- `test_flake8.py`: runs ament_flake8 with default config. Will catch PEP-8 violations. Useful but
  doesn't test behavior.

- `test_pep257.py`: runs ament_pep257 for docstrings. Useful but not behavioral.

**Missing unit tests (P1 for safety code):**

For a 426-line safety-critical control node, the following pure-function tests are trivially
writable, take no hardware, and would catch *most* of the regression cases I'd worry about.
Recommended `test/test_kinematics.py`:

- `test_yaw_from_quaternion_zero()` — zero rotation → yaw 0.0
- `test_yaw_from_quaternion_pi_over_2()` — z = sin(π/4), w = cos(π/4) → yaw = π/2
- `test_yaw_from_quaternion_pi()` — yaw = ±π wraps correctly
- `test_wrap_angle_in_range()` — already-wrapped angles unchanged
- `test_wrap_angle_above_pi()` — π + ε → -(π - ε)
- `test_wrap_angle_below_minus_pi()` — -π - ε → π - ε
- `test_diff_drive_inverse_pure_translation()` — v=1, w=0 → l_mps == r_mps == 1
- `test_diff_drive_inverse_pure_rotation()` — v=0, w=1 → l_mps = -track/2, r_mps = +track/2
- `test_slew_limiter_acceleration_clamped()` — large step, accel cap applied
- `test_slew_limiter_deceleration_uses_decel_cap()` — toward zero, decel cap (different)
- `test_slew_limiter_sign_change_uses_decel_cap()` — forward → reverse
- `test_slew_limiter_at_target()` — `dv = 0`
- `test_midpoint_pose_pure_x()` — yaw=0, v=1, dt=1 → x += 1, y unchanged
- `test_midpoint_pose_curve()` — v=1, w=1, dt=1 → known closed-form

These would fit in ~100 LOC, run in milliseconds, and lock in the kinematics for future
refactors. **P1.**

**Missing integration tests:**

A `launch_pytest`-style test that brings up the node against a virtual serial port (use
`socat -d -d pty,raw,echo=0 pty,raw,echo=0` to create a pty pair, point the node at one end, and
have a test harness on the other end emit fake `E` lines and assert the node publishes
`/wheel_odom` correctly). This is a moderate effort (200 LOC fixture) and would be the gold
standard. **P2 — nice to have but lower urgency than the unit tests.**

**No regression tests for known-bug-fixed cases.** Per CLAUDE.md the slew limiter was added in
response to a brown-out problem; the midpoint integrator was added in response to over-rotation
on turns. Neither has a regression test. If someone "simplifies" the slew limiter or reverts to
forward Euler in a future edit, no test will catch it. **P1.**

### actuator_params.yaml coverage

**1:1 match with `declare_parameter` calls.** Every parameter declared in `actuator_node.py:58-84`
is present in `actuator_params.yaml` with a default consistent with the declaration. No
orphaned YAML keys, no node-side parameters missing from YAML.

| Parameter | actuator_node.py default | YAML value | Match |
|---|---|---|---|
| `serial_port` | `/dev/ttyACM0` | `/dev/ttyACM0` | yes |
| `serial_baud` | 115200 | 115200 | yes |
| `track_width_m` | 0.7366 | 0.7366 | yes |
| `m_per_motor_rev` | 0.01994 | 0.01994 | yes |
| `max_linear_mps` | 1.5 | 1.5 | yes |
| `max_angular_rps` | 1.0 | 1.0 | yes |
| `max_linear_accel_mps2` | 1.0 | 1.0 | yes |
| `max_linear_decel_mps2` | 1.5 | 1.5 | yes |
| `max_angular_accel_rps2` | 2.0 | 2.0 | yes |
| `heading_hold_deadband` | 0.05 | 0.05 | yes |
| `heading_kp` | 1.5 | 1.5 | yes |
| `yaw_rate_kp` | 0.3 | 0.3 | yes |
| `cmd_timeout_s` | 0.5 | 0.5 | yes |
| `control_rate_hz` | 50.0 | 50.0 | yes |
| `state_pub_rate_hz` | 20.0 | 20.0 | yes |
| `kFF` | 0.000197 | 0.000197 | yes |
| `kP` | 0.0004 | 0.0004 | yes |
| `kI` | 0.0 | 0.0 | yes |
| `kD` | 0.0 | 0.0 | yes |
| `odom_frame` | odom | odom | yes |
| `base_frame` | base_link | base_link | yes |

**Parameters that should exist but don't (P1 from above sections):**

- `imu_stale_timeout_s` — for IMU staleness detection
- `serial_response_timeout_s` — for "no E line in N ms → declare Teensy dead"
- `estop_decel_mps2` — separate cap for emergency-stop slew (currently zero/instant)
- `max_angular_decel_rps2` — separate decel cap for ω
- `bus_voltage_min_v` — for brown-out detection if/when DIAG ingestion is added

**No parameter type / range constraints.** None of the `declare_parameter` calls use
`ParameterDescriptor` with a `floating_point_range` / `integer_range`. A user setting
`max_linear_mps = -5` would crash the node mid-loop. The standards doc § 2 recommends `ParameterDescriptor`
for "type, range, and read-only constraints." **P2.**

**No range validation in the (missing) `add_on_set_parameters_callback`.** Already noted as P1 in
rclpy idioms.

## Safety analysis (failure modes table)

| # | Failure mode | What happens today | What should happen | Severity |
|---|---|---|---|---|
| F1 | Operator presses Ctrl-C | `KeyboardInterrupt` → `destroy_node` writes one `S` then closes serial. Reader thread is daemon, dies with process. | Same, plus install SIGTERM handler so systemd-stop also stops cleanly. | OK (P2 to harden) |
| F2 | Jetson loses power mid-drive | Teensy stops receiving heartbeats; firmware-side cmd-loss timeout (per CLAUDE.md) stops motors. | Same (firmware handles it). | OK |
| F3 | USB cable to Teensy unplugged at runtime | `_serial_reader` catches `SerialException`, logs at full rate, sleeps 100 ms, retries forever. Control loop keeps writing into dead handle, every write logs an error. **Chassis keeps moving** until Teensy's own firmware watchdog (per CLAUDE.md) trips. | Detect dead handle, set `_estop`, attempt re-open with backoff, publish `watchdog_active=True` on actuator_state. | **P0** |
| F4 | Teensy resets / hangs (no E lines) | Control loop keeps writing setpoints; `_l_meas_rpm` / `_r_meas_rpm` freeze at last value; webui sees stale telemetry. | Track timestamp of last E line; if > 200 ms, force `_estop`. | **P0** |
| F5 | Xsens IMU dies / stops publishing | `_imu_fresh = True` permanently; `_current_yaw` frozen; heading-hold applies bogus correction forever. | Track IMU stamp; if > 200 ms, disable heading-hold and yaw-rate correction; log throttled warning. | **P0** |
| F6 | E-stop button pressed (via webui) | `_estop = True`; slew steps to 0 (infinite jerk); `S` written. **Next non-estop ActuatorCommand silently clears it.** | Latch the e-stop; require explicit clear (service or specific flag); use a bounded decel cap. | **P0** |
| F7 | E-stop fires from Nav2 (no ActuatorCommand path) | No way for Nav2 to set `_estop` — only `ActuatorCommand` can. Nav2 just stops sending `cmd_vel`, watchdog times out → motors stop after 500 ms. | Add a topic / service for software e-stop that any node can publish; or document that hardware e-stop is the only Nav2-visible path. | P1 |
| F8 | `cmd_vel` watchdog tripped (no command for >500 ms) | `v_req, w_req = 0`; slew limiter ramps current setpoint to zero using **`decel` cap**. Good. But `watchdog_active` flag on ActuatorState is hard-coded false. | Same behavior, but set `msg.watchdog_active = True` so webui can show it. | P1 |
| F9 | Operator publishes `cmd_vel` faster than 50 Hz | Last-message-wins; control loop reads whatever is current. No issue. | Same. | OK |
| F10 | Nav2 publishes `cmd_vel` faster than its planner ticks (e.g. 50 Hz controller, 10 Hz planner) | Same — last-wins. | Same. | OK |
| F11 | Bus voltage drops during hard accel | Not detected. Jetson may brown-out before any software response. | Parse bus voltage from D lines; if < 11.0 V, force a soft stop. | P1 |
| F12 | Quaternion in IMU msg is NaN or all-zero | `yaw_from_quaternion` returns 0.0 (numerically); heading-hold locks at 0. | Check `|q|` is in [0.5, 1.5]; if not, treat as IMU-stale. | P2 |
| F13 | Operator pushes joystick beyond `max_linear_mps` | Clamped on intake (lines 205-206, 222-223). Slew limiter then ramps. | Same — correct. | OK |
| F14 | `kFF / kP` push to Teensy is dropped on serial reset | Teensy uses prior gains (or factory defaults). No verification. | Read back D line; assert gains were applied. | P1 |
| F15 | Slew limiter computes `dt` from a stale parameter, not real-time | Uses `_ctrl_dt` (constant) — under-runs slew on dropped ticks. | Use measured dt from `now - last_tick`. | P1 |
| F16 | Yaw-rate correction overshoots `_max_w` | `w + 0.3 * w_err` not re-clamped to `_max_w`; can produce `> 1.0` rad/s. | Clamp the corrected ω before diff-drive inverse. | P1 |
| F17 | Reader thread regex doesn't match a renamed E format | Silent — `_l_meas_rpm` etc. stop updating. | Log throttled parse failures. | P1 |
| F18 | E-stop slew = instant zero step | SparkMAX velocity PID asked to step from 1.5 m/s to 0 in one tick — saturates current, brown-out risk. | Use a decel cap (e.g. 3 m/s²) for emergency stop. | **P0** |

## Cross-cutting issues

**1. Failure-state observability is poor.** The `/avros/actuator_state` topic carries
`estop`, `mode`, `watchdog_active`, throttle/brake/steer — all of which are *commanded* state, not
*measured fault* state. Three categories of observable fault should be on this topic (or a sibling
diagnostics topic):

   - serial-link health (alive / silent / dead)
   - IMU staleness (fresh / stale)
   - bus voltage (volts; brown-out warning threshold crossed)

This is the difference between "the chassis just stopped, why?" being answerable from a Foxglove
recording vs. requiring a Jetson SSH session. **P1 cross-cutting.**

**2. The node assumes the Teensy is "always there."** Multiple defects (F3, F4, F14) all share
the root cause: the node has no model of "Teensy state" — alive, boot, normal, fault, dead. The
fix is the same in all three cases: add a `_teensy_state` enum and a per-tick check that
transitions it based on E-line freshness, serial-write success, and DIAG response. **P0 cross-
cutting (covered by F3+F4 P0s).**

**3. The IMU integration is similarly trusting.** If the Xsens has any failure mode that produces
"plausible-looking but wrong" data (which Xsens has a few of: GNSS heading lock loss producing
yaw jumps, magnetic distortion biasing heading), the node will silently apply heading-hold to the
wrong direction. Adding a covariance / status check from the IMU message would harden this.
**P1 cross-cutting.**

**4. Documentation/code drift.**

   - Module docstring (line 23) says `/wheel_odom @ 50 Hz` but actual rate is `state_pub_rate_hz`
     (20 Hz).
   - Line 23 mentions `/avros/actuator_state @ 20 Hz` correctly.
   - Comment on line 286 ("close loop on ω") overstates what the math actually does.

   These are minor but accumulate; would benefit from a top-of-file "this is the architecture,
   here are the actual rates" block. **P2.**

**5. Magic constants without parameters.**

   - Line 276: `abs(v) > 0.02` — minimum speed for heading-hold engagement. Should be
     `min_v_for_heading_hold_mps` parameter.
   - Line 284: `_max_w * 0.5` — heading-hold correction cap. Should be a separate parameter
     (`heading_hold_max_correction_rps`).
   - Line 298: `abs(v) < 1e-6 and abs(w) < 1e-6` — the "near-zero" threshold for "send S instead
     of L0 R0". Could be a parameter; would reduce chatter when the slew is at exactly 0.
   - Line 335: `dt > 0.5` — odom integration max-dt clamp. Should be a parameter.

   None of these are urgent; for a competition vehicle, parameterizing them lets you tune without
   recompiling. **P2.**

**6. No telemetry on slew saturation.** When the operator commands a step the limiter cannot
satisfy, `v_req - v_slew` is the saturation magnitude — useful for tuning but never published.
Easy to add to the state message. **P2.**

**7. No diagnostic_msgs/DiagnosticArray output.** ROS 2 has a standard pattern (`diagnostic_updater`)
for publishing diagnostics on `/diagnostics` consumed by `rqt_diagnostics`. Currently this node
publishes nothing of that form. **P2.**

**8. Reader thread is daemon (line 193).** Daemon threads die abruptly on interpreter shutdown.
Combined with the `_running = False` flag set in `destroy_node`, the reader can race with the
interpreter cleanup. Acceptable but worth being explicit about. **P2.**

## Punch list

### P0

These are the items where a reasonable engineer would refuse to drive on the current code.

1. **Add serial-link liveness detection (F3, F4).** Time-stamp the last successful `_serial.write`
   *and* the last `E` line received. If either exceeds 200 ms, set an internal `_serial_dead`
   flag, force `_estop = True`, and on `actuator_state` publish `watchdog_active = True`. Attempt
   re-open every 1 s with backoff. *File: `actuator_node.py:107-110, 369-398`. Estimated effort:
   one afternoon.*

2. **Add IMU staleness detection (F5).** Time-stamp the last `/imu/data` message. If > 200 ms,
   reset `_imu_fresh = False` and skip the heading-hold/yaw-rate branch. Log throttled warning.
   *File: `actuator_node.py:225-229, 274-289`. Estimated effort: 30 minutes.*

3. **Latch `_estop` until explicit clear (F6).** Once `_estop = True`, do **not** auto-clear on
   the next non-estop ActuatorCommand. Add either a service (`/avros/clear_estop`) or a specific
   `clear_estop` field on the message that the operator must invoke. *File: `actuator_node.py:209-223`.
   Estimated effort: 30 minutes.*

4. **Bound the e-stop deceleration (F18).** Replace the instant zero-step on `_estop` (lines
   258-259) with a slew-limited ramp using a dedicated `estop_decel_mps2` parameter (e.g. 3.0
   m/s²). Document that the SparkMAX brake-mode handles the rest. *File: `actuator_node.py:256-259`.
   Estimated effort: 15 minutes.*

5. **Confirm IGVC competition rule compliance for hardware e-stop is unblocked by these
   changes.** None of the P0 fixes above remove the requirement for a hardware-mushroom-button
   + contactor + wireless e-stop per the standards doc § 5 / IGVC rules. The software stack here
   is *layered on top of* the hardware path; the hardware path must exist independently. **Verify
   on the physical vehicle** that the contactor opens motor power on mushroom press, regardless
   of node state. (Not a code change; an inspection task before competition.)

### P1

rclpy / safety-control convention violations.

6. **Switch `/imu/data` subscriber to `qos_profile_sensor_data`** (best-effort, depth 5).
   *File: `actuator_node.py:163`.*

7. **Compute slew-limit `dt` from real time, not from the parameter.** Record `last_tick`,
   compute `dt = (now - last_tick).nanoseconds / 1e9`, clamp to `[0.001, 0.1]`, use that.
   *File: `actuator_node.py:197, 264-269`.*

8. **Wire `msg.watchdog_active` to actually reflect watchdog state.** Set `True` whenever
   `_estop` or both `has_actuator` and `has_cmd_vel` are False. *File: `actuator_node.py:327`.*

9. **Add `add_on_set_parameters_callback` with range validation,** especially for
   `max_linear_mps`, `max_angular_rps`, accel/decel caps, `cmd_timeout_s`. Reject negatives
   and zero. *File: `actuator_node.py:54-101`.*

10. **Throttle high-frequency error logs.** `serial_write` and serial-reader error paths use
    `get_logger().error(..., throttle_duration_sec=2.0)`. *File: `actuator_node.py:374, 397`.*

11. **Verify SparkMAX gain push.** After `KF/KP/KI/KD`, parse the next D line (or send a
    specific echo command) and assert the new value was applied. Log clear error otherwise.
    *File: `actuator_node.py:113-121`.*

12. **Clamp ω after yaw-rate correction (F16).** `w = max(-_max_w, min(_max_w, w + yaw_rate_kp *
    w_err))` before diff-drive inverse. *File: `actuator_node.py:289`.*

13. **Log on E-line parse failure (F17).** Inside `_serial_reader`, if a non-empty line doesn't
    match the regex, throttled-debug-log the unmatched line. *File: `actuator_node.py:387-395`.*

14. **Ingest D / DIAG lines and republish key fields on `/avros/actuator_state` or
    `/diagnostics`.** Bus voltage, watchdog state, tx/rx counts. *File: `actuator_node.py:376-398`.*

15. **Add unit tests for the kinematics and slew-limiter.** See test list in *tests* section.
    Target ≥ 12 cases, ≤ 100 LOC. *File: `test/test_kinematics.py` (new).*

16. **Clarify or rewrite the yaw-rate "closed loop" comment + math.** Either match the comment
    by replacing `w = w + yaw_rate_kp * w_err` with `w = w_target_from_user; w_actual = some_PI(w_target,
    measured_yaw_rate)`, or rewrite the comment to "feed-forward boost when under-rotating." *File:
    `actuator_node.py:286-289`.*

17. **Document parameters as "init-time only" if you don't add the runtime callback (P1 #9).**
    A docstring or comment in the YAML and the node `__init__` is enough. *File: both.*

### P2

Style, naming, polish.

18. Save subscription/timer handles on `self.<name>` for introspection.
19. Use `try/except` around `serial.Serial(...)` in `__init__` and exit with a clear error.
20. Handle SIGTERM with an explicit signal handler (write `S`, close port, then shutdown).
21. Bump `package.xml` version from `0.0.0` to `0.1.0` once safety fixes land.
22. Drop `pyserial` from `setup.py:install_requires` (kept in `package.xml`).
23. Remove dead `_l_pos_prev` / `_r_pos_prev` fields (lines 147-148) or wire up delta-position odom.
24. Compile `E_RE` at module level instead of inside the thread function.
25. Add `frame_id` constant to ActuatorState header (currently uses `base_frame` — fine, but
    document).
26. Add bounds-checking for quaternion magnitude before `yaw_from_quaternion`.
27. Parameterize the magic constants (heading-hold min-v threshold, max-correction cap,
    odom-dt-sane upper bound).
28. Publish slew saturation magnitude as a diagnostic.
29. Change `node = ActuatorNode()` setup so `finally` block doesn't NameError if `__init__`
    raises.
30. Add separate `max_angular_decel_rps2` parameter for ω consistency with v.
31. Fix module docstring rate (`/wheel_odom @ 50 Hz` → 20 Hz).
32. Add `ParameterDescriptor` with floating_point_range to declare_parameter calls for safety
    caps.
33. Add 'S'/'R' modes to the actuator_state.mode field (or remove the unused mode enum from the
    contract).
34. Reframe `brake` field on actuator_state when reversing — currently shows brake while driving
    in reverse.
35. Use `diagnostic_updater` for `/diagnostics` output.
36. `except Exception` → `except (serial.SerialException, OSError)` in serial paths.
37. Add a comment explaining the threading model (callbacks on executor thread, reader on a
    separate Python thread, GIL keeps writes to floats atomic).
38. Add `ros2 launch` smoke test (launch_pytest) that verifies node comes up and publishes
    `/avros/actuator_state` within 2 s — useful regardless of P1 unit tests.
39. Add covariance scaling with speed in `_publish_odom` (not urgent — EKF eats it either way).

## Positives

There is a lot of professional discipline visible in this code; the criticisms above are about
hardening, not foundational correctness.

- **Clear separation of concerns.** Kinematics, slew limiting, IMU integration, serial protocol,
  state publishing, odometry are all in distinct functions. Easy to read top-to-bottom.

- **Module docstring is excellent (lines 1-24).** Names the chassis, gear ratio, kinematics,
  heading-hold logic, and the input/output topics. This is the kind of docstring upstream packages
  rarely have.

- **Slew-rate limiting with separate accel/decel caps (lines 263-269).** Matches standards § 6
  exactly — "Robotics convention treats braking as asymmetric." Most beginner / intermediate
  control nodes don't bother with this; getting it right (modulo the `dt` and zero-step issues
  above) is a sign the author knows the failure modes.

- **Last-message-wins command priority (lines 235-250).** `actuator_command` overrides `cmd_vel`
  when both fresh; both contribute to the same `_target_v` / `_target_w`; both go through the
  same slew limiter and IMU corrections. This is *exactly* the right architecture — webui and
  Nav2 produce identical chassis behavior, which is what the user wants for testing parity.

- **Heading-hold gating logic (lines 274-289).** The `|w| < deadband AND |v| > 0.02` double
  condition correctly avoids engaging hold while stationary. The lock-on-entry (`if not
  _heading_locked`) and explicit unlock-on-turn are professional touches.

- **Midpoint pose integration (lines 348-352).** 2nd-order accurate; the comment notes the prior
  forward-Euler bug ("over-rotated displacement on turns"); the fix shows the author understood
  the math.

- **Diagonal covariance design (lines 175-188).** Setting unobservable axes (z, roll, pitch,
  vy/vz/vroll/vpitch) to 1e6 to tell the EKF "ignore these" is the textbook pattern. Setting yaw
  position and yaw rate to small values matches the diff-drive observability.

- **Parameter declaration is complete and matches YAML 1:1.** No drift, no missing keys, no
  orphaned YAML. Parameter coverage is the professional baseline; this code has it.

- **Fixed-rate timer at 50 Hz with 25× margin over `cmd_timeout_s` of 0.5 s.** Matches the
  standards doc § 4 / § 2 recommendations cleanly.

- **`fb_lock` correctly used.** The reader thread and the publisher both acquire the lock around
  the four shared variables. No torn reads.

- **Use of `rclpy.try_shutdown()` (line 422).** Correct idiom — doesn't raise on already-shut-
  down. Many nodes use bare `rclpy.shutdown()` and crash on second SIGINT.

- **Clean shutdown intent (line 402-410).** The `destroy_node` override sends `S` and closes the
  port — the *intent* is right. The race condition on the timer is fixable.

- **Parameter sourcing pattern (lines 86-101).** `p = self.get_parameter` followed by a tight
  block of assignments is readable. Keeps the `__init__` from sprawling.

- **Clear naming conventions.** `_target_v` / `_slew_v` / measured `_l_meas_rpm` distinguish
  command intent, slewed setpoint, and observed quantity. No `v` / `v2` / `v_final` ambiguity.

- **Test infrastructure exists.** Three boilerplate linter files are present with the right
  pytest markers. Linters will catch regressions in style. The behavioral test gap is the
  P1 issue, not the linter setup.

- **Python file-level structure follows ament_python conventions.** `setup.py`, `setup.cfg`,
  `package.xml`, `LICENSE`, `resource/avros_control` empty marker, `__init__.py` empty. No
  surprises.

- **Sensible parameter values.** `max_linear_mps = 1.5` matches IGVC top-speed limit; `cmd_timeout_s
  = 0.5` matches the upstream `diff_drive_controller` default; track gauge and `m_per_motor_rev`
  are precisely calibrated and documented in CLAUDE.md.

- **Track-drive specific hardening.** `kFF = 0.000197` ≈ 1/5076 RPM matches the *measured*
  Phase 4 max RPM of the right (slower) track. This shows the author tuned to the actual
  hardware, not to nameplate values.
