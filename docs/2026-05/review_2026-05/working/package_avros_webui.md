# avros_webui — Review

## Summary

`avros_webui` is a 211-LOC ament_python package providing one node, `webui_node`: a FastAPI + uvicorn HTTPS server at `wss://<jetson>:8000/ws` that bridges nipplejs joystick frames to `/avros/actuator_command` (avros_msgs/ActuatorCommand). uvicorn runs on the main thread, `rclpy.spin` runs on a daemon thread, shared mutable state is guarded by `threading.Lock`. Disconnect → e-stop is implemented correctly via a `finally:` clause and works on every exit path.

**P0:** no authentication on the control WebSocket — anyone reachable to TCP 8000 (vehicle LAN, Tailscale, IGVC venue Wi-Fi) can drive the chassis. Also flagged as P0 (project-level, not strictly this package's fault): the webui e-stop is software-only, while IGVC AutoNav rules require a hardware mechanical e-stop and a wireless e-stop. The webui button is a fourth-layer convenience and does not satisfy inspection (`standards_firmware_safety.md` § 5).

**P1:** no server-side joystick-frame watchdog (saved end-to-end by actuator_node's 500 ms freshness fallback, but webui should own its own timeout); no NaN / type validation on incoming floats; FastAPI / uvicorn / websockets not declared as deps in `package.xml` (rosdep cannot resolve them); a TOCTOU window in the single-controller lock; e-stop state desynchronizes between client and server across reconnect; nipplejs CDN-loaded so offline operation breaks; no test for the disconnect → e-stop contract.

**P2:** mostly metadata polish — `<depend>` should be `<exec_depend>`, unused `<depend>std_msgs</depend>`, hardcoded `/home/dinosaur` paths in `webui_params.yaml`, no runtime parameter validation, no telemetry-staleness indicator in UI.

## Per-file findings

### webui_node.py

**Concurrency.** Canonical "FastAPI main thread + rclpy.spin daemon thread" split (lines 184-202). Shared state crossings: `_latest_state` (rclpy callback writer + asyncio reader, guarded by `_state_lock` lines 48-70) and `active_controller` (guarded by `controller_lock`, lines 103-163). rclpy publishers are thread-safe so calling `publish_command`/`publish_estop` from the asyncio thread is fine. No blocking I/O inside critical sections. Architecture is correct.

**Parameters.** All four (`web_port`, `ssl_certfile`, `ssl_keyfile`, `max_throttle`) declared with defaults before read (lines 30-38). Compliant. Missing: `add_on_set_parameters_callback` for runtime validation — `ros2 param set webui_node max_throttle 99.0` would silently bypass the bench-safety cap. **P2**.

**QoS.** Both publisher (line 40-42) and subscriber (line 43-46) use depth-10 default (reliable, volatile, KEEP_LAST=10). Matches actuator_node's subscriber (`actuator_node.py:159-162`). Correct for control-topic semantics.

**Input validation — partial / unsafe.** `publish_command` clamps throttle/brake/steer (lines 79-81). Good. But the WS handler at lines 138-139 does `float(data.get('x', 0))` with no `try/except` and no `math.isfinite` check. Two failure modes:
- `{'x': 'NaN'}` → `float('NaN')` returns `nan`. NaN flows through `max(0.0, nan)` → `nan` and reaches `publish_command`, which clamps via `min(max(0.0, nan), max_throttle)` = `nan`. **NaN reaches actuator_node.**
- `{'x': null}` or `{'x': 'foo'}` → raises `TypeError` / `ValueError` that escapes the handler. The `try:` only catches `WebSocketDisconnect` (line 159), so the exception propagates to the `finally:` which does publish e-stop — safety holds, but only because the malformed-input path happens to terminate the WS, not by intentional design. **P1.**

Mode validation (line 154 — `new_mode in ('N', 'D', 'S', 'R')`) is good.

**E-stop on disconnect — correct.** The `finally:` at lines 161-165 always clears `active_controller` and publishes e-stop, regardless of how the handler exits. Safety contract verified.

**No server-side joystick-frame watchdog.** `await websocket.receive_json()` (line 132) blocks indefinitely if the TCP connection goes half-open (phone Wi-Fi drops without sending FIN). The kernel keepalive default is ~2 hours; meanwhile the last published ActuatorCommand stays in the actuator_node's queue. End-to-end safety is salvaged by actuator_node's 500 ms freshness check (CLAUDE.md "Control priority"), but webui_node should own its own timeout. **P1.**

**SSL is silent on missing cert.** Lines 193-196 only enable SSL if both paths are non-empty strings; no check that the files exist. If either path is mistyped, uvicorn raises mid-startup, the `finally:` runs cleanup, but the operator may not realize they fell through to a non-SSL configuration that the `wss://` client cannot reach. Pre-flight existence check + clear log message would help. **P1.**

**Shutdown.** `daemon=True` spin thread + `uvicorn.run()` blocking on main → on Ctrl-C, uvicorn returns, `finally:` runs `destroy_node` + `try_shutdown`, daemon thread dies on process exit. Works but no graceful uvicorn `should_exit` signaling if rclpy decides to die first. Minor. **P2.**

**Logging** uses `self.get_logger()` (no bare `print()`). Two log calls only — could add accept/reject logs on WS handshake. **P2.**

### static/index.html + static/app.js

**HTML (88 lines).** E-stop button is full-width across the top of the screen, 24 px font, red — UX-correct (lines 20-29). Viewport disables zoom; CSS sets `touch-action: none; user-select: none` (lines 5, 17-18) to prevent accidental pinch/text-select during driving.

nipplejs is loaded from cdnjs (line 7) — **competition-day risk** if the Jetson is offline. Vendor it into `static/`. **P1**.

**JS (108 lines).** WS URL hardcoded `wss://${location.host}/ws` (line 3); falls down silently over `http://`. Auto-reconnect with 1 s backoff (lines 28-33). 20 Hz send rate matches actuator_node's 500 ms freshness window with 10× margin (line 57). Client-side joystick clamp to `[-1, 1]` (lines 79-80) — defense in depth alongside server clamp.

**State desync on reconnect.** Server initializes `estop = False` on every new WS handshake (line 127). JS retains the latched UI state. After a Wi-Fi blip and reconnect, the UI shows `E-STOP ACTIVE` while the server is publishing un-estopped commands — the JS only sends `{type: 'estop'}` on user click, not on every tick. **P1.**

Telemetry rendering at `data.t.toFixed(2)` (line 43) has no null-check; mismatched server schema would throw. **P2.**

No client-side auth — see Security Analysis.

### package.xml / setup.py / setup.cfg / launch

**`package.xml`.** Format 3, ament_python build_type — compliant. Issues:
- `<depend>` for `rclpy` / `avros_msgs` (lines 10-11) — should be `<exec_depend>` for ament_python (`standards_ros2_python.md` § 1). **P2.**
- `<depend>std_msgs</depend>` (line 12) — unused. The node imports only `avros_msgs.msg`. **P2.**
- **Missing FastAPI / uvicorn / websockets deps.** rosdep cannot resolve them. CLAUDE.md instructs operators to `pip install fastapi uvicorn[standard] websockets` manually; on Humble the apt names `python3-fastapi`, `python3-uvicorn` exist. Add corresponding `<exec_depend>` entries. **P1.**

**`setup.py`.** Correctly globs `static/*` and installs `package.xml` and resource marker (lines 13-18). The node uses `get_package_share_directory('avros_webui')` to find static files (webui_node.py:99-101) — symlink-install-compatible. `entry_points` maps `webui_node = avros_webui.webui_node:main` correctly. Minor: `extras_require['test']: ['pytest']` duplicates `<test_depend>python3-pytest</test_depend>` (harmless). **P2.**

**`setup.cfg`.** Conformant ament_python `script_dir` entries. Good.

**`webui.launch.py`.** Loads `actuator_params.yaml` and `webui_params.yaml` from `avros_bringup/config/`; YAML node-name keys match `name=` args. One `DeclareLaunchArgument` for `use_sim_time`. Issues:
- No `RMW_IMPLEMENTATION` set in this launch file. If the operator's shell defaults to FastDDS while `navigation.launch.py` (run separately) sets CycloneDDS, the two stacks can't see each other. **P2.**
- No SSL pre-flight check before invoking the node. **P2.**
- No `enable_actuator:=true` arg — UI-only testing requires hand-editing or `ros2 run`. **P2.**

**`webui_params.yaml`.** Hardcoded `/home/dinosaur/avros_certs/{cert,key}.pem` paths — works only on the Jetson; any other host falls through to non-SSL silently while the JS hardcodes `wss://`. Use `~/avros_certs` with `os.path.expanduser` resolution. **P2.**

`max_throttle: 1.0` in YAML overrides the node's safer default of `0.55` (webui_node.py:33). For paddock bench testing, a single tap on the phone joystick demands full speed. Recommend reducing to `0.5`. **P1.**

### tests

The three boilerplate ament linters are present (`test_copyright.py`, `test_flake8.py`, `test_pep257.py`). They check style only, not behavior.

**Missing tests for the safety contract.** The node has nontrivial logic — input clamping, disconnect → e-stop, mode validation, single-controller mutex, JSON parsing — and none of it is exercised. The most important test, by far, is "WebSocket disconnect publishes e-stop" (FastAPI `TestClient` + WS-open-then-close + assert `publish_estop` was called, ~20 lines). Add that minimum. **P1.**

## Security analysis

**P0: no authentication on the control WebSocket.** The node binds `host='0.0.0.0'` (line 200) — all interfaces, all networks (vehicle LAN, Tailscale 100.93.121.3, IGVC venue Wi-Fi). The WS handler (lines 110-165) accepts every handshake unconditionally and trusts every JSON frame. The only access control is the single-controller mutex — a hostile actor can win it by tight-loop reconnecting.

Realistic attack at competition: spectator joins venue Wi-Fi, `nmap` for port 8000, opens `https://<ip>:8000/`, accepts self-signed cert, drives the robot off course mid-run.

Mitigations, ordered:
1. **Bind to localhost or Tailscale-only** (`host='100.93.121.3'` or `host='127.0.0.1'`) — removes venue-Wi-Fi attack surface. ~15 minutes.
2. **Token in URL** (`wss://.../ws?token=<random>`) validated server-side against `webui_params.yaml` — minimum acceptable. ~20 minutes.
3. **HTTP Basic Auth** via FastAPI `Depends(HTTPBasic())` + cookie-based login flow — ~1 hour.

For competition prep, option 1 is mandatory and option 2 is the minimum.

**No origin check** on the WS handshake (line 111). Drive-by attack: any malicious web page loaded on the operator's phone can open `new WebSocket('wss://192.168.13.10:8000/ws')` and drive. Add `websocket.headers.get('origin')` allow-list check. **P1.**

**TOCTOU on single-controller lock.** Lines 113-125: the lock is released between the read (line 114-121) and the assignment (line 124-125). Two clients can both observe `active_controller is None` in the gap and both claim ownership. Hold the lock across the whole check-and-assign. **P1.**

**No rate limiting on connections.** With auth, becomes a non-issue. **P2.**

**Logged errors leak nothing sensitive** — only startup and disconnect lines. Good.

**Project-level P0 (not this package's defect):** IGVC AutoNav rules require a hardware mechanical e-stop center-rear 2-4 ft high AND a wireless e-stop with ≥ 50 ft range held by judges (`standards_firmware_safety.md` § 5). The webui's e-stop button is a software fourth-layer convenience and does **not** satisfy inspection. The team must have separate hardware solutions; `avros_webui` is a bench tool only.

## Cross-cutting issues

**Defense-in-depth is working as designed.** webui_node has no server-side joystick-frame watchdog, but actuator_node's 500 ms `/avros/actuator_command` freshness check (CLAUDE.md "Control priority") fires e-stop if commands stop arriving. End-to-end safety holds. webui_node should still own its own watchdog so the system is robust to actuator_node bugs or freshness-check regressions.

**Operator awareness.** If actuator_node dies but webui keeps publishing, the JS UI shows the last frozen `ActuatorState` snapshot — operator has no visual cue. Subscribe to telemetry timestamps and show "STALE" if older than 500 ms. **P2.**

**`max_throttle` is a UX clamp, not a safety clamp.** A misconfigured node publishing ActuatorCommand directly bypasses webui's cap. The actuator_node's clamps hold the real safety contract. Worth a docstring note.

## Punch list

### P0 — Must fix before competition

1. **No auth on the control WebSocket.** Bind to localhost or Tailscale-only (`webui_node.py:200`) AND add a token check on the WS handshake (`webui_node.py:110-125`). 1-hour fix.
2. **Software-only e-stop.** IGVC rules require hardware mechanical + wireless e-stop. Project-level — `avros_webui` is a bench tool, but the team must have separate hardware before relying on this stack for safety. (`standards_firmware_safety.md` § 5.)

### P1 — High priority

3. Lower `max_throttle` default in `webui_params.yaml` from `1.0` to `0.5`. (1 line.)
4. Add server-side joystick-frame watchdog: `await asyncio.wait_for(receive_json(), timeout=0.5)` + e-stop on TimeoutError. (`webui_node.py:132`, ~5 lines.)
5. NaN / type validation on incoming joystick floats: `try/except (ValueError, TypeError)` around `float()`, plus `math.isfinite` check before publishing. (`webui_node.py:138-143`.)
6. Add `<exec_depend>python3-fastapi</exec_depend>` + `python3-uvicorn` + websockets to `package.xml`. (3 lines.)
7. Fix TOCTOU race on single-controller lock — hold the lock across check-and-assign. (`webui_node.py:113-125`.)
8. State sync hazard on reconnect — server initializes `estop = False` on every handshake while UI may show ACTIVE. Send current server state in first frame after reconnect. (`webui_node.py:127`.)
9. Add Origin allow-list check on WS handshake to defeat drive-by attacks. (`webui_node.py:111`.)
10. Add integration test for "disconnect publishes e-stop" — entire safety contract is currently untested.
11. Vendor nipplejs into `static/` to remove the cdnjs runtime dependency. (`static/index.html:7`.)
12. Fail-fast pre-flight check on SSL cert/key existence before invoking `uvicorn.run`.

### P2 — Polish

13. `<depend>` → `<exec_depend>` for `rclpy` and `avros_msgs`.
14. Remove unused `<depend>std_msgs</depend>`.
15. Hardcoded `/home/dinosaur/avros_certs/` paths in `webui_params.yaml` — use `~` with `os.path.expanduser`.
16. Add `add_on_set_parameters_callback` for `max_throttle` runtime validation.
17. JS: detect `location.protocol`; use `ws://` when served over HTTP.
18. JS: null-check telemetry keys; UI staleness indicator if `/avros/actuator_state` stops.
19. Stronger disconnect feedback (border flash / tone) beyond small status text.
20. Add `enable_actuator:=true` arg to `webui.launch.py` for UI-only testing.
21. Set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in webui.launch.py for cross-launch consistency.
22. Per-WS-connection accept/reject log entries.
23. Graceful uvicorn shutdown signaling (`server.should_exit = True`); replace `daemon=True` spin thread with explicit `rclpy.shutdown()` + `join()`.
24. Remove duplicated pytest dep (`extras_require['test']` + `<test_depend>python3-pytest</test_depend>`).
25. Add PEP-257 docstrings on `_state_callback`, `publish_command`, `publish_estop`.
26. Switch launch `output='screen'` → `output='log'` for cleaner competition stdout.

## Positives

- Disconnect → e-stop is implemented correctly via `try/.../finally:` (lines 130-165) — fires on every exit path. Most important safety property of the package, and it works.
- Concurrency model is the canonical FastAPI + rclpy split: uvicorn on main thread, `rclpy.spin` on daemon thread, locks guarding shared state. No anti-patterns from `standards_ros2_python.md` § 6 apply.
- All four parameters declared with defaults before reading.
- Output clamping in `publish_command` is server-side (lines 79-81) — independent of client values.
- Mode validation before mutation (line 154).
- Single-controller mutex with clean `1008` close code (modulo TOCTOU P1).
- 20 Hz JS send rate gives 10× margin against the 500 ms actuator_node freshness window.
- HTML/JS is 196 lines combined, no transpilation, easy to audit. nipplejs is the right choice for phone.
- E-stop button is full-width, top-of-screen, always visible.
- Touch handling correctly locked down (`touch-action: none`, `user-select: none`, `user-scalable=no`).
- JS auto-reconnect with 1 s backoff.
- REP-103 steering inversion documented inline (lines 136-142).
- `get_package_share_directory` used for static/ — symlink-install-compatible.
- Boilerplate ament linters present.
- No `print()` calls — `self.get_logger()` throughout.
- `rclpy.try_shutdown()` (not `shutdown()`) in cleanup — idempotent.
- ament_python conformant: `setup.cfg`, resource marker, static-file glob in `setup.py`.
