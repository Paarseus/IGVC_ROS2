# Cross-cutting: Documentation Quality — Review

## Summary

Two largely-correct documents (CLAUDE.md, TODO.md), one model pattern (firmware 3-doc CLAUDE/BRING_UP/FINDINGS), and a sprawling pile of session notes (7 dated changelogs, 2,844 lines). **The single highest-leverage P0 is rewriting README.md** — it describes an Ackermann + UDP + steering/throttle/brake/gear platform that doesn't exist; the real vehicle is a track diff-drive over USB-CDC serial. A competition judge clicking the team's GitHub link forms a fundamentally wrong picture. CLAUDE.md needs ~10 lines of surgical edits to close the route-graph (52 vs 16,651), ZED resolution (HD720 row), and missing-avros_sim Packages drift. TODO.md needs the Phase 2 P0 set rolled in. The changelogs need consolidation into a `CHANGELOG.md` at repo root. Replicating the firmware 3-doc pattern in `avros_control` and `avros_perception` would make those safety-critical packages competition-grade. **Documentation maturity by document:**

| Doc | Maturity | Drift risk |
|---|---|---|
| README.md | Low | Critical — fundamentally wrong about platform |
| CLAUDE.md | High | Moderate — known drift in 4-6 specific claims |
| TODO.md | High | Low — recent, concrete, current |
| docs/REALSENSE_SETUP.md | High | Low — pinned versions, replayable |
| docs/xsens_mt_manager_setup.md | Medium | Moderate — `~/AVROS/` references |
| docs/CHANGELOG_*.md | High detail, low form | Unmanageable volume (2,844 lines / 7 days) |
| docs/*.tex | Mixed | High — committed PDFs go stale |
| firmware/teensy_diff_drive/{CLAUDE,BRING_UP,FINDINGS}.md | High | Low |
| Per-package docstrings | Mixed | actuator strong, webui thin, perception medium |

The documentation is in better shape than most competition codebases at this stage — but the front-door README is critically broken for outsiders.

## Top-level documentation
### README.md

`/home/mspacman/IGVC_ROS2/README.md` (62 lines). The GitHub-facing front door for any outsider — judge, new team member, visitor. **Dramatically out of step with the actual codebase**:

- **P0 — Wrong title / workspace.** Title says "AVROS"; `README.md:46` says `cd ~/AVROS`. Per CLAUDE.md:7, repo is `IGVC_ROS2` and `~/AVROS/` is a forbidden dead branch.
- **P0 — Wrong kinematics.** `README.md:16, 19` say "SmacPlannerHybrid (Ackermann-aware)" / "Ackermann inverse kinematics." Real platform is a track diff-drive (CLAUDE.md "Diff-Drive Parameters", `package_avros_control.md`).
- **P0 — Wrong actuator transport.** `README.md:19` says "over UDP." Reality is USB-CDC serial at 115200 baud (CLAUDE.md, `firmware/teensy_diff_drive/CLAUDE.md`); UDP appears nowhere.
- **P0 — Wrong actuator surface.** `README.md:40` says "(steering, throttle, brake, gear)" — none of those exist on this platform. The real surface is two SparkMAX velocity setpoints per wheel.
- **P1 — Misses major workspace pieces.** No mention of `avros_perception` (the most-tested package per Phase 2), no ZED cameras, no NTRIP/RTK, no `vcs import` step, no kiwicampus patch script, no librealsense build, no safety/e-stop posture. The README's Quick Start is `colcon build` + a sim launch — nothing about real-vehicle bring-up.
- **P1 — `docs/avros_nav_demo.gif`** referenced as hero image; verify the asset matches the current platform (not the old Ackermann sim).
- **P1 — No license statement** at the top, even though every `package.xml` says MIT.

The README reads like an early-project pitch. CLAUDE.md is the only document accurately describing the repo. **A competition judge clicking the team's GitHub link forms a fundamentally wrong picture of the vehicle.**
### CLAUDE.md

`/home/mspacman/IGVC_ROS2/CLAUDE.md` (~440 lines). The de facto README — comprehensive and largely current.

What works:

- **Structure is excellent.** Project Overview → Build → Packages → Network → Sensors → TF → Topics → Actuator → Nav2 → WebUI → Remote Desktop → Launches → Configs → DDS → Known Issues → TODOs. Outsider locates any subsystem in <30 seconds.
- **Known Issues table** (~20 rows, file:line cited) is the workspace's biggest single institutional-knowledge asset.
- **Sensor sections** cite firmware versions, IPs, ports, verification dates ("Verified working — 100Hz IMU data confirmed on /dev/ttyUSB0").
- **Network inventory** with MAC addresses is unusual and useful.

Where it has rotted:

- **P0 — Route graph "52 nodes, 113 edges"** (line 279). Committed graph is 16,651 / 17,492 — ~300× drift per `package_avros_navigation.md`. Any route_server tuning based on the doc number is orders-of-magnitude wrong.
- **P0 — ZED row "HD720@15fps"** (line 389) contradicts CLAUDE.md's own Known Issues row that says HD720 was deprecated for ZED X in v5.2. Body says HD1080 (line 148).
- **P1 — `avros_sim` missing from Packages table** (line 68-75). Lists only 5 packages; `avros_sim` is the 7th.
- **P1 — kiwicampus PR2/PR3 patches not mentioned** in Known Issues (only PR1 is). Both PR2 (mutex) and PR3 (raytrace-clear) are applied per `scripts/apply_kiwicampus_patches.sh` and CHANGELOG_2026-04-29.
- **P1 — Track-width drift not propagated.** TODO.md notes effective track ≈ 0.7416 m vs configured 0.7366 m. CLAUDE.md "Diff-Drive Parameters" still cites 0.7366.
- **P1 — Hardware e-stop status invisible.** Per `package_firmware.md` no hardware e-stop is wired (P0 IGVC inspection blocker). CLAUDE.md is silent. A judge cannot tell the team is incomplete on safety.
- **P1 — `generate_graph.py` workflow.** CLAUDE.md "Build & Test" shows path-relative invocation; the script isn't installed via `setup.py` so this is the only way to run it. Undocumented as the only path.
- **P2 — No maintenance stamp** ("last updated") at top.
- **P2 — No vehicle photos / diagrams.**

**Verdict.** CLAUDE.md is dramatically more correct than README.md. The drift fixes are surgical — 10-15 lines of edits close all the listed P0/P1s.
### TODO.md

`/home/mspacman/IGVC_ROS2/TODO.md` (88 lines, last updated 2026-04-28). **The workspace's healthiest document.**

What works:

- Three priority buckets (High/Medium/Low) with rationale per item ("Blocks safe field testing").
- Items are concrete and actionable, citing file:line (`actuator_params.yaml`, `xsens.yaml`, `pipelines/hsv.py`).
- Ground-truth measurements captured: "(360° spin done 2026-04-28: wheels +0.7 % vs IMU; effective track ≈ 0.7416 m...)".
- "Done in YYYY-MM-DD session" trailer with commit SHAs (`a750e37`, `1587259`).

What could be better:

- **P1 — Doesn't cross-reference Phase 2 reports.** Phase 2 surfaced ~50 P0/P1 items per package; TODO.md's "High priority" has 7. Either Phase 2 P0s aren't yet rolled in, or TODO.md is the operator's working set vs Phase 2 the deeper audit. A single owner must merge before competition.
- **P1 — Some items are de-facto P0 but live under Medium.** "Test localization stack" is Medium; per `package_avros_sim.md` both EKFs share one config (dual `odom→base_link` publishers). That's a P0 if manifesting.
- **P1 — "Done" section will balloon.** Migrate to a CHANGELOG.md and prune.
- **P1 — TODO.md is more current than CLAUDE.md** on the same issues. Track-width drift, kiwicampus dual-clock workaround — both in TODO.md, not in CLAUDE.md.

## docs/ directory inventory
### Setup guides (REALSENSE, xsens_mt_manager)

**`docs/REALSENSE_SETUP.md`** (96 lines, 5-step) — strong. Pinned versions (librealsense 2.57.6, FW 5.13.0.50, realsense-ros 4.56.4, JetPack 6), citation-grade root-cause statement with three issue links, explicit "Why 4.56.4 and not 4.57.6" tradeoff, pitfalls list. Internally consistent (`~/IGVC` references). P1: add "verified on YYYY-MM-DD" stamp.

**`docs/xsens_mt_manager_setup.md`** (143 lines) — same pattern but **P1 stale workspace paths** (`~/AVROS/install/...` at lines 87, 110); the alternative auto-config path tells the reader to edit a file inside `install/` which gets clobbered on `colcon build --symlink-install`.

**`docs/PLAN_perception_phase4_phase5.md`** (~466 lines) — exemplary planning artifact. Per `package_avros_perception.md`, much of this plan was executed (test pyramid in place, hash-based threshold guard). P1: add a header noting which sections are complete (with CHANGELOG cross-references) so readers know it's history, not roadmap.
### Changelogs

Seven dated changelog files in `docs/` totalling 2,844 lines over 7 days:

| File | Lines | Topic |
|---|---|---|
| CHANGELOG_2026-04-23.md | 255 | Diff-drive commissioning |
| CHANGELOG_2026-04-24.md | 322 | Semantic segmentation Phases 0-3 |
| CHANGELOG_2026-04-25.md | 353 | Live HSV tuning, sensor mounts |
| CHANGELOG_2026-04-27.md | 462 | GPU anomaly, kiwicampus mutex patch |
| CHANGELOG_2026-04-27_field.md | 400 | Field session: outdoor HSV |
| CHANGELOG_2026-04-28.md | 381 | Costmap dual-clock, wheel-odom EKF |
| CHANGELOG_2026-04-29.md | 671 | kiwicampus raytrace-clear PR3, Nav2 BT |

What's good: every entry has a Scope, commit SHAs, "Files touched" git-status, "Out of scope" deferred-list, and cross-references to companion docs. The 2026-04-27 entry explicitly cross-references `perception_latency_investigation.pdf`. The wrong-then-right reasoning is preserved (e.g., the `kOutputMax_0` hypothesis being re-interpreted as PID-saturation in CHANGELOG_2026-04-27 / FINDINGS.md).

What's problematic:

- **P0 — These are session notes, not a changelog.** Standard `CHANGELOG.md` (per Keep a Changelog) records user-visible behavioral changes by version. These are operator workshop journals. There is **no `CHANGELOG.md` at the repo root.**
- **P1 — They don't update CLAUDE.md drift.** No changelog flags the 52→16,651 route-graph drift, the HD720→HD1080 ZED row, or the missing `avros_sim` Packages row.
- **P1 — They duplicate critical content.** The "kiwicampus master-grid bug" fix is documented only in CHANGELOG_2026-04-25 §3; CLAUDE.md "Known Issues" mentions kiwicampus PR1 only.
- **P1 — Naming inconsistent.** Six are `CHANGELOG_YYYY-MM-DD.md`; one is `_field.md`-suffixed. Risks `_morning`/`_afternoon` proliferation.
- **P2 — Volume.** 2,844 lines / 7 days → ~12,000 by competition. No judge will read that.
- **P2 — Header convention drift.** CHANGELOG_2026-04-28.md uses 14 top-level `#` headers; CHANGELOG_2026-04-27_field.md uses 5.

**Remediation:** roll the changelogs into a single `CHANGELOG.md` at repo root (one `## YYYY-MM-DD` H2 per session, most-recent-first, ~10 lines each); keep detailed files in `docs/sessions/` for deep-dive readers; link from CLAUDE.md.
### .tex / .pdf reports

Five `.tex` files (2,085 lines) + 2 PDFs (357 KB total) + 1 GIF (38 MB).

| File | PDF? | Topic |
|---|---|---|
| nav2_bringup_troubleshooting.tex | no | Nav2 lifecycle / bring-up |
| nav2_route_integration_report.tex | yes | nav2_route GeoJSON integration |
| nav2_testing_troubleshooting.tex | no | Nav2 testing patterns |
| perception_latency_investigation.tex | yes | GPU/depth_stabilization root-cause |
| realsense_d455_setup.tex | no | Older RealSense procedure |

- **P0 — `realsense_d455_setup.tex` duplicates `REALSENSE_SETUP.md`.** Two sources on a setup procedure is the textbook drift recipe. Delete the .tex.
- **P1 — None of the .tex files are referenced from CLAUDE.md / README / changelogs.** Readers can't tell which is current.
- **P1 — No Makefile for `.tex → .pdf`.** Edits silently leave the committed PDF stale.
- **P1 — `avros_nav_demo.gif` 38 MB** bloats every `git clone`. Should be externally hosted or git-lfs'd.

## firmware/ documentation
### the 3-doc pattern (CLAUDE / BRING_UP / FINDINGS)

`firmware/teensy_diff_drive/` contains three docs that together form the workspace's strongest documentation pattern:

| File | Lines | Role |
|---|---|---|
| `CLAUDE.md` | 128 | Stable spec — protocol, hardware, gains, "delta from upstream" |
| `BRING_UP.md` | 409 | Test plan — phase-by-phase prerequisites/risk/goal/procedure/pass/fail |
| `FINDINGS.md` | 153 | Post-mortem — verified results, blockers, tuned gains |

Why this works: **three docs, three audiences.** New contributor reads BRING_UP, maintainer reads CLAUDE, auditor reads FINDINGS. CLAUDE states what's true now; BRING_UP states how to get there with safety gates (`BRING_UP.md:181-188` pre-flight checklist before motors spin); FINDINGS preserves wrong-then-right reasoning (the `kOutputMax_0` hypothesis re-interpreted as PID-saturation). PID gains appear consistently in CLAUDE.md, FINDINGS.md, and `phase6c_pid_verify.py` — drift-detectable across all three.

What the rest of the workspace *should* replicate but doesn't:

- `avros_control` has safety-critical actuator code but no BRING_UP/FINDINGS equivalent. Slew-limiter, midpoint integrator, and gyro-stabilized turns each had a bug that motivated them; those bugs live only in commit messages and CHANGELOG_2026-04-23.md.
- `avros_perception` has the most-tested code but no BRING_UP. `PLAN_perception_phase4_phase5.md` is a planning doc, not a bring-up plan with safety gates.
- `avros_sim` has neither. Per `package_avros_sim.md`, the package has multiple P0 sim/real divergences with no triage document.

Issues within firmware:

- **P1 — `BRING_UP.md:348` has stale `~/AVROS/install/setup.bash`** that CLAUDE.md explicitly forbids.
- **P2 — CLAUDE.md TODOs** (lines 117-128) duplicate root `TODO.md`.

## In-code documentation
### Inline comments — sample assessment

Sampled first 100 lines of three Python nodes + firmware sketch.

- **actuator_node.py** — exemplary 24-line module docstring; every parameter has a units comment (`track_width_m', 0.7366  # 29 inches`); lines 64-70 explain *why* slew-rate exists ("12V rail brown-outs"). **Workspace best.**
- **perception_node.py** — module docstring states the load-bearing kiwicampus contract: "All four outputs carry the SAME header.stamp ... kiwicampus message-filters them" (lines 13-15). Solid.
- **webui_node.py** — 3-line module docstring. No safety-contract statement despite owning the only software auto-`estop:True` publish path on WebSocket disconnect. **P1: needs an e-stop contract paragraph.**
- **teensy_diff_drive.ino** — 35-line header preserving bug-fix forensics: "The old firmware used cls=48 for both, which does not exist in the spec — all previous KP/KI/KD/KF commands and BURN commands were silently discarded" (lines 64-66); "kF_0 is ID 16, NOT 17. ID 17 is kIZone_0" (lines 75-77). Future maintainers can't accidentally revert.
### Type hints

Inconsistent. Pure helpers (e.g. `yaw_from_quaternion`, `wrap_angle`, `publish_command`) are annotated; class methods (`__init__`, `_on_cmd_vel`, `_state_callback`, `_on_actuator_cmd`) typically are not. `webui_node.py:57` types the param (`msg: ActuatorState`) but not the return; `:68` does the reverse. `teensy_bridge.py` per Phase 2 firmware report: "Type hints throughout, Python 3.10+ syntax" — bridge tooling is *more* annotated than node code. No `mypy` config at repo root. **P1: annotate every node callback method signature; ROS message types are valuable IDE info.**
### Docstrings

Module docstrings present on all 4 sampled files (quality assessed above). Class docstrings minimal: `actuator_node.py` and `webui_node.py` each have one line; `perception_node.py:75` has **none** (the class itself is undocumented despite the module being well-covered). **`__init__` docstrings describing pubs/subs/params are uniformly absent**. The Pipeline ABC (`pipelines/base.py`) per `package_avros_perception.md` is not declared via `abc.ABC` / `@abstractmethod` and has no contract docstring on `run()`. **P1 single-most-leverage:** a 5-line docstring on every node `__init__` listing the topics + parameters — ~25 lines of doc, dramatic readability win.
### Magic-number labeling

ROS parameters are named/unitted well (`actuator_params.yaml` 1:1 match with declares; `teensy_diff_drive.ino` names every CAN constant with REV-Specs citations). In-function literal constants leak through: `actuator_node.py` has unnamed `0.02` (heading-hold min-v), `_max_w * 0.5` (correction cap), `1e-6` (near-zero), `dt > 0.5` (odom dt clamp). `pipelines/hsv.py:31 _DEFAULT_CLASS_IDS` duplicates class_map.yaml + perception.yaml (3 sources). `teensy_diff_drive.ino` heartbeat bytes `0x78, 0x01, 0x00, 0x12, ...` mostly unnamed. `avros_vehicle_driver.py:23-26` has `WHEELBASE`, `TRACK_FRONT`, `MAX_STEERING_RAD`, `WHEEL_RADIUS` as constants; should be parameters. **P1 cross-cutting** — no flake8 magic-number rule (WPS432) configured.

## Code-vs-docs drift (highest priority)

| # | Doc claim | Code reality | Severity |
|---|---|---|---|
| D1 | README "Ackermann" / "SmacPlannerHybrid (Ackermann-aware)" | track diff-drive | **P0** |
| D2 | README "over UDP to a Teensy" | USB-CDC serial 115200 baud | **P0** |
| D3 | README "steering, throttle, brake, gear" | per-wheel velocity setpoints | **P0** |
| D4 | README `cd ~/AVROS` | workspace `~/IGVC` per CLAUDE.md:7 | **P0** |
| D5 | CLAUDE.md "52 nodes, 113 edges" | 16,651 nodes / 17,492 edges | **P0** |
| D6 | CLAUDE.md "zed_front.yaml HD720@15fps" | HD720 deprecated; YAML uses HD1080 | **P0** |
| D7 | actuator_node.py docstring "/wheel_odom @ 50 Hz" | 20 Hz | P2 |
| D8 | actuator_node.py comment "closes the loop on ω" | feed-forward boost only | P1 |
| D9 | xsens_mt_manager_setup.md `~/AVROS/install/setup.bash` | `~/IGVC` | P1 |
| D10 | firmware BRING_UP.md `~/AVROS/install/setup.bash` | same | P1 |
| D11 | CLAUDE.md Known Issues lists kiwicampus PR1 only | PR2 (mutex) + PR3 (raytrace-clear) also applied | P1 |
| D12 | CLAUDE.md track gauge 0.7366 m | measured ≈ 0.7416 m per TODO.md | P1 |
| D13 | perception class IDs "single source" | 3 sources: class_map.yaml + perception.yaml + hsv.py:31 | P1 |
| D14 | CLAUDE.md "Packages" lists 5 | repo has 7 (avros_sim missing) | P1 |
| D15 | scripts/diagnose_sim.py docstring `ros2 run avros_sim diagnose_sim` | not installed via console_scripts | P1 |
| D16 | nav2_sim_overrides.yaml "VelodynePuck sees ground" | `min_obstacle_height` problem | P1 |
| D17 | test.py "MAX_DUTY = 0.6" | firmware MAX_DUTY = 0.30 | P1 |

**Pattern:** drift concentrated in (a) public-face README (D1-D4), (b) numeric/version claims that age (D5, D6, D11, D12), (c) docstrings describing partially-realized behavior (D8, D13). CLAUDE.md is more current than README.md; TODO.md is more current than CLAUDE.md.

## License + copyright

- **All 7 `package.xml` declare `<license>MIT</license>`** — consistent.
- **4 of 7 packages have a LICENSE file** (`avros_bringup`, `avros_control`, `avros_msgs`, `avros_navigation`). **3 missing**: `avros_perception`, `avros_sim`, `avros_webui`. **No LICENSE at repo root.**
- **No copyright headers in `.py` source files.** All 5 ament_python packages have `test_copyright.py` boilerplate but it is `pytest.mark.skip`-ped because no headers exist. The `<test_depend>ament_copyright</test_depend>` becomes dead weight.
- **MIT is a valid choice but the standards reference notes Apache-2.0 is the de facto ROS 2 default.** Third-party packages cloned via `avros.repos` (zed-ros2-wrapper, realsense-ros) are typically Apache-2.0; mixing is fine but the team should verify compatibility.

**P1 fixes:**

- Add LICENSE at repo root.
- Add LICENSE to perception/sim/webui (cp from existing).
- Either `ament_copyright --add-missing 'AV Lab' mit` and unskip `test_copyright.py`, or drop `ament_copyright` from test_depends.

## Documentation punch list
### P0 (would block IGVC inspection or cause an outsider to fail)

1. **README.md says Ackermann + UDP + steering/throttle/brake/gear; the platform is diff-drive serial-over-USB.** A judge clicking the GitHub link forms a fundamentally wrong mental model. Rewrite README.md as a thin trampoline to CLAUDE.md, or replace its body with the relevant parts of CLAUDE.md. Cite: `README.md:16, 19, 40, 46`. Drift D1-D4.
2. **README.md `cd ~/AVROS` workspace path.** Forbidden by CLAUDE.md:7. New team member following README will fail the first command. `README.md:46`. Drift D4.
3. **CLAUDE.md "52 nodes, 113 edges" route graph.** Reality is 16,651 / 17,492 — a 300× gap. Either re-tune the graph and update CLAUDE.md, or update CLAUDE.md to reflect the committed file. `CLAUDE.md:279`. Drift D5.
4. **CLAUDE.md zed_front.yaml row "HD720@15fps".** HD720 was deprecated for ZED X in v5.2 per CLAUDE.md's own Known Issues table. Update to `HD1080@15fps`. `CLAUDE.md:389`. Drift D6.
5. **No top-level LICENSE file in repo.** GitHub flags this as missing and competition rules typically require licensing clarity. Cp `src/avros_bringup/LICENSE` to repo root.
6. **Demo gif `docs/avros_nav_demo.gif` referenced by README is 38 MB**, bloats every clone, and may be the only image — verify it actually shows current platform behavior (not the old Ackermann sim) before keeping. `README.md:9`.
### P1 (professional polish)

7. **CLAUDE.md missing rows.** Add `avros_sim` to Packages table; add kiwicampus PR2/PR3 to Known Issues (both applied per the patch script). Drift D11, D14.
8. **`~/AVROS/` references in `xsens_mt_manager_setup.md`, `firmware/teensy_diff_drive/BRING_UP.md:348`.** Search-and-replace to `~/IGVC/`. Drift D9, D10.
9. **Consolidate changelogs** into a single `CHANGELOG.md` at repo root; keep detailed files in `docs/sessions/`; link from CLAUDE.md.
10. **3 packages missing LICENSE files** (perception/sim/webui). Cp from existing.
11. **All `.py` files lack copyright headers.** Either run `ament_copyright --add-missing 'AV Lab' mit` and unskip `test_copyright.py`, or drop the test dep.
12. **Node `__init__` docstrings absent** describing pubs/subs/params. Add 5-line docstring per node — major readability win for ~25 lines of doc.
13. **`webui_node.py` doesn't document the e-stop contract** despite owning the only software auto-e-stop path on WebSocket disconnect.
14. **PerceptionNode class has no docstring.** Add 3-5 lines stating the kiwicampus 4-topic contract.
15. **TODO.md "Done" section grows unbounded** — migrate to CHANGELOG and prune.
16. **Phase 2 reports not cross-referenced from TODO.md.** Roll Phase 2 P0/P1 set in before competition.
17. **`PLAN_perception_phase4_phase5.md`** needs a "what's executed" header so readers know it's history.
18. **No diagrams** (vehicle photo, sensor placement, ROS topic graph, rendered TF tree). IGVC pre-event documentation expects at least one.
19. **Delete `realsense_d455_setup.tex`** (superseded by REALSENSE_SETUP.md).
20. **Mismatched comments**: `actuator_node.py` "closes the loop on ω", `nav2_sim_overrides.yaml` VelodynePuck-as-obstacles, `test.py` MAX_DUTY = 0.6. Drift D8, D16, D17.
21. **HSV class IDs duplicated in 3 places** — resolve by name from LabelInfo.
22. **Type hints sporadic** — annotate every node callback method signature.
23. **Magic numbers** in actuator_node.py / hsv.py / firmware without named constants.
24. **No Makefile for `.tex → .pdf`** — edits silently leave committed PDFs stale.
25. **Hardware e-stop status invisible** in CLAUDE.md despite being a P0 IGVC inspection blocker.
26. **Add "verified on YYYY-MM-DD"** stamps to REALSENSE_SETUP.md, xsens_mt_manager_setup.md.
### P2 (nice-to-have)

27. CLAUDE.md needs a "last-updated" / "version" header.
28. Changelog filename convention should be documented (six are `YYYY-MM-DD.md`, one has `_field` suffix).
29. `avros_nav_demo.gif` (38 MB) externally host or git-lfs.
30. TF tree could be Mermaid (renders on GitHub) instead of ASCII.
31. Per-package LICENSE files could symlink to a single root LICENSE.
32. Pipeline ABC docstring should state "subclasses must return PipelineResult with mask shape (H, W) dtype uint8 …".
33. Add CONTRIBUTING.md, CODE_OF_CONDUCT.md, `.github/` issue+PR templates.
34. `docs/foxglove_layout.json` is unreferenced from any doc.
35. `.tex` files lack "supersedes/obsoleted by" headers when stale.
36. No `mypy` / `ruff` / `black` config — style enforced ad hoc via ament_flake8 / ament_pep257.

## Positives

- **CLAUDE.md** (~440 lines) is the de facto README and largely correct; most operations succeed by following it alone.
- **CLAUDE.md "Known Issues" table** (~20 rows, file:line cited) preserves institutional knowledge that competition teams typically lose between cohorts.
- **`firmware/teensy_diff_drive/` 3-doc pattern (CLAUDE/BRING_UP/FINDINGS)** — should be replicated in `avros_control` and `avros_perception`.
- **`actuator_node.py` module docstring** sets the bar for the workspace; every parameter has a units comment.
- **`perception_node.py` module docstring states the kiwicampus bit-identical-stamp contract** at lines 13-15 — the single most load-bearing constraint of the perception path.
- **TODO.md** items reference file:line and capture measured ground truth ("effective track ≈ 0.7416 m vs configured 0.7366 m").
- **Setup docs are pinned and replayable.** REALSENSE_SETUP.md cites versions, issue links, build flags.
- **Changelogs preserve wrong-then-right reasoning.** CHANGELOG_2026-04-27 records the GPU anomaly was `depth.depth_stabilization` not NEURAL_LIGHT.
- **`perception.yaml` carries inline calibration history** with dates and reasons per tuning iteration.
- **`avros_perception` test_hsv_thresholds.py SHA256 hash guard** forces threshold changes through PR review — governance most teams skip.
- **`license` field is consistent across all 7 packages** (MIT declared everywhere in package.xml).

