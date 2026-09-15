# AVL / dinosaur codebase cleanup — design

**Date:** 2026-07-13
**Scope:** Full deep clean across the laptop (alexander@) and the Jetson (dinosaur), covering both live git repos, the deploy scripts, and the live perception dashboard. Goal: remove duplication and cruft, get real uncommitted work safely into git, without changing runtime behavior of the currently-live vehicle stack (motors/cameras/joystick stay up throughout).

## Out of scope (explicitly confirmed, do not touch)

- `Desktop/steve` (687MB, undocumented) — leave alone
- `avl_carla` (35GB local CARLA install) — leave alone
- `rhylon/joystick` (different project — mechanical BOM repo) — leave alone
- `AVL_ObjectDetection` and `AVL_ObjectDetection_local` (genuinely different projects) — leave alone
- `/home/dinosaur/carla-nav2-avl/carla-nav2-avl-jchy05/` — a **teammate's active clone** (branch `feature/jchy05`, commit from 2026-07-12) on the shared Jetson. Not cruft. Never touch.
- `bags/` (686GB field-test recordings on the Jetson's IGVC repo) — stays on disk exactly as-is; only touched via a `.gitignore` entry so it can never be accidentally staged.

## 1. Laptop dedup

| Path | Action | Why |
|---|---|---|
| `Desktop/IGVC_Nav2_SegFormer` | delete | exact duplicate clone of `IGVC_YOLO_Nav2_Prototype` (same remote, same commit) |
| `selfdrive_carla_ue5_ros2` | delete | stale clone of `carla-nav2-avl`; Jetson + GitHub `feature/alexander` are now the source of truth |
| `carla-nav2-backup` | delete | same reason as above |
| `kbot_stack`, `avl_ros2`, `nav2_perception_ws` | move to `~/archive/` | small, non-git, untouched since March/June — likely dead but not deleting outright |

## 2. Jetson — `carla-nav2-avl` repo (`feature/alexander`)

- Delete the `.bak_*` files (`perception_dinosaur.yaml.bak_*`, `viz_node.py.bak_*`) — the good state is already committed and pushed (`f709c69`, `66652b0`); git history is the real safety net now.
- No other changes — this repo is otherwise clean and pushed.

## 3. Jetson — `IGVC` repo (`main`)

Currently: 1 unpushed commit + a large uncommitted diff + several untracked additions. Plan, as separate commits (keeps history legible and revertable):

1. Push the already-committed `28ee28c` (Teensy serial fix + webui deadlock fix) — no working-tree change needed, just `git push`.
2. Commit the nav2/costmap tuning diff as-is (confirmed validated): `vx_max` 1.5→0.7, `decay_acceleration` 2.0→0.5 (both costmaps), `RateController hz` 1.0→3.0.
3. Revert `zed_front/left/right.yaml` `grab_resolution` back to `SVGA` (undoes an accidental reversion of a documented performance fix — HD1080 was shown to starve the 20Hz MPPI loop). If this brings the files back to HEAD's content exactly, this is just a `git checkout` of those 3 files, no new commit needed.
4. Add `scripts/` to git as a new commit — a real, previously-uncommitted toolkit (`analyze_M1/2/3.py`, `bag_preflight.py`, `extract_bag.py`, `tf_audit.py`, `deploy.sh`, `kill_ros2.sh`, `systemd/avros-webui.service` + `install.sh`, etc.), dropping the superseded `launch_stack.sh.bak` (confirmed: current `launch_stack.sh` has a real fix the `.bak` lacks).
5. Add `localization.launch.py` and `navsat.yaml` to git as their own commit (separate from `scripts/` — unrelated files) — real, currently-untracked launch/config files already in active use.
6. Add `bags/` and `src/realsense-ros/` to `.gitignore` (both stay on disk, untracked, on purpose).
7. Delete the remaining `.bak_*` files throughout the repo (config backups from various tuning sessions) — same rationale as the other repo, git history now covers rollback.
8. Push everything to `origin/main`.

## 4. Jetson — deploy scripts

`boot_stack.sh` and `full_stack_restart.sh` are near-duplicates; the only difference is that `full_stack_restart.sh` wraps the tmux server start in a `systemd-run --user --scope` for persistence across SSH teardown. Merge into a single script (keep the `full_stack_restart.sh` name, since restart is the common case) with a flag, e.g. `--boot`, that skips the scope-wrapping for the boot-time invocation (where the service cgroup already provides persistence). Delete the other file. Update any reference to the old script names (systemd service file, docs).

## 5. Live dashboard — switch to real MJPEG streaming (not snapshot polling)

**Revised per explicit feedback: never show stagnant/polled footage — always genuinely live.** The original plan (periodically re-pull a snapshot into a static `.jpg`) is still fundamentally stale between polls, just less stale. Rejected.

Root cause of today's frozen dashboard: `index.html` was deliberately switched from live MJPEG `<img>` tags to static-snapshot polling during the 2026-07-09 calibration session, per an inline comment, because MJPEG "didn't render reliably in the remote NoMachine Firefox session." That's a narrow, NoMachine-specific rendering quirk — not a reason to give up on live video for normal browser access (phone, laptop Chrome/Firefox/Safari all handle `multipart/x-mixed-replace` MJPEG fine in a plain `<img>` tag, which is exactly how `web_video_server`'s `/stream?topic=...` endpoint serves it).

Fix: point `index.html`'s `<img>` tags directly at `web_video_server` (`:8080/stream?topic=/viz/fused_bev` etc.) instead of local snapshot files. No refresh loop, no new script, no new tmux window — genuinely live, zero staleness, less moving parts than the snapshot approach. Keep the existing dark dashboard layout/styling, just change the image source and drop the polling JS.

If NoMachine/embedded-Firefox viewing turns out to still be needed later and MJPEG really doesn't render there, that's a separate, narrower problem to solve then (e.g. a NoMachine-specific fallback) — not a reason to default the primary dashboard to stale snapshots.

## Verification

- After each repo change: `git status` clean (only intentional untracked items — `bags/`, `src/realsense-ros/` — remain, both gitignored), `git log` shows the expected new commits, `git push` succeeds.
- After deploy script merge: run the merged script in restart mode against the live stack, confirm `tmux -L percept list-windows` shows the same 9 windows as before (plus the new dashboard-refresh window), and all topics/services (cameras, IMU, costmap, webui:8000, wvs:8080) are healthy exactly as verified earlier this session.
- After dashboard fix: open `:8090`, confirm every tile is a live, continuously-updating MJPEG stream (motion visible in real time — wave a hand in front of a camera and see it immediately), not a periodically-bumped static image.
- Vehicle stays live throughout: no step should require killing `avros-webui.service` or the running `percept` tmux session except the deliberate, brief restart in the deploy-script verification step above (confirm with operator immediately before doing that one).
