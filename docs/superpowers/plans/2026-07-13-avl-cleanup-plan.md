# AVL / dinosaur Codebase Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Dedupe and organize the AVL/dinosaur codebase across the laptop and the Jetson, get real uncommitted work safely into git, and fix the live dashboard to show genuinely live camera/costmap video — all without disrupting the currently-running vehicle stack (cameras, IMU, costmap, phone-joystick/actuator bridge).

**Architecture:** This is an ops/cleanup pass across two machines, not a software feature — there is no single codebase to add tests to. Each task instead has concrete shell verification commands with exact expected output. Tasks are ordered so read-only/independent work happens first, git history is secured before any deletion, and the one step that touches the live running stack is isolated last, gated on explicit operator go-ahead.

**Tech Stack:** bash, git, systemd, tmux, ROS2 Humble (informational only — no ROS code is written).

## Global Constraints

- Jetson access: `ssh dinosaur` (pre-configured in `~/.ssh/config` on the laptop, routes via `tailscale nc` using an already-trusted key — do not use password auth, it's account-locked).
- NEVER touch `/home/dinosaur/carla-nav2-avl/carla-nav2-avl-jchy05/` — a teammate's active clone on a different branch (`feature/jchy05`).
- NEVER touch `Desktop/steve`, `avl_carla`, `rhylon/joystick`, `AVL_ObjectDetection`, `AVL_ObjectDetection_local` — explicitly out of scope.
- The vehicle stack (tmux session `percept` on socket `-L percept`, and `avros-webui.service`) must keep running throughout every task except Task 14, which requires explicit operator confirmation immediately before executing.
- Every git-destructive step (`rm`, `git checkout --`) must be preceded by confirming the file is either already committed elsewhere (git history covers it) or explicitly approved for discard — never delete uncommitted work silently.

---

### Task 1: Laptop — delete redundant repo clones

**Files:**
- Delete: `/home/alexander/Desktop/IGVC_Nav2_SegFormer`
- Delete: `/home/alexander/selfdrive_carla_ue5_ros2`
- Delete: `/home/alexander/carla-nav2-backup`

**Interfaces:** None — fully independent of every other task in this plan.

- [ ] **Step 1: Re-verify these are safe to delete (no uncommitted work, no unpushed commits)**

Run:
```bash
for d in /home/alexander/Desktop/IGVC_Nav2_SegFormer /home/alexander/selfdrive_carla_ue5_ros2 /home/alexander/carla-nav2-backup; do
  echo "=== $d ==="
  git -C "$d" status --short
  git -C "$d" log --oneline '@{u}..HEAD' 2>&1
done
```
Expected: empty `git status --short` (or only expected untracked build artifacts) and empty unpushed-commit list for all three. If any shows real uncommitted/unpushed content, STOP and report it — do not delete.

- [ ] **Step 2: Delete the three directories**

```bash
rm -rf /home/alexander/Desktop/IGVC_Nav2_SegFormer /home/alexander/selfdrive_carla_ue5_ros2 /home/alexander/carla-nav2-backup
```

- [ ] **Step 3: Verify deletion**

```bash
for d in /home/alexander/Desktop/IGVC_Nav2_SegFormer /home/alexander/selfdrive_carla_ue5_ros2 /home/alexander/carla-nav2-backup; do
  [ -e "$d" ] && echo "STILL EXISTS: $d" || echo "gone: $d"
done
```
Expected: `gone:` for all three.

---

### Task 2: Laptop — archive stale scratch directories

**Files:**
- Move: `/home/alexander/kbot_stack` → `/home/alexander/archive/kbot_stack`
- Move: `/home/alexander/avl_ros2` → `/home/alexander/archive/avl_ros2`
- Move: `/home/alexander/nav2_perception_ws` → `/home/alexander/archive/nav2_perception_ws`

**Interfaces:** None — independent of every other task.

- [ ] **Step 1: Create the archive directory and move the three**

```bash
mkdir -p /home/alexander/archive
mv /home/alexander/kbot_stack /home/alexander/avl_ros2 /home/alexander/nav2_perception_ws /home/alexander/archive/
```

- [ ] **Step 2: Verify**

```bash
ls /home/alexander/archive/
[ -e /home/alexander/kbot_stack ] || [ -e /home/alexander/avl_ros2 ] || [ -e /home/alexander/nav2_perception_ws ] && echo "FAIL: still at old location" || echo "OK: moved"
```
Expected: `kbot_stack  avl_ros2  nav2_perception_ws` listed under archive, then `OK: moved`.

---

### Task 3: Jetson — clean `.bak_*` files in `carla-nav2-avl` repo

**Files (on dinosaur):**
- Delete: `/home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/config/perception_dinosaur.yaml.bak_20260709_155958`
- Delete: `/home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/config/perception_dinosaur.yaml.bak_20260709_160337`
- Delete: `/home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/config/perception_dinosaur.yaml.bak_lidar`
- Delete: `/home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/config/perception_dinosaur.yaml.bak_precalib`
- Delete: `/home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/tools/viz_node.py.bak_20260709_155958`
- Delete: `/home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/tools/viz_node.py.bak_20260709_160337`

**Interfaces:** None — this repo's good state (`perception_dinosaur.yaml`, `viz_node.py`) is already committed and pushed at `66652b0`; git history is the rollback path, not these files.

- [ ] **Step 1: Confirm the good state is actually pushed (rollback safety net exists)**

Run:
```bash
ssh dinosaur "cd /home/dinosaur/carla-nav2-avl/ros2_ws && git log --oneline @{u}..HEAD && git status --short"
```
Expected: empty unpushed-commit list (already pushed), and `git status --short` shows only the `.bak_*` files and `../carla-nav2-avl-jchy05/` as untracked (nothing else).

- [ ] **Step 2: Delete the 6 `.bak_*` files**

```bash
ssh dinosaur "cd /home/dinosaur/carla-nav2-avl/ros2_ws && rm -f \
  src/perception_costmap/config/perception_dinosaur.yaml.bak_20260709_155958 \
  src/perception_costmap/config/perception_dinosaur.yaml.bak_20260709_160337 \
  src/perception_costmap/config/perception_dinosaur.yaml.bak_lidar \
  src/perception_costmap/config/perception_dinosaur.yaml.bak_precalib \
  src/perception_costmap/tools/viz_node.py.bak_20260709_155958 \
  src/perception_costmap/tools/viz_node.py.bak_20260709_160337"
```

- [ ] **Step 3: Verify clean status (only the teammate's directory remains untracked)**

```bash
ssh dinosaur "cd /home/dinosaur/carla-nav2-avl/ros2_ws && git status --short"
```
Expected: only `?? ../carla-nav2-avl-jchy05/` remains.

---

### Task 4: Jetson — push already-committed IGVC doc commits

**Files (on dinosaur):** none modified — this is a push-only step.

**Interfaces:** Must run before Task 5 (keeps history linear: push what's already committed before adding new commits on top).

- [ ] **Step 1: Confirm the 3 unpushed commits are what we expect**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git log --oneline @{u}..HEAD"
```
Expected exactly these 3, oldest last:
```
0f4745d docs: revise dashboard fix to real MJPEG streaming, not snapshot polling
55f73d9 docs: cleanup design spec for laptop+jetson AVL codebase cleanup
28ee28c Fix actuator Teensy serial (board swap 18639150->19915500) + webui websocket deadlock
```

- [ ] **Step 2: Push**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git push origin main"
```
Expected: `main -> main` fast-forward push, no errors.

- [ ] **Step 3: Verify nothing left unpushed**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git log --oneline @{u}..HEAD"
```
Expected: empty output.

---

### Task 5: Jetson — commit the validated nav2/costmap tuning diff

**Files (on dinosaur):**
- Modify (commit as-is): `/home/dinosaur/IGVC/src/avros_bringup/config/nav2_params_humble.yaml`
- Modify (commit as-is): `/home/dinosaur/IGVC/src/avros_bringup/config/navigate_igvc_autonav_humble.xml`

**Interfaces:** Depends on Task 4 (must run after that push so this new commit builds on the pushed history, not behind it).

- [ ] **Step 1: Re-confirm this is the exact diff expected (values only, not a surprise superset)**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git diff src/avros_bringup/config/nav2_params_humble.yaml src/avros_bringup/config/navigate_igvc_autonav_humble.xml"
```
Expected: exactly 3 value changes — `vx_max: 1.5` → `0.7`, `decay_acceleration: 2.0` → `0.5` (appears twice, local + global costmap), `RateController hz="1.0"` → `"3.0"`. If the diff shows anything beyond these, STOP and report — do not commit blindly.

- [ ] **Step 2: Commit**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git add src/avros_bringup/config/nav2_params_humble.yaml src/avros_bringup/config/navigate_igvc_autonav_humble.xml && git commit -m 'nav2: validated tune - vx_max 1.5->0.7, decay_acceleration 2.0->0.5, replan hz 1.0->3.0'"
```

- [ ] **Step 3: Verify**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git status --short src/avros_bringup/config/nav2_params_humble.yaml src/avros_bringup/config/navigate_igvc_autonav_humble.xml && git log -1 --oneline"
```
Expected: no output from `git status --short` for those two files (clean), and the log shows the new commit.

---

### Task 6: Jetson — revert ZED camera resolution to SVGA

**Files (on dinosaur):**
- Restore to HEAD: `/home/dinosaur/IGVC/src/avros_bringup/config/zed_front.yaml`
- Restore to HEAD: `/home/dinosaur/IGVC/src/avros_bringup/config/zed_left.yaml`
- Restore to HEAD: `/home/dinosaur/IGVC/src/avros_bringup/config/zed_right.yaml`

**Interfaces:** Independent of Task 5 (touches different files), but keep after it so IGVC's commit sequence stays linear per the spec's ordering.

- [ ] **Step 1: Confirm current diff is only the resolution field (nothing else riding along)**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git diff src/avros_bringup/config/zed_front.yaml src/avros_bringup/config/zed_left.yaml src/avros_bringup/config/zed_right.yaml"
```
Expected: exactly 3 lines changed, each `grab_resolution: 'HD1080'` → back to `'SVGA'` (currently showing the reverse, HD1080, which is what we're discarding).

- [ ] **Step 2: Discard the diff (restore HEAD's SVGA setting)**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git checkout -- src/avros_bringup/config/zed_front.yaml src/avros_bringup/config/zed_left.yaml src/avros_bringup/config/zed_right.yaml"
```

- [ ] **Step 3: Verify — clean, and SVGA confirmed present**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git status --short src/avros_bringup/config/zed_front.yaml src/avros_bringup/config/zed_left.yaml src/avros_bringup/config/zed_right.yaml && grep grab_resolution src/avros_bringup/config/zed_front.yaml src/avros_bringup/config/zed_left.yaml src/avros_bringup/config/zed_right.yaml"
```
Expected: no `git status --short` output, and all 3 greps show `grab_resolution: 'SVGA'`.

**Note:** this only affects camera behavior the next time those ZED nodes restart — the currently-running cameras are unaffected until then.

---

### Task 7: Jetson — commit the new scripts, drop the superseded `.bak`

**Files (on dinosaur):**
- Add: `/home/dinosaur/IGVC/scripts/kill_ros2.sh`
- Add: `/home/dinosaur/IGVC/scripts/launch_stack.sh`
- Delete (don't add): `/home/dinosaur/IGVC/scripts/launch_stack.sh.bak`

**Interfaces:** Independent of Tasks 5/6 (different files). Note: most of `scripts/` (analyze_M1.py, bag_preflight.py, deploy.sh, etc.) is **already tracked in git** — only these 2 files are new; do not `git add scripts/` wholesale.

- [ ] **Step 1: Confirm exactly which scripts/ files are untracked**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git status --short scripts/"
```
Expected exactly:
```
?? scripts/kill_ros2.sh
?? scripts/launch_stack.sh
?? scripts/launch_stack.sh.bak
```

- [ ] **Step 2: Confirm `launch_stack.sh.bak` is genuinely superseded (no unique content worth keeping)**

```bash
ssh dinosaur "diff /home/dinosaur/IGVC/scripts/launch_stack.sh /home/dinosaur/IGVC/scripts/launch_stack.sh.bak"
```
Expected: only the previously-confirmed 2-line diff (current version wraps ROS setup sourcing in `set +u`/`set -u` for unset-variable safety; the `.bak` lacks this fix). If the diff shows anything else, STOP and report.

- [ ] **Step 3: Delete the `.bak`, add the two real files, commit**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && rm -f scripts/launch_stack.sh.bak && git add scripts/kill_ros2.sh scripts/launch_stack.sh && git commit -m 'scripts: add kill_ros2.sh and launch_stack.sh (drop superseded .bak)'"
```

- [ ] **Step 4: Verify**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git status --short scripts/ && git log -1 --oneline"
```
Expected: empty `git status --short scripts/`, log shows the new commit.

---

### Task 8: Jetson — commit localization launch/config files

**Files (on dinosaur):**
- Add: `/home/dinosaur/IGVC/src/avros_bringup/localization.launch.py`
- Add: `/home/dinosaur/IGVC/src/avros_bringup/navsat.yaml`

**Interfaces:** Independent of every other task (different files, no shared state).

- [ ] **Step 1: Confirm these are the only 2 untracked files at this path level**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git status --short src/avros_bringup/localization.launch.py src/avros_bringup/navsat.yaml"
```
Expected:
```
?? src/avros_bringup/localization.launch.py
?? src/avros_bringup/navsat.yaml
```

- [ ] **Step 2: Add and commit**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git add src/avros_bringup/localization.launch.py src/avros_bringup/navsat.yaml && git commit -m 'avros_bringup: add localization launch file and navsat config'"
```

- [ ] **Step 3: Verify**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git status --short src/avros_bringup/localization.launch.py src/avros_bringup/navsat.yaml && git log -1 --oneline"
```
Expected: no status output, log shows the new commit.

---

### Task 9: Jetson — gitignore `bags/` and `src/realsense-ros/`

**Files (on dinosaur):**
- Modify: `/home/dinosaur/IGVC/.gitignore`

**Interfaces:** Independent of every other task.

- [ ] **Step 1: Append the two entries**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && cat >> .gitignore << 'EOF'

# Field-test rosbag recordings (hundreds of GB) — never commit.
bags/

# Third-party driver, its own upstream git repo — not part of this history.
src/realsense-ros/
EOF"
```

- [ ] **Step 2: Verify both paths are now ignored and disappear from untracked list**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git status --short | grep -E 'bags/|realsense-ros/' ; git check-ignore -v bags/ src/realsense-ros/"
```
Expected: the first command produces **no output** (both now ignored, not listed as untracked); the second confirms both paths match the new `.gitignore` lines.

- [ ] **Step 3: Commit the `.gitignore` change**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git add .gitignore && git commit -m 'gitignore: exclude bags/ (field-test recordings) and src/realsense-ros/ (upstream driver repo)'"
```

---

### Task 10: Jetson — delete remaining `.bak_*` files in IGVC

**Files (on dinosaur) — delete all of these:**
- `/home/dinosaur/IGVC/src/avros_bringup/config/actuator_params.yaml.bak_teensy18639150`
- `/home/dinosaur/IGVC/src/avros_bringup/config/nav2_params_humble.yaml.bak_cautious`
- `/home/dinosaur/IGVC/src/avros_bringup/config/navigate_igvc_autonav_humble.xml.bak2`
- `/home/dinosaur/IGVC/src/avros_bringup/config/navigate_igvc_autonav_humble.xml.bak_replanrate`
- `/home/dinosaur/IGVC/src/avros_bringup/config/navsat.yaml.bak`
- `/home/dinosaur/IGVC/src/avros_bringup/config/zed_front.yaml.bak_linetune`
- `/home/dinosaur/IGVC/src/avros_bringup/config/zed_front.yaml.bak_svga`
- `/home/dinosaur/IGVC/src/avros_bringup/config/zed_left.yaml.bak_svga`
- `/home/dinosaur/IGVC/src/avros_bringup/config/zed_right.yaml.bak_svga`
- `/home/dinosaur/IGVC/src/avros_bringup/launch/navigation.launch.py.bak_preIGVCarg`
- `/home/dinosaur/IGVC/src/avros_perception/config/perception.yaml.bak_linetune`
- `/home/dinosaur/IGVC/src/avros_webui/avros_webui/webui_node.py.bak_deadlock`

**Interfaces:** Must run **after** Tasks 5, 6, 7, 8 (those tasks' commits are what make each corresponding `.bak_*` file's "good state" safely recoverable from git history — don't delete backups before the thing they back up is actually committed).

- [ ] **Step 1: Re-list all remaining `.bak*` files under `src/` to confirm this is the complete, current set**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && find src -iname '*.bak*'"
```
Expected: exactly the 12 files listed above (order may vary). If anything differs, STOP and report — do not delete an unexpected file blindly.

- [ ] **Step 2: Confirm each corresponding tracked file is clean (committed, matches what the .bak would roll back to or better)**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git status --short src/avros_bringup/config/actuator_params.yaml src/avros_bringup/config/nav2_params_humble.yaml src/avros_bringup/config/navigate_igvc_autonav_humble.xml src/avros_bringup/config/navsat.yaml src/avros_bringup/config/zed_front.yaml src/avros_bringup/config/zed_left.yaml src/avros_bringup/config/zed_right.yaml src/avros_bringup/launch/navigation.launch.py src/avros_perception/config/perception.yaml src/avros_webui/avros_webui/webui_node.py"
```
Expected: no output (all clean/committed).

- [ ] **Step 3: Delete all 12 `.bak_*` files**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && rm -f \
  src/avros_bringup/config/actuator_params.yaml.bak_teensy18639150 \
  src/avros_bringup/config/nav2_params_humble.yaml.bak_cautious \
  src/avros_bringup/config/navigate_igvc_autonav_humble.xml.bak2 \
  src/avros_bringup/config/navigate_igvc_autonav_humble.xml.bak_replanrate \
  src/avros_bringup/config/navsat.yaml.bak \
  src/avros_bringup/config/zed_front.yaml.bak_linetune \
  src/avros_bringup/config/zed_front.yaml.bak_svga \
  src/avros_bringup/config/zed_left.yaml.bak_svga \
  src/avros_bringup/config/zed_right.yaml.bak_svga \
  src/avros_bringup/launch/navigation.launch.py.bak_preIGVCarg \
  src/avros_perception/config/perception.yaml.bak_linetune \
  src/avros_webui/avros_webui/webui_node.py.bak_deadlock"
```

- [ ] **Step 4: (Optional cleanliness) delete the same stray `.bak*` copies colcon copied into `install/`/`build/` — these are gitignored build artifacts, safe to remove, regenerate on next `colcon build`**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && find install build -iname '*.bak*' -delete 2>/dev/null; find install build -iname '*.bak*'"
```
Expected: second command produces no output (all removed).

- [ ] **Step 5: Verify no `.bak*` files remain under `src/`**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && find src -iname '*.bak*'"
```
Expected: no output.

---

### Task 11: Jetson — push all new IGVC commits

**Files:** none — push only.

**Interfaces:** Depends on Tasks 5, 6, 7, 8, 9 all being committed first.

- [ ] **Step 1: Review the full set of commits about to be pushed**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git log --oneline @{u}..HEAD"
```
Expected: the tuning commit (Task 5), the scripts commit (Task 7), the localization commit (Task 8), and the `.gitignore` commit (Task 9) — 4 commits (Task 6 was a discard, no new commit; Task 10's deletions may or may not have their own commit — see Step 2).

- [ ] **Step 2: Commit the Task 10 deletions if not already folded into another commit**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git status --short"
```
If this shows deleted `.bak_*` files as unstaged deletions (`D  ...`), commit them:
```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git add -u && git commit -m 'chore: remove stale .bak_* config/code backups (good state already committed)'"
```

- [ ] **Step 3: Push**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git push origin main"
```
Expected: fast-forward push, no errors.

- [ ] **Step 4: Final verification — working tree clean except intentional gitignored items**

```bash
ssh dinosaur "cd /home/dinosaur/IGVC && git status"
```
Expected: `nothing to commit, working tree clean` (bags/ and src/realsense-ros/ don't show — they're gitignored).

---

### Task 12: Jetson — merge `boot_stack.sh` + `full_stack_restart.sh`, update all references

**Files (on dinosaur, in `carla-nav2-avl` repo):**
- Modify: `/home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/full_stack_restart.sh` (add `--boot` flag support)
- Delete: `/home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/boot_stack.sh`
- Modify: `/home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/percept-stack.service` (ExecStart → `full_stack_restart.sh --boot`)
- Modify (installed copy): `/etc/systemd/system/percept-stack.service`
- Modify: `/home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/README.md` (drop the `boot_stack.sh` row, note the `--boot` flag instead)

**Interfaces:** `clean_camera_restart.sh` calls `full_stack_restart.sh` by relative path (`$(dirname "$0")/full_stack_restart.sh`) with no flag — this must keep working as the default (non-boot) mode after the merge, so no change needed there, but Step 5 verifies it explicitly.

**Important:** This task only edits files. It does **not** restart the running `percept-stack.service` or the live `percept` tmux session — that verification is Task 14, gated on explicit operator confirmation. `percept-stack.service` is `Type=oneshot, RemainAfterExit=yes`; it already ran once at this boot and won't re-invoke `ExecStart` again until next boot or an explicit restart, so editing its unit file now is safe.

- [ ] **Step 1: Write the merged `full_stack_restart.sh`**

```bash
ssh dinosaur "cat > /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/full_stack_restart.sh" << 'SCRIPT_EOF'
#!/bin/bash
# Rebuild the whole percept tmux -L percept session from scratch: sensors (TF+lidar),
# 3 ZED cameras (sequential, watchdog loops), fused costmap (TRT engine +
# TwinLiteNet), live viz node, web_video_server, dashboard http server.
#
# Usage:
#   full_stack_restart.sh          # manual restart: wrap tmux in a systemd
#                                   # --user --scope so it survives SSH teardown
#   full_stack_restart.sh --boot    # boot-time restart (invoked by
#                                   # percept-stack.service): skip the scope
#                                   # wrapper -- the service's own cgroup is
#                                   # the persistence
set -e

BOOT=false
if [ "$1" = "--boot" ]; then
  BOOT=true
fi

CFG=/home/dinosaur/IGVC/install/avros_bringup/share/avros_bringup/config
E="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file://$CFG/cyclonedds.xml"

tmux -L percept kill-session -t percept 2>/dev/null || true
pkill -f "zed_camera.launch.py" 2>/dev/null || true
pkill -f costmap_node 2>/dev/null || true
pkill -f "python3 .*viz_node\.py" 2>/dev/null || true
pkill -f web_video_server 2>/dev/null || true
pkill -f "python3 -m http\.server 8090" 2>/dev/null || true
sleep 3

echo "[1/6] sensors (TF + velodyne + xsens)..."
# Start the tmux server in a systemd user scope (linger is enabled): a scope
# lives until every process in it exits, so the daemonized tmux server
# survives SSH teardown. (A service unit reaps it the moment the client
# detaches — learned the hard way.) Dedicated socket (-L percept): on the
# default socket an operator's pre-existing tmux server would own the
# session in THEIR login cgroup, and the scope would protect nothing.
# --boot (percept-stack.service): the service cgroup is the persistence --
# plain tmux, no scope wrapper.
if [ "$BOOT" = false ]; then
  systemctl --user stop percept-tmux.scope 2>/dev/null || true
  systemctl --user reset-failed percept-tmux.scope 2>/dev/null || true
  systemd-run --user --scope --collect --unit percept-tmux \
    tmux -L percept new-session -d -s percept -n sensors \
    "bash -c \"source /home/dinosaur/IGVC/install/setup.bash && $E && ros2 launch avros_bringup sensors.launch.py 2>&1 | tee /tmp/sensors.log; exec bash\""
else
  tmux -L percept new-session -d -s percept -n sensors \
    "bash -c \"source /home/dinosaur/IGVC/install/setup.bash && $E && ros2 launch avros_bringup sensors.launch.py 2>&1 | tee /tmp/sensors.log; exec bash\""
fi
sleep 3
tmux -L percept has-session -t percept
sleep 15

echo "[1b/6] EKF (odom->base_link; velodyne + Nav2 need the odom frame)..."
tmux -L percept new-window -t percept -n ekf \
  "bash -c \"source /home/dinosaur/IGVC/install/setup.bash && $E && ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_filter_node_odom --params-file $CFG/ekf.yaml 2>&1 | tee /tmp/ekf.log; exec bash\""
sleep 5

zed_cmd() {
  local name=$1 serial=$2
  echo "source /home/dinosaur/IGVC/install/setup.bash && $E && \
while true; do \
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx \
camera_name:=$name serial_number:=$serial publish_tf:=false \
publish_urdf:=false ros_params_override_path:=$CFG/${name}.yaml \
2>&1 | tee -a /tmp/${name}.log; \
echo \"[watchdog] $name exited, relaunching in 5s\" | tee -a /tmp/${name}.log; \
sleep 5; done"
}

echo "[2/6] zed_front..."
tmux -L percept new-window -t percept -n zed_front "bash -c '$(zed_cmd zed_front 42569280)'"
sleep 40

echo "[3/6] zed_left..."
tmux -L percept new-window -t percept -n zed_left "bash -c '$(zed_cmd zed_left 49910017)'"
sleep 40

echo "[4/6] zed_right..."
tmux -L percept new-window -t percept -n zed_right "bash -c '$(zed_cmd zed_right 43779087)'"
sleep 40

echo "[5/6] costmap (3-cam fused, TRT engine + TwinLiteNet)..."
tmux -L percept new-window -t percept -n costmap \
  "bash -c \"source /opt/ros/humble/setup.bash && source /home/dinosaur/carla-nav2-avl/ros2_ws/install/setup.bash && $E && ros2 launch perception_costmap perception.launch.py config:=/home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/config/perception_dinosaur.yaml 2>&1 | tee /tmp/costmap.log; exec bash\""
sleep 15

echo "[6/6] viz node + streaming servers..."
tmux -L percept new-window -t percept -n viz \
  "bash /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/run_viz.sh 2>&1 | tee /tmp/viz_node.log"
tmux -L percept new-window -t percept -n wvs \
  "bash -c \"source /opt/ros/humble/setup.bash && $E && ros2 run web_video_server web_video_server --ros-args -p port:=8080 -p address:=0.0.0.0 2>&1 | tee /tmp/wvs.log; exec bash\""
tmux -L percept new-window -t percept -n www \
  "bash -c \"cd /home/dinosaur/live_dashboard && python3 -m http.server 8090 --bind 0.0.0.0; exec bash\""

echo "done: $(tmux -L percept list-windows -t percept -F '#W' | tr '\n' ' ')"
SCRIPT_EOF
ssh dinosaur "chmod +x /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/full_stack_restart.sh"
```

- [ ] **Step 2: Delete `boot_stack.sh`**

```bash
ssh dinosaur "rm -f /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/boot_stack.sh"
```

- [ ] **Step 3: Update both copies of `percept-stack.service` to use `--boot`**

```bash
ssh dinosaur "sed -i 's#ExecStart=/bin/bash /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/boot_stack.sh#ExecStart=/bin/bash /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/full_stack_restart.sh --boot#' \
  /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/percept-stack.service"
ssh dinosaur "sudo sed -i 's#ExecStart=/bin/bash /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/boot_stack.sh#ExecStart=/bin/bash /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/full_stack_restart.sh --boot#' \
  /etc/systemd/system/percept-stack.service"
ssh dinosaur "sudo systemctl daemon-reload"
```
Note: `daemon-reload` only refreshes systemd's cached parse of the unit file — it does **not** restart the running service.

- [ ] **Step 4: Update `README.md`**

```bash
ssh dinosaur "cd /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy && \
sed -i '/| `boot_stack.sh` | boot-time variant/d' README.md && \
sed -i 's#| `percept-stack.service` | systemd unit wrapping boot_stack.sh#| `percept-stack.service` | systemd unit wrapping `full_stack_restart.sh --boot`#' README.md"
```

- [ ] **Step 5: Verify no dangling references to `boot_stack.sh` remain, and `clean_camera_restart.sh`'s call still resolves**

```bash
ssh dinosaur "grep -rn 'boot_stack\.sh' /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/ /etc/systemd/system/percept-stack.service; echo 'grep exit code:' \$?"
echo "---"
ssh dinosaur "ls -la /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/full_stack_restart.sh && grep -n 'full_stack_restart.sh' /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/clean_camera_restart.sh"
```
Expected: the grep for `boot_stack.sh` finds nothing (exit code 1), and `full_stack_restart.sh` exists executable and is still referenced correctly by `clean_camera_restart.sh`.

- [ ] **Step 6: Commit and push**

```bash
ssh dinosaur "cd /home/dinosaur/carla-nav2-avl/ros2_ws && git add -A src/perception_costmap/deploy/ && git status --short src/perception_costmap/deploy/ && git commit -m 'deploy: merge boot_stack.sh into full_stack_restart.sh --boot flag' && git push origin feature/alexander"
```
Expected `git status --short` (staged, before the commit runs) to show: `M  .../full_stack_restart.sh`, `M  .../percept-stack.service`, `M  .../README.md`, `D  .../boot_stack.sh`.

- [ ] **Step 7: Verify pushed and clean**

```bash
ssh dinosaur "cd /home/dinosaur/carla-nav2-avl/ros2_ws && git status --short && git log --oneline @{u}..HEAD"
```
Expected: only `?? ../carla-nav2-avl-jchy05/` in status, empty unpushed list.

---

### Task 13: Jetson — restore live-MJPEG dashboard, prevent future drift

**Files (on dinosaur):**
- Overwrite: `/home/dinosaur/live_dashboard/index.html` (with the already-correct, git-tracked `deploy/live_dashboard.html`)

**Interfaces:** Independent of Task 12 (different concern), but do after it since both touch the `deploy/` directory — avoids any confusion about which edits landed when.

- [ ] **Step 1: Re-diff to confirm the repo copy is still the live-MJPEG version we want (nothing changed since we looked)**

```bash
ssh dinosaur "diff /home/dinosaur/live_dashboard/index.html /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/live_dashboard.html"
```
Expected: the previously-seen diff — served copy uses `data-src="fused_bev.jpg"` etc + a `setInterval` poll; repo copy uses `data-src="http://100.93.121.3:8080/stream?topic=...&type=mjpeg&quality=..."` with no interval poll (only error/visibility-triggered re-arm).

- [ ] **Step 2: Deploy the correct version**

```bash
ssh dinosaur "cp /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/live_dashboard.html /home/dinosaur/live_dashboard/index.html"
```

- [ ] **Step 3: Prevent this drift from happening again — symlink instead of a copy**

```bash
ssh dinosaur "rm /home/dinosaur/live_dashboard/index.html && ln -s /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/live_dashboard.html /home/dinosaur/live_dashboard/index.html && ls -la /home/dinosaur/live_dashboard/index.html"
```
Expected: `index.html -> /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/live_dashboard.html`. Editing the dashboard from now on means editing the tracked repo file directly — it's what's actually served.

- [ ] **Step 4: Verify the dashboard serves live streams, not stale snapshots**

```bash
curl -s http://100.93.121.3:8090/ | grep -o 'src="[^"]*"' | head -10
```
Expected: every `src="..."` (or `data-src="..."`, check the actual attribute used) points at `http://100.93.121.3:8080/stream?topic=...`, not a bare `.jpg` filename.

Then open `http://100.93.121.3:8090` in a real browser and confirm each of the 5 tiles (fused BEV, costmap, 3 cameras) visibly updates in real time — e.g. wave a hand in front of a camera and see the motion appear within a second, not a frozen frame.

- [ ] **Step 5: Delete the now-stale standalone snapshot `.jpg` files (no longer read by anything)**

```bash
ssh dinosaur "rm -f /home/dinosaur/live_dashboard/fused_bev.jpg /home/dinosaur/live_dashboard/costmap_render.jpg /home/dinosaur/live_dashboard/zed_front.jpg /home/dinosaur/live_dashboard/zed_left.jpg /home/dinosaur/live_dashboard/zed_right.jpg"
ls /home/dinosaur/live_dashboard/
```
Expected: only `index.html` (the symlink) remains.

---

### Task 14: Jetson — live verification of the merged deploy script (REQUIRES OPERATOR GO-AHEAD)

**Files:** none — this is a runtime verification step only.

**Interfaces:** Depends on Task 12 (the merged script must exist) and Task 13 (dashboard fix should already be live so this restart doesn't undo it — the symlink means it can't be undone by a restart anyway).

**⚠️ STOP: this is the one step in the whole plan that touches the live running vehicle stack. Do not run Step 2 without asking the operator to confirm the area around the vehicle is clear and it's an OK time to briefly bounce the perception/camera stack (motors/joystick are on a separate service, `avros-webui.service`, and are NOT restarted by this step — only cameras/costmap/EKF/viz/streaming go down and back up, for roughly 3-4 minutes).**

- [ ] **Step 1: Ask the operator for explicit go-ahead before proceeding**

Confirm out loud/in chat: "About to restart the percept tmux stack (cameras, EKF, costmap, viz, streaming — NOT the joystick/motors) to verify the merged deploy script. This takes ~3-4 minutes with cameras offline during that window. OK to proceed?" Wait for an explicit yes before Step 2.

- [ ] **Step 2: Run the merged script in its default (non-boot) mode**

```bash
ssh dinosaur "bash /home/dinosaur/carla-nav2-avl/ros2_ws/src/perception_costmap/deploy/full_stack_restart.sh"
```
Expected: output ending in `done: sensors ekf zed_front zed_left zed_right costmap viz wvs www` (same 9 windows as before the merge).

- [ ] **Step 3: Verify all 9 tmux windows are present and the systemd user scope wrapping worked**

```bash
ssh dinosaur "tmux -L percept list-windows -t percept"
ssh dinosaur "systemctl --user status percept-tmux.scope --no-pager | head -5"
```
Expected: 9 windows (`sensors ekf zed_front zed_left zed_right costmap viz wvs www`), and the scope shows `active (running)`.

- [ ] **Step 4: Verify cameras, IMU, and costmap are actually publishing again (not just processes existing)**

```bash
ssh dinosaur "bash -lc 'source /home/dinosaur/IGVC/install/setup.bash && timeout 5 ros2 topic hz /zed_front/zed_node/rgb/color/rect/image 2>&1 | tail -3'"
ssh dinosaur "bash -lc 'source /home/dinosaur/IGVC/install/setup.bash && timeout 5 ros2 topic hz /imu/data 2>&1 | tail -3'"
```
Expected: both show a nonzero `average rate`, matching the ~7.9Hz (front camera) and ~99Hz (IMU) seen earlier this session.

- [ ] **Step 5: Confirm the joystick/actuator service was never touched**

```bash
ssh dinosaur "systemctl status avros-webui.service --no-pager | head -5"
```
Expected: `active (running)`, with an uptime spanning well before this restart (proving it was never affected).

- [ ] **Step 6: Confirm the dashboard is still live after the restart**

Open `http://100.93.121.3:8090` again and re-confirm live motion on all 5 tiles (the symlink from Task 13 means the fix persists automatically, but verify end-to-end anyway since `web_video_server` itself just restarted).

---

## Final Summary Checklist

- [ ] Laptop: 3 duplicate repo clones deleted, 3 stale dirs archived
- [ ] `carla-nav2-avl`: `.bak_*` cleaned, deploy scripts merged, pushed
- [ ] `IGVC`: all real work committed (tuning, scripts, localization files), SVGA restored, `.bak_*` cleaned, `bags/`+`realsense-ros/` gitignored, pushed
- [ ] Dashboard: genuinely live (verified by eye), symlinked to prevent future drift
- [ ] Vehicle stack: verified healthy after the one deliberate restart, joystick/actuator confirmed untouched throughout
