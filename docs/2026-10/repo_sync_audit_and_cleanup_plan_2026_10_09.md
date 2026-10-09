# Repo sync audit + cleanup/organization plan (2026-10-09)

## 0. Why this exists

Across this session we kept finding the same shape of problem — `actuator_params.yaml` had three different, unsynced versions (GitHub, laptop, Jetson); `avros_lidar`'s "architecture change" turned out to be sitting uncommitted on an unmerged branch; STVL config exists in two parallel production files. This doc is the full audit behind that pattern: where GitHub, the Jetson, and this laptop have actually diverged, and a concrete plan to stop it from happening again — including consolidating the scattered documentation into one organized, dated place, per your request.

This is a plan, not yet executed. Nothing below has been moved, deleted, or committed except where explicitly marked `[DONE]`.

---

## 1. The big finding: the Jetson is running an unmerged, no-PR branch with uncommitted changes on top

This is the most important thing in this audit. The Jetson's `~/IGVC_ROS2` is **not on `main`** — it's on `lidar_testing`:

```
* lidar_testing   4ec5c89 [origin/lidar_testing] lidar: add avros_lidar package (preprocessor, voxel mapper, launch, README) and sim_lidar launch
```

- `lidar_testing` was pushed to GitHub (`origin/lidar_testing` exists) but **no PR was ever opened for it**. It's 1 commit ahead of `main`, sitting there unreviewed.
- On top of that commit, the Jetson's working tree has **18 modified files, 2 deleted-but-uncommitted files, and 6 new untracked files** relative to `lidar_testing`'s own last commit — including `nav2_params_igvc_autonav.yaml`, `actuator_params.yaml`, `zed_front.yaml`, and all of `avros_lidar`'s own source.
- Specifically: `voxel_grid.py` and `voxel_mapper_node.py` show as `D` (deleted, unstaged) — **this means the "avros_lidar dropped its custom voxel mapper and now feeds STVL directly" finding from earlier this session is not a committed design decision. It's an uncommitted local deletion sitting on an unmerged branch.** If the Jetson is reflashed, the disk fails, or anyone runs a hard git operation, that "decision" disappears with no record it ever happened.
- New untracked work on the Jetson that exists **nowhere else** — not on GitHub, not on the laptop: `src/avros_lidar/config/stvl_overrides.yaml`, `src/avros_lidar/launch/bench_costmap.launch.py`, `src/avros_perception/avros_perception/pipelines/white_ridge_detector.py`, `white_ridge.py`, `data_logger/`.

**This needs to be resolved before anything else in this plan** — it's real, uncommitted engineering work that currently exists in exactly one place.

---

## 2. Config files with three (sometimes four) unsynced copies

Same pattern as `actuator_params.yaml` (already partially fixed — PR #26 synced the gain block, but three fields still legitimately differ on purpose: `max_linear_mps`, `heading_hold_deadband`, `serial_port`). The same pattern exists elsewhere, confirmed modified-but-uncommitted on the Jetson:

| File | Status |
|---|---|
| `src/avros_bringup/config/nav2_params_igvc_autonav.yaml` | Modified on Jetson, not on GitHub. Also: `nav2_params_igvc_autonav.yaml.navtest.orig` and `actuator_params.yaml.navtest.orig` — leftover backup files from some sed/script run, untracked, pure cruft. |
| `src/avros_bringup/config/zed_front.yaml` | Modified on Jetson, not on GitHub — not investigated yet, unknown content. |
| `src/avros_bringup/config/navsat.yaml`, `ntrip_params.yaml` | Modified on **both** laptop and Jetson, independently, not on GitHub — two different uncommitted edits to the same files, never compared against each other. |
| `src/avros_perception/avros_perception/perception_node.py`, `pipelines/__init__.py` | Modified on **both** laptop and Jetson, not on GitHub — same risk: two independent uncommitted edits that have never been diffed against each other. |
| `firmware/teensy_diff_drive/CLAUDE.md` | Modified on both laptop and Jetson. This is the legacy v1 firmware's doc — given v2d is now final, worth checking whether this edit is even still relevant. |

**Risk:** every row with "modified on both" is a silent merge-conflict waiting to happen — nobody has ever diffed the laptop's edit against the Jetson's edit. We don't yet know if they agree, conflict, or one supersedes the other.

---

## 3. Git branch and PR sprawl

### Open PRs (GitHub)

| PR | Branch | Opened | Status |
|---|---|---|---|
| #26 | `actuator-fw26-final-gains-2026-10-09` | 2026-10-09 | Current, from this session, awaiting your review/merge. |
| #24 | `drive/fw26-motor-control` | 2026-09-29 | **Superseded by merged PR #25** — confirmed earlier this session (byte-identical firmware, reference docs ported forward). Should be closed. |
| #22 | `arassal:feature/perception-submodule` | 2026-09-15 | Teammate's PR (not yours) — "Auto Drive confirm gate, perception fusion + speedups, actuator/joystick fixes." Needs a look before any decision; may well be live, relevant work. |
| #1 | `feature/lane-detection` | 2026-04-13 | **6 months old.** HSV lane detection — per project memory, this approach has been superseded by multiple later perception iterations (sooner25/adaptive, canny, yolopv2). Almost certainly stale; worth confirming and closing. |

### Branches pushed to GitHub with no PR at all

- `lidar_testing` — see §1, this is live uncommitted-on-top work, highest priority.
- `imitation_learning_data_collection` — pushed, no PR, last activity "Added new column called 'vehicle_stopped'..." — unclear ownership/status, needs a quick check of who's using it.

### Stale local branches (laptop) — safe-to-delete candidates, already merged into `main`

These all show a merged PR with the identical commit already in `main`'s history: `firmware/slew-rate-ramp` (PR #23), `docs/ekf-mandow-inverse-investigation` (PR #10), `cleanup/remove-rtabmap-route-planner` (PR #2, remote-only), `refactor/xsens-submodule-to-vcs-import` (PR #3, remote-only). Keeping them around costs nothing functionally but adds noise to every `git branch` / `git branch -r` listing — low-priority cleanup.

### Leftover `.claude/worktrees/*` (laptop only)

Ten linked git worktrees sitting in `.claude/worktrees/`, each on its own throwaway branch (`worktree-claudemd-staleness-fix`, `worktree-control-stack-analysis`, `worktree-cv-costmap-recovery-fixes`, `worktree-decay-accel-bump`, `worktree-docs-costmap-rate`, `worktree-hsv-retune`, `worktree-ntrip-default-false`, `worktree-revert-obs-persistence`, `worktree-stvl-migration`, `worktree-w-test-plan`). Several commit messages describe work that's clearly already live in `main` per `CLAUDE.md` (e.g. the STVL migration, the ntrip-default-false rule). These are almost certainly dead scratch worktrees from past sessions that were never cleaned up after merging. Needs one pass to confirm each is actually superseded, then `EnterWorktree`/remove each and prune the branch.

### A dead git remote (laptop)

```
jetson  jetson:/home/dinosaur/IGVC (fetch/push)
```

Points at the **pre-rename path** (`~/IGVC`, renamed to `~/IGVC_ROS2` on 2026-09-25 per project memory). It happens to still resolve today only because something created a symlink `~/IGVC -> ~/IGVC_ROS2` on the Jetson very recently (2026-10-09) — likely incidental, not a deliberate fix. This remote has clearly not been the real sync path (everything this session went through GitHub + manual `scp`/`ssh`, never `git push jetson`). Either fix the path or remove the remote so it stops being a trap for a future `git push jetson main` that does something nobody expects.

---

## 4. Untracked work on the laptop — real deliverables mixed with scratch debris

The laptop's `git status` shows ~45 untracked paths. Triaged:

**Real, finished work that should be committed somewhere (not loose at repo root):**
- `research/` — `README.md`, `RESEARCH_PLAN.md`, `STANDARDS.md`, `TEMPLATE_TOPIC.md`, `tools/`, `topics/`, `evidence/desk_review_2026_09/`, `evidence/field_2026_09_27/` — this looks like a whole structured research methodology that's never been committed.
- ~20 dated analysis docs directly under `docs/` (e.g. `drivetrain_architecture_analysis_2026_10_06.md`, `cv_costmap_deep_analysis_2026_05_29.md`, `obstacle_stall_rca_2026_05_30.md`, `imu_usb_latency_2026_09_30.md`, `avros_lidar_layout_and_standard_2026_10_09.md`) — these are genuine deliverables from real debugging/research sessions, sitting uncommitted for months in some cases.
- `src/avros_perception/avros_perception/pipelines/canny.py` — **an actual untracked source file**, referenced by project memory ("Canny pipeline finding") as real, evaluated work (shipped off by default, but real).
- `scripts/kill_ros2.sh`, `scripts/launch_stack.sh`, `scripts/udev/`, `scripts/xsens/` — operational scripts referenced directly by `CLAUDE.md`'s own Known Issues table (the udev latency-timer rule, the Xsens restore script). These are load-bearing, not scratch, and currently untracked.
- `docs/phase0_field_test_2026_05_29_scripts/*.py` — four test scripts, same situation.

**Scratch/debris — candidates for deletion, not commit:**
- Loose images/video at repo root: `adaptive_verify.png`, `costmap_north.mp4`, `mask_v205.png`, `mdot1.png`, `overlay_*.png` (4 files), `path_mid07.png`, `path_southpa16.png`, `rgb_now.png`, `layer.txt`, `mdot_profile.json`, `adaptive_verify.py` — these look like one-off debugging artifacts from CV tuning sessions, dumped at the repo root instead of a scratch dir.
- `bags/`, `exp_frames/` — likely large binary test data; should probably be `.gitignore`d rather than committed regardless of their fate.
- `igvc_winners_research/`, `robojackets/`, `research_notes/` — unclear provenance; need a quick look at contents before deciding keep/delete.
- `.playwright-cli/`, `research/.playwright-cli/` — tool scratch state, should be gitignored.
- `reports/` — unclear contents, needs a look.

---

## 5. Unexplained Jetson state

- **`~/IGVC_webrtc`** — a completely separate directory on the Jetson, never referenced in `CLAUDE.md` or any doc this session has touched. Unknown purpose (name suggests a WebRTC video-streaming experiment). Needs a one-line "what is this and is it still needed" check before it's either documented or ignored.
- **`~/IGVC` → `~/IGVC_ROS2` symlink** — harmless, but undocumented. Worth either removing (now that the rename is 2 weeks old and everyone should be using the real path) or keeping deliberately and noting why in `CLAUDE.md`.

---

## 6. Proposed plan

### Phase 0 — stop the bleeding on the Jetson (highest priority, do first)
1. On the Jetson, **commit the real, working `avros_lidar`/STVL-integration changes** as an actual commit on `lidar_testing` (or a fresh branch off it) — including the `voxel_mapper`/`voxel_grid` deletion as a deliberate, message-explained commit, not a silent uncommitted deletion. This is the one piece of work that would be lost outright otherwise.
2. Diff the Jetson's uncommitted `navsat.yaml`/`ntrip_params.yaml`/`perception_node.py`/`pipelines/__init__.py` edits against the laptop's independent uncommitted edits to the *same* files. Reconcile or pick one before either gets committed, so we don't silently drop one person's fix.
3. Delete the `.navtest.orig` backup-cruft files.
4. Open a PR for `lidar_testing` (or fold it into a clean follow-up branch) so it's reviewed and either merged or explicitly rejected — not left as a silent fork of `main`.

### Phase 1 — GitHub housekeeping (cheap, no risk)
1. Close PR #24 (superseded).
2. Confirm and close PR #1 (stale HSV approach) — quick check first, since memory says superseded but hasn't been verified against the actual PR diff.
3. Check in with whoever owns PR #22 and `imitation_learning_data_collection` before touching either — teammate work, not yours to close unilaterally.
4. Delete the laptop's stale local branches that are already merged (`firmware/slew-rate-ramp`, `docs/ekf-mandow-inverse-investigation`, etc.) and prune the matching remote-tracking refs.
5. Clean up the ten `.claude/worktrees/*` entries once each is confirmed superseded.
6. Fix or remove the stale `jetson` git remote.

### Phase 2 — documentation consolidation (your request)
Move every dated analysis/research doc into one organized, dated structure instead of scattered across `docs/`, `research/`, `research_notes/`, and repo-root loose files. Proposed layout:

```
docs/
├── README.md                      # index: one line per doc, newest first
├── 2026-04/
├── 2026-05/
│   ├── cv_costmap_deep_analysis_2026_05_29.md
│   ├── obstacle_stall_rca_2026_05_30.md
│   ├── ... (every doc dated in May, grouped by month)
├── 2026-06/
├── 2026-09/
│   └── imu_usb_latency_2026_09_30.md
├── 2026-10/
│   ├── drivetrain_architecture_analysis_2026_10_06.md
│   ├── avros_lidar_layout_and_standard_2026_10_09.md
│   └── repo_sync_audit_and_cleanup_plan_2026_10_09.md   # this file
├── drive_tuning_2026_09_28/        # multi-file investigation dirs stay as their own folder, just moved under docs/
├── nav_test_2026_10_02/
└── reference/                      # NOT dated — living docs that get updated in place, not superseded (e.g. protocol docs, standards)
```

Rationale for month-folders over one flat dated-prefix list: there are already 25+ dated docs; a flat list is already hard to scan, and every filename already starts with a date, so grouping by month just makes the directory listing itself navigable without renaming every file. `research/` folds in as `docs/research/` (keeping its own internal structure — `RESEARCH_PLAN.md`, `STANDARDS.md`, `TEMPLATE_TOPIC.md`, `tools/`, `topics/`, `evidence/`) since it's a sibling concern to dated analysis docs, not a separate top-level thing.

This phase is pure `git mv` + one index file — zero code risk — but touches ~50 files across two machines, so it should happen **after** Phase 0 (don't move files out from under uncommitted Jetson work) and get its own single commit/PR for easy review.

### Phase 3 — untracked-file triage
1. Commit the load-bearing untracked files that `CLAUDE.md` already documents as real (`scripts/kill_ros2.sh`, `scripts/launch_stack.sh`, `scripts/udev/`, `scripts/xsens/`, `pipelines/canny.py`, the `phase0_field_test` scripts).
2. Delete the repo-root scratch images/video/text debris (or move to a local-only scratch dir if anyone wants to keep them).
3. Decide `bags/`, `exp_frames/` fate — almost certainly `.gitignore`, not commit (binary test data).
4. One-line look at `igvc_winners_research/`, `robojackets/`, `research_notes/`, `reports/` to decide keep/merge-into-docs/delete.

### Phase 4 — prevent recurrence
- Add a short section to `CLAUDE.md` (or a `CONTRIBUTING.md`) stating explicitly: config files under `avros_bringup/config/` are committed as the single source of truth; a machine-local override (test speed caps, serial ports) gets called out in a commit message or left as an explicitly-documented local diff, never silently drifted.
- Consider a lightweight periodic check (even just a reminder, not automation) comparing Jetson `git status`/`git diff` against `main` before any multi-week gap, so divergence this size doesn't accumulate again.

---

## 7. Final decisions (2026-10-09)

Everything in this plan has now been either executed or explicitly decided. Status:

**Executed:**
- PR #26 — SparkMAX FW26 gains (`kFF`/`kP`/`kS_left`/`kS_right`) committed.
- PR #27 — this doc + full `docs/` reorganization into `docs/YYYY-MM/` + `docs/reference/` (this PR).
- PR #28 — `research/` methodology, `CLAUDE.md`-documented scripts, `canny.py`, `.gitignore` additions (`bags/`, `data_logger/`, `exp_frames/`, `.playwright-cli/`, `robojackets/`).
- PR #29 — laptop-side config/doc drift reconciled (`navsat.yaml`, `perception_node.py`, `pipelines/__init__.py`, `perception.yaml`, both `CLAUDE.md` files, stale doc-path fixes), plus `ntrip_params.yaml` (real credentials, committed with explicit owner sign-off).
- PR #24 and PR #1 closed (superseded/stale).
- 6 stale local branches + all 10 `.claude/worktrees/*` deleted on the laptop (all confirmed fully merged first; two had real content — a `research/` sub-dir and an orphaned doc — rescued before deletion).
- The two credential-bearing `deployed_snapshot/.../ntrip_params.yaml` files (inside the rescued research dirs) deleted outright.
- Repo-root scratch debris (loose PNGs/videos/one-off scripts) deleted; superseded `research_notes/`/`reports/` drafts deleted (confirmed duplicate of the properly-sourced `research/topics/D2_wheeled_skid_steer` and `C1_motor_velocity_control`); `igvc_winners_research/` merged into `docs/reference/winners_research/`.
- The `jetson` git remote fixed — now points at `jetson:/home/dinosaur/IGVC_ROS2` (was the pre-rename `/home/dinosaur/IGVC`). Run by the user directly, per the "never touch git config" rule.

**Explicitly decided to leave alone (not a gap — a deliberate call):**
- `lidar_testing` branch, `stash@{0}`, `stash@{1}` on the Jetson — active WIP (Changwe was confirmed logged into the Jetson live while this audit ran). **Leave alone.**
- `data_logger/` (5.7GB on the Jetson) — real recorded data tied to `imitation_learning_data_collection`. Already `.gitignore`d (the actual problem — git choking on 5.7GB — is fixed); the data itself stays in place. **Leave alone.**
- PR #22 (`arassal:feature/perception-submodule`) and the `imitation_learning_data_collection` branch — teammate-owned work. **Leave alone.**

Nothing here is unresolved; every item has an owner's decision behind it now.
