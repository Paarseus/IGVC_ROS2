# IGVC 2026 AutoNav — Autonomy Presentation (Preliminary Oral)

Parsa's autonomy section of the 10-min team preliminary oral. AutoNav single challenge.
Minimal, professional Beamer deck (5 slides) aligned to the submitted design report.
Real reference photos via `\includegraphics` with `\IfFileExists` fallbacks.
The opening "AutoNav challenge" course-overview slide was removed — the master team deck frames the course.

## Files
| File | What |
|---|---|
| `IGVC2026_Autonomy.tex` | Deck source — 5 slides, 16:9, custom minimal theme. Edit here. |
| `IGVC2026_Autonomy.pdf` | Compiled deck (present from this). |
| `SCRIPT.md` | Memorization script: 5-verb spine, per-slide words, timing, delivery cues. |
| `QA_CHEATSHEET.md` | Judge Q&A prep (arms the 100-pt "Response to questions") + image provenance. |
| `assets/` | Slide images (see provenance below). |

## Build
```
pdflatex IGVC2026_Autonomy.tex      # run twice for the slide-count footer
```
Word-for-word script is embedded as `\note{}` per frame. For a notes handout, add
`\setbeameroption{show notes}` after `\begin{document}` and recompile.

## Timing
~445 spoken words ≈ **3:25 @130 wpm** (3:11 rehearsed @140) — under the 3.5-min budget. See SCRIPT.md timing note to trim to ≤3:00.

## Slides → IGVC IV.5 preliminary-oral rubric (each 100 pts) + AutoNav course (§II.2)
1. **Perception** — LiDAR + camera → one costmap; white potholes separated from white lanes → **cat. 6**.
2. **Driving logic** — Navfn + MPPI + dual-EKF + recovery BT; handles switchbacks/dead-ends/traps/potholes/ramp → **cat. 7**.
3. **Validation and results** — Webots sim + measured hardware KPIs → **cat. 8 + 9**.
4. **Localization** — odom-frame control / GPS-advisory design → cat. 15 + arms cat. 14 (Q&A).
5. **Cyber security** — NIST risk assessment, three vulnerabilities, defense-in-depth → **cat. 11**.

## Image provenance / licenses
- `igvc_course.jpg` — IGVC 2023 AutoNav course, Wikimedia, **CC-BY-SA 4.0** (Nczem).
- `velodyne_vlp16.jpg` — Velodyne Puck (cropped), Wikimedia, **CC-BY-SA 4.0** (APJarvis).
- `zed_camera.jpg` — Stereolabs ZED X, manufacturer product image.
- `nav2_costmap.png`, `gps_nav.png` — Nav2 official docs (docs.nav2.org), BSD/Apache — **reference visualizations of the stack we run, not our own captures**.
- `segmentation.png` — kiwicampus `semantic_segmentation_layer` demo (the layer we deploy).
- `sim_nav.png` — our Webots `avros_sim` campus simulation (ours).

## Honesty guardrails (do not overclaim)
- GPS is **unaided SBAS, advisory only** — never claim RTK (IGVC §I.2 forbids positioning base stations).
- Odom-frame **local** control is deployed; full **map ≡ odom** migration is the committed next step, not done.
- The camera lane layer joined the live loop **2026-05-29**; the May-21 avoidance proof was **LiDAR-only**.
- The May-21 run **aborted by design** on the 45 s watchdog; a clean SUCCEEDED goal-reach is the next capture.
- The costmap/GPS screenshots are Nav2-doc references illustrating the stack — say so if asked.

## Rubric coverage (this 6-slide section)
Now covers, for the autonomy speaker: **6 Perception · 7 Driving Logic · 8 KPIs (target vs measured) · 9 Sim+results · 11 Cyber Security · 15 Understanding**, plus partial **2 (requirements lines on slides 2–3)**. Teammates still own **3 Mechanical · 4 Safety · 5 Electrical**, and the broader **2 / 10** narrative. Content is drawn from the design report `~/IGVC/AVL-IGVC2026/design_report/main.tex` so slides and report match.

## ⚠️ Report-vs-live divergences
See `QA_CHEATSHEET.md` — slides follow the report; the live robot differs on the perception pipeline (sooner25 vs HSV), MPPI rollout count (500 vs 2000), and camera count (1 vs 3). Reconcile or be ready to explain in Q&A.
