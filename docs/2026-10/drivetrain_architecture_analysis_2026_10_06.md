# Drivetrain architecture analysis — 2026-10-06

| | |
|---|---|
| **Question** | Should IGVC_ROS2 replace its tracked skid-steer chassis with a different drivetrain architecture to get more precise, easier-to-tune motion? |
| **Trigger** | Team parts list for swerve drive modules (AVL "Swerve drive module list," 9 off-the-shelf modules) plus a request for broad multi-architecture research, not limited to that list |
| **Method** | 5 research topics (`research/topics/D1`-`D5`), each independently mapped, researched, gap-checked, and verified by two independent reviewers (claims + sources) per `research/STANDARDS.md`. 654 findings, 192 sources, all five topics status **Verified** |
| **Scope** | Research-grounded comparison + recommendation. Does not replace field testing. |

## Bottom line

**Keep the tracked skid-steer chassis.** The evidence does not support that switching architecture would fix the problem that motivated this research (the skid-steer kinematic-correction and control-tuning burden). That burden is a physics property of *any* skid-steer vehicle — wheeled or tracked — not a tracks-specific defect, confirmed independently by multiple sources in both D2 and D5. The two architectures that would genuinely eliminate it (Ackermann, true holonomic swerve/mecanum) each fail a different hard requirement for this vehicle: Ackermann can't do the tight/zero-radius maneuvering IGVC's course and your own Nav2 history require, and swerve/mecanum have no outdoor-proven off-the-shelf path — every real outdoor-grass success story in the research used fully custom-built hardware, not catalog parts.

The highest-value next step is **not a hardware swap** — it's applying the specific, already-identified tuning fixes (right-track friction asymmetry, grass recalibration) and, if time allows, the real-time slip-estimation techniques D5 surfaced as the literature's actual answer to "beyond a fitted constant."

## Comparison

| | Swerve (D1) | Wheeled skid-steer (D2) | Ackermann (D3) | Omni/mecanum (D4) | Tracked — current (D5) |
|---|---|---|---|---|---|
| Formal mobility class | (δm=1, δs=2) — **not** fully holonomic, same mobility as a car | (δm=2, δs=0) | (δm=1, δs=1) | (δm=3, δs=0) — fully holonomic | Same as wheeled skid-steer, physically |
| Removes rotational scrub/slip? | In theory, yes — unproven in practice | **No** — same physics as tracks | Yes | In theory, yes, but roller-contact loss replaces it with a different precision problem | No |
| Off-the-shelf outdoor-rated parts? | **None** — no vendor publishes an IP rating | Yes (Husky/Jackal/Warthog-class, AndyMark kits) | Yes (AgileX Hunter 2.0, RC-car platforms, golf-cart conversions) | None validated for sustained grass/mud use | Yes (current platform + others) |
| ROS 2 / Nav2 support | **None** — `ros2_controllers` has no swerve controller | Mature — native `diff_drive_controller` | **Most mature** — `ackermann_msgs`, steering controllers, Regulated Pure Pursuit, MPPI `AckermannMotionModel` | New — `mecanum_drive_controller` added Dec 2024, Humble backport March 2025 | Mature — same diff-drive interface every tracked robot in the literature uses |
| Real IGVC outdoor precedent | 1 team (Oklahoma "Twistopher"), fully custom modules | Not found in this research as IGVC-specific, but Clearpath-class platforms are field-proven elsewhere | Used by multiple past teams; turning radius flagged as the explicit downside they accepted | Mixed: Oakland & Bluefield had grass problems with stock wheels; Buffalo succeeded only with custom large-roller wheels | Current platform; one other team's experience switching *away* from tracks is documented (see below) |
| Est. incremental cost (4-corner) | ~$900–2,600 for modules alone; $3,000–6,000+ all-in before any weatherproofing R&D | Moderate — reuses much of the existing control stack | $2,000–tens of thousands depending on platform class | Unknown — no validated outdoor product exists to price | Sunk — already built and tuned |
| Can it turn in place? | Yes | Yes | **No** (min. radius 1.94 m single-axle, 1.29 m dual-axle) | Yes | Yes |

## Why "switch architecture" doesn't address the actual problem

The premise behind this research was that the tracked chassis's need for an empirically-fitted correction factor (your `wheel_separation_multiplier: 1.19`) and the associated PID-tuning history were caused by *tracks specifically*. Two independent lines of evidence say otherwise:

- **D2** found published effective-width correction ratios for *wheeled* skid-steer platforms ranging from ~1.3–1.44× (Pioneer P3-AT) to ~1.33–1.54× (a 330 kg UGV) to **2.6–3.7×** (Clearpath Warthog, concrete vs. snow) — comparable to or *more* variable than your current 1.19×. The literature's explicit conclusion: the ratio is "surface- and vehicle-dependent, not a universal constant," for wheels and tracks alike.
- **D5** found a 2022 tracked-locomotion review stating plainly that for tracked vehicles "macroscopic skidding is unavoidable during steering" — not a tuning artifact — and an independent 2021 account of fielding a 1450 kg tracked UGV making the same point about any two-track differential system. Every tracked platform found in the research, from hobby kits to a 1450 kg NATO-research UGV, is still commanded through the same two-value (forward velocity, rotation velocity) interface your actuator already exposes — none needs or has a tracked-specific ROS 2 motion model. **Your current architecture shape is already what the rest of the field does.**

So a tracks→wheels swap would change maintenance characteristics (no track tension, no track-off risk, no debris packing) but would **not** remove the need for empirical correction or close the control-precision gap that motivated this research.

## Why the two architectures that *would* help aren't a good fit right now

**Ackermann** is the strongest *technically* — three independent studies in D3 found skid-steering draws roughly 2× the power of explicit/Ackermann-type steering and tracks arcs far more precisely (>100% drive-radius error for skid-steer vs. ~12% for Ackermann-like steering on the same rover testbed), and it has by far the best ROS 2/Nav2 support of anything researched. But it structurally can't turn in place, and your own Nav2 history already rejected a 2.31 m-turning-radius planner (Smac Hybrid-A*) as too wide for the course. One correction worth making to your own documentation: CLAUDE.md states IGVC lane width as "2-3 m," but D3 pulled the actual 2026 official rule — **lanes are 10-20 ft (3.05-6.1 m) wide with a minimum turning radius requirement of 5 ft (1.52 m), unchanged since at least 2020**. That's wider than CLAUDE.md assumes, but a single-axle Ackermann vehicle's 1.94 m minimum turning radius is still tight against the narrow end of that range, and — more importantly — your zero-turn-radius capability is what lets you dodge unmapped obstacles mid-lane, not just stay inside the lane lines. Worth updating that figure in CLAUDE.md regardless.

**Swerve and mecanum/omni** are the two that could, in theory, eliminate rotational scrub (swerve) or achieve full 3-DOF holonomic motion (mecanum). In practice, neither has a credible off-the-shelf outdoor path right now:
- No swerve vendor (REV, SDS, WCP, ThriftyBot) publishes any water/dust ingress rating — they're built for flat competition carpet. The one real fielded outdoor-grass IGVC swerve robot found (Oklahoma's "Twistopher") used **fully custom** modules with pneumatic wheels and weatherstripping specifically because stock modules weren't trusted outdoors.
- Mecanum's failure mode is directly on-point for your terrain: multiple rough-terrain robotics papers (including one that explicitly rejected mecanum/omni rollers for a man-portable UGV) identify the roller-to-roller discrete contact handoff as the literal mechanism by which small rollers lose traction on uneven, loose, or debris-laden ground. Real IGVC evidence splits exactly along this line: Oakland and Bluefield both reported stock-mecanum grass problems; Buffalo's "Big Blue" succeeded, but only after switching to **custom large-roller wheels** — again, a hardware R&D project, not a parts purchase. ROS 2's `mecanum_drive_controller` is also brand new (December 2024, Humble backport March 2025) with no field-maturity track record yet.

Both would mean committing to a custom-hardware-plus-from-scratch-control-stack project (swerve has zero native ROS 2/Nav2 controller support at all) chasing a precision benefit that no source in this research actually quantifies against skid-steer. That's a real engineering bet, not a drop-in upgrade — and not one I'd recommend taking on with a competition deadline ahead of you.

## When tracks genuinely are the wrong call (for awareness, not because it applies here)

D5 did surface one clear, real-world case for switching away from tracks: a team that ran a tracked platform one year found the tracks "frequently came off" under competition-grass stress, couldn't climb the required incline without slipping back, and capped their speed at 2.5-3 mph against a 5 mph target — so they moved to wheels. That's a **mechanical reliability and traction/speed** failure, not a control-precision one, and it's not a failure mode your own documented issues report (your known issues are tuning/precision, not track derailment or being unable to meet speed/grade targets). If that ever changes, wheeled skid-steer — not swerve or mecanum — is the lowest-risk switch, since it reuses your existing control architecture almost entirely and would let you drop the custom Teensy/`actuator_node` kinematic layer in favor of the ROS-native `diff_drive_controller`.

## Recommended actions, in priority order

1. **Right-track friction asymmetry** — `kS_left`/`kS_right` are both 0.18 V despite your own Phase 4 data showing the right track delivers only 89.4% of free speed vs. 97.5% on the left. Retune `kS_right` upward (try 0.20-0.22 V) to match delivery. *(Already identified; repeating here because it's the highest-value fix found across this whole effort.)*
2. **Recalibrate `wheel_separation_multiplier` on actual IGVC grass**, not the bench/smooth surface it was fit on. D2's literature shows this ratio can swing 2-3× between surfaces for a comparable platform — this is not a minor correction.
3. **If time permits beyond #1-#2**: D5 surfaced three technique families beyond a single fitted constant — terramechanics/multibody slip prediction, and (most relevant to your stack) real-time online slip/ICR estimators (EKF or sliding-mode observers) that feed directly into the same robot_localization architecture you already run. One cited design showed a plain adaptive controller left "a large trajectory tracking control error...not eliminated" during turns, which a real-time sliding-mode slip observer then corrected specifically on turns. This is the actual ceiling above a fixed multiplier, if #1-#2 turn out insufficient — and it fits your existing dual-EKF architecture rather than replacing it.
4. **Correct the lane-width figure in CLAUDE.md** from "2-3 m" to the verified official rule (10-20 ft / 3.05-6.1 m, min. turning radius 5 ft), so future planner/turning-radius decisions are made against the real constraint.
5. **Do not pursue swerve, mecanum, or Ackermann conversions** on the current timeline — none has a credible off-the-shelf outdoor path, and the one (Ackermann) with mature software support conflicts with your maneuverability requirements.

## Sources

Full findings, foundational references, and per-claim verification for each architecture:
- [D1 — Swerve drive modules](../research/topics/D1_swerve_drive/README.md) (153 findings, 43 sources)
- [D2 — Wheeled skid-steer / differential drive](../research/topics/D2_wheeled_skid_steer/README.md) (134 findings, 38 sources)
- [D3 — Ackermann / car-like steering](../research/topics/D3_ackermann_steering/README.md) (134 findings, 45 sources)
- [D4 — Omnidirectional (mecanum/omni) drive](../research/topics/D4_omnidirectional_drive/README.md) (136 findings, 39 sources)
- [D5 — Tracked drive alternatives and fixes](../research/topics/D5_tracked_alternatives/README.md) (97 findings, 27 sources)
