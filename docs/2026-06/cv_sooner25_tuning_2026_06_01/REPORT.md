# sooner25 Robustness Tuning — Different Angles / Lighting (2026-06-01)

**Goal:** make the `sooner25` lane pipeline work across different **angles + lighting**, not just one
scene. Method: capture fresh ZED frames + 10 diverse historical frames → drive the **real**
`Sooner25Pipeline` over the set under **8 angle/lighting augmentations** (perspective-tilt, rotation,
bright±, gamma, warm-WB, shadow) → sweep params + adaptive-V modes → adversarially verify. Model: Opus 4.8.

## Verdict

Neither a fixed nor a naive-adaptive threshold generalizes — they have **opposite** failure modes:

| Config | worst-case lane flood (8 augs) | failure |
|---|---|---|
| Fixed V=195 (old prod) | **87.7%** (bright_up) | floods → carpet of false-LETHAL cells when scene brightens |
| Raw adaptive p93 (no floor) | ~7.3% (shadow) | floods under shadow (ceiling drops, shadowed asphalt inverts to lane) |
| **Floored-adaptive p93 + S-drop 80** | **3.06%** (shadow) | **zero flood-class frames on any aug — ROBUST** |

Confirmed on the real updated pipeline: per-aug max lane% — none 1.64, bright_up 1.85, bright_down 0.62,
gamma_hi 2.32, warp_tilt 1.06, rot 2.29, wb_warm 1.55, shadow 3.06. Lines traced + barrels/tents/grass
rejected; degrades to **thin/blank (safe)** under extremes, never floods. Overlays: `recommended_overlays/`.

## The change (`pipelines/sooner25.py`, param-gated, default OFF)

1. **Floored-adaptive V-ceiling:** `V_ceiling = clamp(p93(asphalt-band V), vfloor=185, vcap=255)`.
   The **floor (185)** keeps shadowed asphalt classed as asphalt (kills the adaptive shadow-flood);
   the percentile lets bright scenes raise the ceiling (keeps the bright-flood protection a fixed
   ceiling lacks). `sooner25_adaptive: 'none'` → original fixed-threshold behaviour.
2. **Post-detection high-S drop (`sooner25_post_sdrop: 80`):** the real colored-clutter rejection.
   sooner25's *inverted* scheme can't reject clutter via inRange's S-upper (asphalt is already low-S),
   and `sky_roi` misses clutter that drops below the horizon at off-angles. Dropping detected lane px
   with blurred S>80 removes the orange-barrel base etc. while keeping the low-S white line (line px
   are byte-identical with S-drop on/off — precision-only).

## Active config (`perception.yaml`)

```yaml
sooner25_upper: [255, 255, 185]   # V is now the FALLBACK (used when adaptive='none')
sooner25_adaptive: 'p93'
sooner25_band: [0.40, 1.0]
sooner25_vfloor: 185
sooner25_vcap: 255
sooner25_post_sdrop: 80
```

**Revert** to old behaviour: `sooner25_adaptive: 'none'` + `sooner25_post_sdrop: 0` (one `ros2 param set`
each, live). Best paired with the ZED exposure lock (commit `27daf71`) so p93 is steady frame-to-frame.

## Honest limits (verified)

- **3.06% shadow residual** is a *bounded* false positive — on a frame with simultaneously lit (V~217)
  and shadowed (V~100) asphalt a single ceiling can't satisfy both; it's low-S gray so S-drop can't
  touch it. Below the flood line; costmap decay should clear transient shadows.
- **Extreme perspective-warp / very dark scenes → thin or blank** (the safe precision direction, not flood).
- **Scene set is all daytime asphalt** (white tape/line + orange barrels + tents). Augmentations stretch
  angle/lighting but can't manufacture novel surfaces (wet/night/grass-edge) — **live multi-angle
  validation on the Jetson with exposure locked still required.**

## Reproduce

```bash
# baseline-vs-recommended across all 8 augmentations (real pipeline):
python3 docs/cv_sooner25_tuning_2026_06_01/validate_recommended.py
# arbitrary config + augment, with overlays:
python3 docs/cv_sooner25_tuning_2026_06_01/sooner25_eval.py --frames <dir> --out /tmp/x \
    --adaptive p93 --supper 255 --augment shadow --json
```

Harness: `sooner25_eval.py` (drives the real pipeline + augmentations + metrics), `frames/all/`
(11-frame diverse set). Pre-existing unrelated test failure: `test_hsv_thresholds_unchanged` (an earlier
`sky_roi_poly` commit drifted its pinned hash — not touched here).
