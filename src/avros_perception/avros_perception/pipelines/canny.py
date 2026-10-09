"""
Color-gated auto-Canny + IPM-geometry lane pipeline.

WHY THIS EXISTS / WHY A NAIVE CANNY WAS ALREADY REJECTED
--------------------------------------------------------
The production ``adaptive`` pipeline's docstring lists, under "Deliberately NOT
included", a Sobel/Canny gradient OR: "measured to triple px (280->923) and push
the exposure CV 0.031->0.273 by importing asphalt aggregate/crack texture —
directly defeats keep-asphalt-clean. There is no IPM-warp + sliding-window stage
downstream to reject that noise (unlike Udacity-style pipelines)."

This pipeline adds exactly the downstream geometric rejection that the rejected
attempt lacked, and — crucially — uses Canny as an AND CONSTRAINT, never an OR.
The candidate generator is adaptive.py's exposure-invariant HLS-L local-contrast
white-paint core (the ONLY recall source); auto-Canny edges are AND'd in so
asphalt texture cannot flood the mask (a crack has a gradient edge but is NOT
brighter-than-its-low-sat-neighborhood, so it dies in the WHITE term).

STAGE PIPELINE (each stage names the adversary it kills)
--------------------------------------------------------
  S0  input guard + resolution anchor (sc = h/300.0 scales every pixel-literal)
  S1  one BGR2HLS; L (idx 1) = contrast channel, S (idx 2) = colour gate.
      Optional CLAHE-on-L (off; clip>3 washes faint paint).
  S2  Gaussian de-speckle on L (odd-coerced — even kernel RAISES, stalls
      _on_synced and kills ALL perception).
  S3a WHITE-PAINT CANDIDATE — cv2.adaptiveThreshold(GAUSSIAN, BINARY, C<0) on L.
      Exposure-invariant BY CONSTRUCTION: T = local_mean + |C|, so any monotone
      exposure transform lifts pixel AND neighbourhood together. (Adversary 4:
      shadows / AE limit cycle.)
  S3b LOW-SAT WHITE GATE — drop sat > canny_max_sat. (Adversary 1: orange/brown/
      green barrels/cones/tents — their bands are high-S.)
  S4  AUTO-CANNY EDGE — median-scaled lo/hi (rides the AE swing; fixed 50/150
      collapsed IoU to 0.27 under a gamma shift).
  S5  AND-FUSION — candidate = white AND dilate(edges). THE fix for the rejected
      naive Canny (NEVER OR). (Adversary 2: cracks die in the WHITE term.)
  S6  SHAPE FILTER (ALWAYS ON) — connected-component elongation/fill/area,
      resolution-relative. A thin tape ribbon passes; a white-barrel FACE / 2-ft
      solid POTHOLE disk is a COMPACT high-fill blob and is REJECTED with zero
      depth. (Adversary 3: WHITE obstacles, axis 1.)
  S7  IPM BIRD'S-EYE WARP (optional, cached M/Minv) — warp the BINARY at
      INTER_NEAREST (NEVER raw RGB — re-imports the AE cycle / bilinear-sinks
      faint lines). Off-plane obstacle faces radially smear into top-flared fans.
  S8  GEOMETRIC LINE-FIT — column-histogram + per-side sliding-window 2nd-order
      polyfit (curve-native; default) OR HoughLinesP (guarded alt). Only long
      smooth near-vertical ground lines survive. (Adversary 2 + 3, geometric.)
  S8b UNWARP back to input HxW (kiwicampus contract REQUIRES the mask at input
      resolution; BEV is a different frame).
  S9  DEPTH GROUND-PLANE REFINEMENT (GUARDED, DEAD until the node feeds depth —
      perception_node.py:441 calls run(bgr) with NO depth today). KEEP-on-NaN.
  S10 SKY/HORIZON ROI zeroed LAST (background tents/trees/people are the
      brightest pixels in frame — non-negotiable, AFTER all detection).
  S11 OUTPUT — mono8 class-ID mask + uint8 confidence, both input HxW. Single
      class_id_lane, kept LETHAL downstream (repo policy — no gradient).

EXPOSURE INVARIANCE (AE kept ON — do NOT lock exposure)
-------------------------------------------------------
Invariant by construction end-to-end: (a) the S3a adaptiveThreshold core is a
local-contrast test; (b) S4 Canny thresholds are MEDIAN-DERIVED (ride the AE
swing); (c) S3b is a relative-channel saturation ratio; (d) S7/S8 geometry runs
on the already-BINARIZED BEV (we deliberately do NOT warp raw grayscale and
re-threshold), so the geometric stage adds ZERO new absolute-brightness
dependence.

OUTPUT CONTRACT (identical to adaptive.py / sooner25.py)
--------------------------------------------------------
A mono8 class-ID mask (everything detected -> class_id_lane) + a uint8
confidence plane, both the same HxW as the input, with the sky/horizon ROI
zeroed LAST. Barrels/potholes are DROPPED — LiDAR/STVL owns obstacle avoidance.

KNOWN RISKS (see the design doc / module-level comments inline)
---------------------------------------------------------------
  * Depth (S9) is NOT wired today — the only PRINCIPLED white-obstacle kill is
    dead code on every live frame. A thin white pole / paint-on-a-barrel can
    still slip through S6+S8 as a phantom LETHAL lane until a node-level height
    test on the organized PointCloud2 lands. Flagged, not shipped-as-done.
  * canny_ipm_src is an UNCALIBRATED, eyeballed normalized quad (no intrinsics)
    — the single biggest correctness risk. Re-pick per camera mount on a real
    straight-lane frame. Mitigation: canny_use_ipm=false degrades to the proven
    S6 shape-filter baseline; the fitter is curve-native (not straight Hough).
  * Ramp / two_d_mode pitch trap: a static homography assumes flat ground. Set
    canny_use_ipm=false on the ramp deck (S6 has no flat-ground assumption).
  * Subtractive stack: S5/S6/S8/S9 are all REMOVAL stages — an over-aggressive
    geometry stage could thin a faint/worn line below detection (a missed outer
    line = E-stop end of run). Mitigated by the kill-switches (canny_edge_and,
    canny_use_ipm), the window/Hough MISS falling back to the pre-warp mask, and
    KEEP-on-NaN-depth. Bias every tune toward DETECTION.
"""

import cv2
import numpy as np

from avros_perception.pipelines.base import Pipeline, PipelineResult


class CannyPipeline(Pipeline):
    """Color-gated auto-Canny + cached-homography IPM + sliding-window polyfit.

    Output contract is identical to AdaptivePipeline: a mono8 class-ID mask
    (everything detected -> class_id_lane) + a uint8 confidence plane, same HxW
    as the input, with the sky/horizon ROI zeroed last.
    """

    # NOTE (DRY): _reshape_poly + _roi_polygon_px are byte-identical to
    # adaptive.py / sooner25.py / hsv.py. The standard fix is to hoist them into
    # the Pipeline base class; that refactor spans several files, so — exactly as
    # adaptive.py documents — they are copied VERBATIM here. The sky/horizon ROI
    # is non-negotiable and is zeroed LAST (see run()).

    def __init__(self, params, logger=None):
        super().__init__(params, logger)
        # Cached perspective transforms (S7). Computed once in warmup() and
        # lazily RECOMPUTED only when (h, w) or the ipm params change — NEVER
        # per frame (getPerspectiveTransform per frame would starve the 20 Hz
        # MPPI loop on the Jetson). _ipm_key fingerprints the inputs that M/Minv
        # depend on so the lazy-recompute fires exactly when they change.
        self._M = None
        self._Minv = None
        self._ipm_key = None

    # ------------------------------------------------------------------ ROI ---
    def _reshape_poly(self, seq):
        """Flat [x0,y0,x1,y1,...] normalized list -> [(x0,y0),(x1,y1),...]."""
        if not seq:
            return []
        if len(seq) % 2 != 0:
            raise ValueError(
                f'sky_roi_poly flat list must have even length, got {len(seq)}'
            )
        return [(float(seq[i]), float(seq[i + 1])) for i in range(0, len(seq), 2)]

    def _roi_polygon_px(self, h, w):
        """sky_roi_poly (normalized) -> int32 pixel polygon, or None if empty.

        Default mirrors the field-tuned perception.yaml value (top 40% zeroed).
        """
        raw_poly = self.params.get(
            'sky_roi_poly',
            [0.0, 0.0, 1.0, 0.0, 1.0, 0.40, 0.0, 0.40],
        )
        poly_norm = self._reshape_poly(raw_poly)
        if not poly_norm:
            return None
        return np.array(
            [(int(x * (w - 1)), int(y * (h - 1))) for x, y in poly_norm],
            dtype=np.int32,
        )

    # --------------------------------------------------------------- IPM cache -
    def warmup(self):
        """Pre-compute M/Minv if (h,w) is already known. Default: lazy.

        The published image size is not known until the first frame, so M/Minv
        are most often built lazily in run() via _ensure_ipm(). warmup() is kept
        for the base-class contract and to pre-warm if a caller sets known dims.
        """
        return None

    def _ensure_ipm(self, h, w):
        """Build/cache M (image->BEV) and Minv (BEV->image), recompute on change.

        getPerspectiveTransform REQUIRES np.float32 (4,2) quads or it returns
        garbage; warpPerspective dsize is (width, height) NOT numpy .shape — a
        classic silent crop/blank bug. Both are guarded here.
        """
        src_raw = list(self.params.get(
            'canny_ipm_src',
            [0.42, 0.42, 0.58, 0.42, 1.0, 1.0, 0.0, 1.0],
        ))
        bev_w = int(self.params.get('canny_bev_w', 300))
        bev_h = int(self.params.get('canny_bev_h', 400))

        # Defensive: a raw `ros2 param set canny_ipm_src [...]` could be the
        # wrong length. 8 floats == 4 (x,y) points. Fall back to the default on
        # a malformed quad rather than raising (would stall _on_synced).
        if len(src_raw) != 8:
            src_raw = [0.42, 0.42, 0.58, 0.42, 1.0, 1.0, 0.0, 1.0]

        key = (h, w, tuple(src_raw), bev_w, bev_h)
        if key == self._ipm_key and self._M is not None:
            return self._M, self._Minv, bev_w, bev_h

        # NORMALIZED flat [x0,y0,...] (TL, TR, BR, BL) -> pixel float32 (4,2).
        src = np.float32([
            [src_raw[0] * w, src_raw[1] * h],
            [src_raw[2] * w, src_raw[3] * h],
            [src_raw[4] * w, src_raw[5] * h],
            [src_raw[6] * w, src_raw[7] * h],
        ])
        dst = np.float32([
            [0, 0],
            [bev_w - 1, 0],
            [bev_w - 1, bev_h - 1],
            [0, bev_h - 1],
        ])
        self._M = cv2.getPerspectiveTransform(src, dst)
        self._Minv = cv2.getPerspectiveTransform(dst, src)
        self._ipm_key = key
        return self._M, self._Minv, bev_w, bev_h

    # ---------------------------------------------------------------- fitters -
    def _fit_window(self, bev, bev_w, bev_h, sc):
        """Per-side column-histogram + sliding-window 2nd-order polyfit (S8).

        Curve-native (the research-preferred fitter for the primarily-sinusoidal
        course; straight Hough under-fits curves). Per-side INDEPENDENT — a
        wide/curved track may show only ONE boundary, so never require both. On
        a side with too few inliers, that side is DROPPED only; if NEITHER side
        fits, the caller keeps the pre-fit BEV (a Hough/window miss must NOT
        blank a real line — asymmetric cost: a missed outer line = E-stop).
        """
        col_k = float(self.params.get('canny_col_k', 3.0))
        col_floor = float(self.params.get('canny_col_floor', 8))
        nwin = int(self.params.get('canny_nwindows', 10))
        min_inliers = int(self.params.get('canny_min_inliers', 150))
        draw_w = max(3, int(round(float(self.params.get('canny_line_draw_w', 7)) * sc)))

        binbev = (bev > 0)
        colsum = binbev.sum(axis=0).astype(np.float64)
        med = float(np.median(colsum))
        thr = max(col_k * med, col_floor)
        keep = colsum >= thr
        # Widen the kept-column band by the resolution-relative half-window so a
        # narrow line peak isn't a single isolated column (np.convolve 'same').
        span = 2 * int(round(8 * sc)) + 1
        keep = np.convolve(keep.astype(np.uint8), np.ones(span, dtype=np.uint8), 'same') > 0

        nonzero = binbev.nonzero()
        nonzeroy = nonzero[0]
        nonzerox = nonzero[1]
        if nonzerox.size == 0:
            return None  # empty BEV -> caller keeps pre-fit mask

        mid = bev_w // 2
        margin = max(8, bev_w // 12)
        minpix = max(4, bev_w // 24)
        win_h = max(1, bev_h // max(1, nwin))

        bev_fit = np.zeros((bev_h, bev_w), dtype=np.uint8)
        any_fit = False

        for side in ('L', 'R'):
            if side == 'L':
                masked = np.where(keep[:mid], colsum[:mid], 0)
                if masked.max() <= 0:
                    continue
                base = int(np.argmax(masked))
            else:
                masked = np.where(keep[mid:], colsum[mid:], 0)
                if masked.max() <= 0:
                    continue
                base = int(np.argmax(masked)) + mid

            cur = base
            lane_idx = []
            for win in range(nwin):
                y_hi = bev_h - win * win_h
                y_lo = y_hi - win_h
                x_lo = cur - margin
                x_hi = cur + margin
                good = ((nonzeroy >= y_lo) & (nonzeroy < y_hi)
                        & (nonzerox >= x_lo) & (nonzerox < x_hi)).nonzero()[0]
                lane_idx.append(good)
                if good.size > minpix:
                    cur = int(np.mean(nonzerox[good]))

            if not lane_idx:
                continue
            inl = np.concatenate(lane_idx) if len(lane_idx) > 1 else lane_idx[0]
            if inl.size < min_inliers:
                continue  # drop THIS side only (don't hallucinate over a gap)

            ys = nonzeroy[inl].astype(np.float64)
            xs = nonzerox[inl].astype(np.float64)
            try:
                fit = np.polyfit(ys, xs, 2)
            except Exception:
                continue
            ploty = np.arange(0, bev_h, dtype=np.float64)
            plotx = fit[0] * ploty ** 2 + fit[1] * ploty + fit[2]
            pts = np.stack([plotx, ploty], axis=1).astype(np.int32)
            # Clip to BEV and rasterize as a thick polyline (warped 3-in width).
            valid = (pts[:, 0] >= 0) & (pts[:, 0] < bev_w)
            pts = pts[valid]
            if pts.shape[0] >= 2:
                cv2.polylines(bev_fit, [pts.reshape(-1, 1, 2)], False, 255, draw_w)
                any_fit = True

        if not any_fit:
            return None
        return bev_fit

    def _fit_hough(self, bev, bev_w, bev_h, sc):
        """HoughLinesP + near-vertical angle gate (S8 guarded alt).

        Straight-segment fitter. Keep BEV segments within ±canny_angle_band of
        vertical (AGC) — rejects guardrail/curb/crack lines and obstacle-smear
        fans that are not near-vertical ground lines. Guards `lines is None`
        (HoughLinesP returns None, not []). On no survivors the caller keeps the
        pre-fit BEV.
        """
        rho = float(self.params.get('canny_hough_rho', 2))
        thresh = int(self.params.get('canny_hough_thresh', 40))
        angle_band = float(self.params.get('canny_angle_band', 35))
        draw_w = max(3, int(round(float(self.params.get('canny_line_draw_w', 7)) * sc)))
        min_len = int(round(40 * sc))
        max_gap = int(round(100 * sc))

        lines = cv2.HoughLinesP(
            bev, rho, np.pi / 180.0, thresh,
            minLineLength=max(1, min_len), maxLineGap=max(1, max_gap),
        )
        if lines is None:
            return None

        bev_fit = np.zeros((bev_h, bev_w), dtype=np.uint8)
        any_seg = False
        for ln in lines:
            x1, y1, x2, y2 = ln[0]
            ang = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1)))  # 0..180
            # near-vertical == |angle - 90| < band
            if abs(ang - 90.0) < angle_band:
                cv2.line(bev_fit, (x1, y1), (x2, y2), 255, draw_w)
                any_seg = True
        if not any_seg:
            return None
        return bev_fit

    # -------------------------------------------------------------------- run -
    def run(self, bgr, depth=None):
        # ---- S0  INPUT GUARD + RESOLUTION ANCHOR -----------------------------
        if bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError(f'CannyPipeline expects HxWx3 BGR; got {bgr.shape}')
        h, w = bgr.shape[:2]
        sc = h / 300.0  # every pixel-literal below scales with the real res

        # Live-tunable params (re-read EVERY frame; ros2 param set is live).
        # In-code defaults match the field-tuned perception.yaml.
        C = float(self.params.get('canny_C', -8.0))
        block = int(self.params.get('canny_block_size', 21))
        blur = int(self.params.get('canny_blur', 9))
        max_sat = int(self.params.get('canny_max_sat', 70))
        use_clahe = bool(self.params.get('canny_use_clahe', False))
        clahe_clip = float(self.params.get('canny_clahe_clip', 2.0))
        sigma = float(self.params.get('canny_sigma', 0.33))
        aperture = int(self.params.get('canny_aperture', 3))
        # edge_and DEFAULTS FALSE (2026-06-01 field re-tune). Measured: the hard
        # white-AND-dilate(Canny) is the single worst operator for BOTH exposure
        # invariance and faint paint — under a gamma 0.6..1.4 AE swing it drove
        # the gamma-sweep CV to 0.78 with TOTAL DROPOUT to 0 px at the dark end
        # (line gradient falls below the median-scaled Canny lo/hi -> empty AND),
        # vs CV 0.20 / no dropout with the AND OFF; and faint worn paint (contrast
        # x0.7) collapsed 364->0 px WITH the AND but stayed 1156 px WITHOUT it. The
        # crack/asphalt rejection the AND was meant to provide is fully covered by
        # the S6 shape gate below (elong + low-fill), verified 0 false asphalt px
        # on every real frame. AND remains a kill-switch (=True) for a rougher
        # surface, but a missed outer line = E-stop, so it is OFF by default.
        edge_and = bool(self.params.get('canny_edge_and', False))
        min_area = int(self.params.get('canny_min_area', 80))
        # elong_min 2.5 / fill_max 0.10 (2026-06-01 field re-tune). Real IGVC tape
        # components measure elong 2.93..3.33, fill 0.035..0.056 across all scene
        # frames + a gamma 0.6..1.4 sweep; white obstacle FACES/disks measure elong
        # 1.10..1.38 and a thin white POLE measures fill 0.13..0.14. elong>=2.5 (a
        # margin below the 2.93 real-line floor) rejects every compact obstacle;
        # fill<=0.10 rejects the elongated pole. See _accept_shape() — the old
        # (elong OR longaxis>=45) clause let a 67x87 barrel face / 55x55 disk pass
        # on bbox size alone (elong~1) and leaked 234..1532 px into the LETHAL mask.
        elong_min = float(self.params.get('canny_elong_min', 2.5))
        fill_max = float(self.params.get('canny_fill_max', 0.10))
        longaxis_min = int(self.params.get('canny_longaxis_min', 0))
        longaxis_base = float(self.params.get('canny_longaxis_min_base', 45))
        # use_ipm DEFAULTS FALSE (2026-06-01 field re-tune). The IPM/sliding-window
        # stage is UNCALIBRATED (eyeballed canny_ipm_src) and on these 480x300
        # frames it NEVER fires on a real line (the near-field tape sits above the
        # trapezoid top), yet it DOES fire on a white-obstacle's vertical edges,
        # extrapolating them into a frame-spanning PHANTOM lane: with IPM ON a
        # barrel face leaked 1532 px (vs 0 with the S6 gate + IPM OFF), a bucket
        # 841 px, a cone 581 px. IPM ON was measured ~3-5x WORSE on every obstacle.
        # Re-calibrate canny_ipm_src per camera mount and verify IPM-on != IPM-off
        # on a real straight-lane frame before flipping this back on.
        use_ipm = bool(self.params.get('canny_use_ipm', False))
        fit_mode = str(self.params.get('canny_fit_mode', 'window'))
        close_v = int(self.params.get('canny_close_v', 15))
        use_depth = bool(self.params.get('canny_use_depth', True))
        height_tol = float(self.params.get('canny_height_tol', 0.15))
        class_id_lane = int(self.params.get('class_id_lane', 1))

        # MANDATORY odd-coercion + lower-bound clamps. cv2.GaussianBlur RAISES on
        # an even kernel; cv2.adaptiveThreshold RAISES on even/<=1 blockSize.
        # A raw `ros2 param set canny_blur 8` would otherwise stall _on_synced
        # and kill ALL perception. IntegerRange(step=2) only constrains sliders.
        bk = blur if blur % 2 == 1 else blur + 1
        bk = max(bk, 1)
        bs = block if block % 2 == 1 else block + 1
        bs = max(bs, 3)

        # ---- S1  COLORSPACE (one BGR2HLS, slice both) ------------------------
        # HLS order is H, L, S -> L is idx 1 (NOT HSV-V), S is idx 2.
        hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)
        chan = hls[:, :, 1]   # L — lighting-stable contrast channel
        sat = hls[:, :, 2]    # S — colour gate (S3b)

        # Optional CLAHE shadow normalizer on L BEFORE blur (per-tile LOCAL
        # contrast == still exposure-relative). OFF by default: clip>3 amplifies
        # aggregate texture and can wash faint worn paint (A/B only).
        if use_clahe:
            clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(8, 8))
            chan = clahe.apply(chan)

        # ---- S2  DENOISE -----------------------------------------------------
        # Low-passes 1-3 px asphalt-aggregate spikes that out-contrast faint
        # paint and would otherwise speckle past min_area + skew the auto-Canny
        # median. Matches adaptive_blur.
        chan_blur = cv2.GaussianBlur(chan, (bk, bk), 0)

        # ---- S3a  WHITE-PAINT CANDIDATE (the ONLY recall source) -------------
        # adaptiveThreshold(GAUSSIAN, BINARY, C<0): T = local_mean + |C| => fires
        # only pixels BRIGHTER than their neighbourhood = white paint. Invariant
        # by construction to the AE 141<->167 limit cycle (line is only +5 HLS-L
        # over asphalt GLOBALLY — no fixed/Otsu level separates them; local
        # contrast is the only viable signal).
        white = cv2.adaptiveThreshold(
            chan_blur, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            bs, C,
        )

        # ---- S3b  LOW-SAT WHITE GATE (Adversary 1) ---------------------------
        # White paint + asphalt are near-grey (S median ~11 / ~6); orange barrel
        # band ~28, tent edge ~79+. Zeroing high-S pixels in the white BASE here
        # means their bright gradient edges never reach the S5 AND. NOTE: green
        # trees/grass and WHITE obstacles pass this gate by definition — those
        # are S6/S8/S9's job, not this gate's. 255 disables.
        if max_sat < 255:
            white[sat > max_sat] = 0

        # ---- S4  AUTO-CANNY EDGE (median-scaled — exposure invariant) --------
        # lo/hi RIDE the AE swing (~0.67v / 1.33v) so Canny does not flicker.
        # Fixed 50/150 collapsed final-mask IoU to 0.27 under a gamma shift;
        # median-scaled ~doubled it to 0.58-0.65. chan_blur is CV_8UC1 as
        # cv2.Canny requires. apertureSize MUST be in {3,5,7} or cv2 RAISES.
        v = float(np.median(chan_blur))
        lo = int(max(0, (1.0 - sigma) * v))
        hi = int(min(255, (1.0 + sigma) * v))
        ak = aperture if aperture in (3, 5, 7) else 3
        edges = cv2.Canny(chan_blur, lo, hi, apertureSize=ak)

        # ---- S5  AND-FUSION (THE fix for the rejected naive Canny) -----------
        # Edges are an AND CONSTRAINT, never OR'd into the mask. A crack has a
        # gradient edge but is NOT brighter-than-neighbourhood-low-sat, so it
        # dies in the WHITE term. This single AND prevents the documented
        # 280->923 px / CV 0.031->0.273 texture explosion (the old pipeline did
        # color OR gradient, exactly backwards). The 3x3 dilate lets a
        # line-interior pixel adjacent to the gradient edge survive + widens
        # tolerance so a faint-but-real edge still ANDs.
        if edge_and:
            ed = cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            candidate = cv2.bitwise_and(white, ed)
        else:
            # Pure-adaptive kill-switch == the proven adaptive pipeline; the safe
            # fallback if the AND ever over-thins a worn line (missed line =
            # E-stop end of run).
            candidate = white

        # ---- S6  SHAPE FILTER (ALWAYS ON — white-obstacle + crack-speck kill) -
        # Resolution-relative, color-independent, no IPM/depth needed. THE primary
        # white-obstacle defense now that depth (S9) is unwired and IPM (S7/S8) is
        # off by default. Acceptance requires BOTH an elongated bbox AND a low fill:
        #
        #   accept  <=>  elong >= elong_min (2.5)  AND  fill <= fill_max (0.10)
        #
        # Measured component signatures (real frames + gamma 0.6..1.4 sweep):
        #   real IGVC tape : elong 2.93..3.33, fill 0.035..0.056  -> ACCEPT
        #   barrel FACE    : elong 1.16..1.30, fill 0.03..0.05    -> REJECT (elong)
        #   cone           : elong 1.38,       fill 0.045         -> REJECT (elong)
        #   bucket/disk    : elong 1.10,       fill 0.08          -> REJECT (elong)
        #   thin white POLE: elong 3.18..6.18, fill 0.13..0.14    -> REJECT (fill)
        #
        # CRITICAL FIX (2026-06-01): the OLD `(elong >= elong_min OR long_side >=
        # la)` clause auto-accepted any large bbox regardless of roundness, so a
        # 67x87 barrel face / 55x55 solid disk (elong~1) PASSED on size alone and
        # leaked 234..1532 px into the LETHAL lane mask (IGVC obstacles include
        # WHITE — this is a real DQ/false-stop hazard). The longaxis OR-escape is
        # REMOVED: acceptance is now strictly elong AND low-fill. The longaxis_min
        # params are retained (no-op by default) so the in-code/yaml param sets
        # stay in sync, but they no longer gate acceptance. `la` is computed only
        # to keep the param live-tunable for a future re-enable; set it to require
        # a minimum long_side by re-adding `and long_side >= la` if a camera ever
        # needs a hard length floor (it would reject short dashed stubs, so OFF).
        n, labels, stats, _ = cv2.connectedComponentsWithStats(candidate, 8)
        la = max(longaxis_min, int(round(longaxis_base * sc)))  # noqa: F841 (kept for param parity)
        ma = int(round(min_area))
        pre = np.zeros((h, w), dtype=np.uint8)
        for i in range(1, n):  # skip label 0 (background)
            A = int(stats[i, cv2.CC_STAT_AREA])
            if A < ma:
                continue
            W = int(stats[i, cv2.CC_STAT_WIDTH])
            H = int(stats[i, cv2.CC_STAT_HEIGHT])
            long_side = max(W, H)
            short_side = max(1, min(W, H))
            elong = long_side / short_side
            fill = A / float(max(1, W * H))
            # BOTH conditions mandatory: elongated AND sparse. A round/compact
            # white obstacle fails the elong test; a solid-filled blob fails the
            # fill test. No bbox-size OR-escape — that was the leak.
            if (elong >= elong_min) and (fill <= fill_max):
                pre[labels == i] = class_id_lane

        # If IPM is off, the shape-filtered `pre` IS the detection mask (the
        # proven baseline). Skip S7/S8/S8b.
        if not use_ipm:
            mask = pre.copy()
        else:
            # ---- S7  IPM BIRD'S-EYE WARP (cached M/Minv) ---------------------
            # Warp the BINARY at INTER_NEAREST (NEVER raw RGB — re-imports the AE
            # cycle and bilinear-sinks faint lines below threshold). dsize is
            # (width, height) NOT numpy shape. Off-plane obstacle faces radially
            # SMEAR into top-flared fans; true ground lines stay narrow columns.
            M, Minv, bev_w, bev_h = self._ensure_ipm(h, w)
            bev = cv2.warpPerspective(
                pre, M, (bev_w, bev_h),
                flags=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT, borderValue=0,
            )

            # ---- S8  GEOMETRIC LINE-FIT ----------------------------------
            # Optional dashed-gap close FIRST (continuity, no texture import):
            # THIN (1,N) vertical kernel bridges dash gaps ALONG the now-vertical
            # line, never across horizontal cracks — safe ONLY in BEV (gaps
            # colinear). IGVC outer boundaries may be dashed, so default ON.
            if close_v > 1:
                bev = cv2.morphologyEx(
                    bev, cv2.MORPH_CLOSE,
                    cv2.getStructuringElement(cv2.MORPH_RECT, (1, close_v)),
                )

            if fit_mode == 'hough':
                bev_fit = self._fit_hough(bev, bev_w, bev_h, sc)
            else:  # 'window' (default — curve-native)
                bev_fit = self._fit_window(bev, bev_w, bev_h, sc)

            # A Hough/window MISS must NOT blank a real line (asymmetric cost):
            # fall back to the pre-fit BEV.
            if bev_fit is None:
                bev_fit = bev

            # ---- S8b  UNWARP TO IMAGE FRAME (contract REQUIRES input HxW) ----
            warped_back = cv2.warpPerspective(
                bev_fit, Minv, (w, h), flags=cv2.INTER_NEAREST,
            )
            mask = np.where(warped_back > 0, class_id_lane, 0).astype(np.uint8)

            # ASYMMETRIC-COST SAFETY (load-bearing): if the ENTIRE IPM/line-fit
            # stack produced an empty image-frame mask while the shape-validated
            # `pre` had real detections, fall back to `pre`. This catches the
            # UNCALIBRATED-quad failure mode where the detected line sits ABOVE
            # the (eyeballed) src trapezoid's top edge — e.g. a near-horizon line
            # at y<0.42h — so warpPerspective maps it OFF the BEV and `bev` is
            # already 0 (the inner "bev_fit=bev" fallback then can't recover it).
            # A missed outer line = E-stop end of run, so a bad/mis-aimed
            # homography must DEGRADE to the proven S6 baseline, never blank it.
            # (Re-pick canny_ipm_src per camera mount to make IPM actually fire;
            # until then this guard keeps recall == the shape-filter baseline.)
            if not mask.any() and pre.any():
                mask = pre.copy()

        # ---- S9  DEPTH GROUND-PLANE REFINEMENT (GUARDED — DEAD today) --------
        # perception_node.py:441 calls run(bgr) with NO depth, so this is a
        # silent no-op on every live frame. When a valid registered depth/cloud
        # IS fed, drop lane pixels sitting > height_tol above expected ground
        # (raised obstacle face). WIDE band (>=0.15 m) survives the 15% ramp /
        # two_d_mode pitch trap. NaN/invalid depth is KEPT (stereo dropout on a
        # thin far pole/line must never silently delete a real line). The REAL
        # home is a node-level height test on the organized PointCloud2 — see
        # module docstring risks.
        if depth is not None and use_depth:
            try:
                z = np.asarray(depth, dtype=np.float32)
                if z.shape[:2] == mask.shape[:2]:
                    rows = np.arange(h, dtype=np.float32)[:, None]
                    # Per-row expected ground depth from the fixed 15deg tilt is
                    # camera-specific; without intrinsics we use a monotone
                    # near->far model so the relative height test is meaningful.
                    # (Placeholder ground model — refined when the node wires a
                    # real extrinsic / RANSAC plane. KEEP-on-NaN below is the
                    # load-bearing safety.)
                    z_ground = np.broadcast_to(
                        np.nanmedian(z) if np.isfinite(z).any() else 0.0,
                        z.shape[:2],
                    ).astype(np.float32)
                    raised = (z_ground - z) > height_tol
                    raised &= np.isfinite(z)  # KEEP NaN/invalid pixels
                    mask[raised] = 0
            except Exception:
                # Depth refinement must NEVER crash _on_synced.
                pass

        # ---- S10  SKY/HORIZON ROI ZEROED LAST (non-negotiable) ---------------
        # Background tents/trees/people are the brightest pixels in frame
        # (rows 20-119 at 300-tall). Zeroed AFTER all detection.
        poly = self._roi_polygon_px(h, w)
        if poly is not None:
            cv2.fillPoly(mask, [poly], 0)

        # ---- S11  OUTPUT -----------------------------------------------------
        confidence = np.where(mask > 0, 255, 0).astype(np.uint8)
        return PipelineResult(mask=mask, confidence=confidence)
