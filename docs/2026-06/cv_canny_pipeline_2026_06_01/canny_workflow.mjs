export const meta = {
  name: 'canny-lane-pipeline',
  description: 'Research, design, implement & validate a robust white-line-only Canny lane pipeline for avros_perception',
  whenToUse: 'Building a new IGVC AutoNav lane-detection pipeline based on Canny edges + color/geometry gating, validated on real ZED field frames',
  phases: [
    { title: 'Research', detail: 'web + official OpenCV docs + codebase contract + IGVC rules + field-frame characterization' },
    { title: 'Design', detail: '3 competing designs, judged, synthesized into one spec' },
    { title: 'Implement', detail: 'write canny.py + wiring + standalone validation harness' },
    { title: 'Validate', detail: 'run on real frames + adversarial probes + tuning loop' },
    { title: 'Report', detail: 'CANNY_PIPELINE.md design + evidence + integration steps' },
  ],
}

// ----------------------------------------------------------------------------
// Shared context embedded in every agent prompt so agents counter the EXACT
// documented failure modes instead of rediscovering them.
// ----------------------------------------------------------------------------
const REPO = '/home/mspacman/IGVC_ROS2'
const DOCDIR = REPO + '/docs/cv_canny_pipeline_2026_06_01'
const PKG = REPO + '/src/avros_perception/avros_perception'

const CONTRACT = `
PIPELINE CONTRACT (must match EXACTLY — read ${PKG}/pipelines/base.py and
${PKG}/perception_node.py to confirm):
- A pipeline subclasses avros_perception.pipelines.base.Pipeline and implements
  run(self, bgr, depth=None) -> PipelineResult(mask, confidence).
- bgr: HxWx3 uint8 in OpenCV BGR order (NOT RGB).
- mask: (H,W) uint8 of CLASS IDs; 0 == free/background; lane pixels == class_id_lane (default 1).
- confidence: (H,W) uint8 0..255.
- mask & confidence MUST be the SAME HxW as the input bgr (the node downsamples to cloud shape itself).
- Single output class: everything detected -> class_id_lane. (Camera layer marks lane_white ONLY;
  barrels/potholes are dropped — LiDAR/STVL handles obstacles.)
- Lanes are kept LETHAL downstream by design (stall-safe > line-touch DQ). Do NOT try to emit a
  cost gradient or "traversable" lanes — just a clean binary lane mask.
- The sky / out-of-interest ROI polygon (param sky_roi_poly, normalized flat list
  [x0,y0,x1,y1,...]) must be zeroed LAST, AFTER detection (background tents/cones are bright).
  Reuse the _reshape_poly/_roi_polygon_px pattern from adaptive.py verbatim.
- Params are read from self.params (a shared dict, re-read every frame so ros2 param set is live).
  In-code defaults must match the field-tuned perception.yaml.
- Register the pipeline: add import + __all__ + PIPELINES['canny'] in ${PKG}/pipelines/__init__.py.
- Surface new params: add their names to _PIPELINE_PARAM_NAMES and add declare_parameter(...) calls
  in ${PKG}/perception_node.py, and a documented block in
  ${REPO}/src/avros_perception/config/perception.yaml.
`

const PROBLEM = `
WHY A NAIVE CANNY PIPELINE WAS ALREADY REJECTED (read the docstring of
${PKG}/pipelines/adaptive.py — the "Deliberately NOT included" section):
  "Sobel/Canny gradient OR: measured to triple px (280->923) and push the exposure CV 0.031->0.273
   by importing asphalt aggregate/crack texture — directly defeats keep-asphalt-clean. There is no
   IPM-warp + sliding-window stage downstream to reject that noise (unlike Udacity-style pipelines)."
So a ROBUST Canny pipeline MUST add the downstream geometric rejection the old attempt lacked.

ADVERSARIES, ranked by difficulty:
1. ORANGE/BROWN/GREEN barrels & cones -> EASY: white-paint color gate (low HLS-S, high HLS-L), same
   idea as the working adaptive_max_sat=70 gate. Their edges are not white.
2. Asphalt CRACKS / aggregate texture & tire scuffs -> HARD: they are gray/low-saturation like the
   paint, so the color gate does NOT remove them. Needs GEOMETRY: connected-component length/area
   filter and/or IPM bird's-eye warp + sliding-window/Hough/RANSAC line fit (only long smooth
   ground-plane lines survive).
3. WHITE obstacles -> IGVC 2026 rules explicitly say obstacles come in "various colors (white,
   orange, brown, green, black...)". A WHITE barrel/cone DEFEATS the color gate. Must be rejected by
   geometry (not a long thin ground line) and/or depth (ZED depth is available as run()'s depth arg;
   a ground line is at ground-plane depth, an obstacle face is raised/closer). Account for this.
4. SHADOWS / worn low-contrast paint / dashed-line gaps -> exposure-invariant thresholds + line
   continuity (Hough gap-filling / morphology) without re-importing texture.

EXPOSURE: the ZED X auto-exposure hunts in a limit cycle even while stationary (V_median ~141<->167
every ~0.3-0.4s). The pipeline must be exposure-invariant by construction (adaptive thresholds /
ratios / normalization), NOT a fixed absolute brightness. Keep AE ON.
`

const FRAMES = `
REAL VALIDATION FRAMES (480x300 BGR ZED captures of IGVC asphalt with white tape):
  ${REPO}/exp_frames/f00_v134.png ... f06_v134.png  (7 frames, ~same scene, capture exposure drift)
  ${REPO}/docs/cv_adaptive_debug_2026_05_31/input_rgb_0*.png  (more scenes/angles)
  ${REPO}/rgb_now.png , ${REPO}/overlay_v205.png , ${REPO}/overlay_roi30.png (reference outputs)
OpenCV 4.13.0 + numpy are installed locally; agents may run python3 with cv2 directly on these files.
`

const RULES = `IGVC 2026 rules full text: ${DOCDIR}/igvc_2026_rules_fulltext.txt
Key AutoNav perception facts: asphalt course ~500ft; outer boundaries are CONTINUOUS OR DASHED WHITE
lines ~3 inches wide; track width varies 10-24ft; primarily SINUSOIDAL curves with repetitive barrel
obstacles; obstacles are VARIOUS COLORS INCLUDING WHITE, orange, brown, green, black; a ramp section;
crossing internal lines = E-stop end of run (so false-negatives that let the robot cross a line are
costly, but so are false-positive phantom lines that box it in).`

const RULES_FILE = DOCDIR + '/igvc_2026_rules_fulltext.txt'

// ----------------------------------------------------------------------------
const RESEARCH_SCHEMA = {
  type: 'object', additionalProperties: false,
  required: ['topic', 'key_findings', 'recommended_techniques', 'pitfalls', 'concrete_params', 'citations'],
  properties: {
    topic: { type: 'string' },
    key_findings: { type: 'array', items: { type: 'string' } },
    recommended_techniques: { type: 'array', items: { type: 'string' } },
    pitfalls: { type: 'array', items: { type: 'string' } },
    concrete_params: { type: 'array', items: { type: 'string' },
      description: 'specific OpenCV calls / param values / thresholds to use' },
    citations: { type: 'array', items: { type: 'string' }, description: 'URLs or file paths' },
  },
}

const DESIGN_SCHEMA = {
  type: 'object', additionalProperties: false,
  required: ['name', 'one_liner', 'stages', 'params', 'rejects_barrels', 'rejects_cracks',
             'handles_white_obstacles', 'exposure_invariance', 'perf_notes', 'risks'],
  properties: {
    name: { type: 'string' },
    one_liner: { type: 'string' },
    stages: { type: 'array', items: { type: 'string' }, description: 'ordered pipeline stages with the cv2 calls' },
    params: { type: 'array', items: {
      type: 'object', additionalProperties: false,
      required: ['name', 'default', 'purpose'],
      properties: { name: {type:'string'}, default: {type:'string'}, purpose: {type:'string'} } } },
    rejects_barrels: { type: 'string' },
    rejects_cracks: { type: 'string' },
    handles_white_obstacles: { type: 'string' },
    exposure_invariance: { type: 'string' },
    perf_notes: { type: 'string', description: 'cost on Jetson Orin; must not starve the 20Hz MPPI loop' },
    risks: { type: 'array', items: { type: 'string' } },
  },
}

const JUDGE_SCHEMA = {
  type: 'object', additionalProperties: false,
  required: ['robustness', 'simplicity', 'igvc_fit', 'total', 'strengths', 'weaknesses'],
  properties: {
    robustness: { type: 'number', description: '0-10: rejects cracks+white obstacles, exposure-invariant' },
    simplicity: { type: 'number', description: '0-10: maintainable, few magic params, fast' },
    igvc_fit: { type: 'number', description: '0-10: fits IGVC course (dashed/continuous white, sinusoidal, ramp)' },
    total: { type: 'number' },
    strengths: { type: 'array', items: { type: 'string' } },
    weaknesses: { type: 'array', items: { type: 'string' } },
  },
}

const VERDICT_SCHEMA = {
  type: 'object', additionalProperties: false,
  required: ['adversary', 'is_real_problem', 'severity', 'evidence', 'mitigation'],
  properties: {
    adversary: { type: 'string' },
    is_real_problem: { type: 'boolean' },
    severity: { type: 'string', enum: ['low', 'medium', 'high', 'critical'] },
    evidence: { type: 'string', description: 'what you observed on the actual frames' },
    mitigation: { type: 'string', description: 'concrete param/stage change to fix it' },
  },
}

const VALIDATION_SCHEMA = {
  type: 'object', additionalProperties: false,
  required: ['ran_ok', 'frames_tested', 'metrics', 'barrels_rejected', 'cracks_rejected',
             'white_line_recall', 'exposure_stable', 'pass', 'changes_made', 'remaining_issues'],
  properties: {
    ran_ok: { type: 'boolean' },
    frames_tested: { type: 'number' },
    metrics: { type: 'string', description: 'per-frame lane px counts, frame-to-frame CV, overlay paths' },
    barrels_rejected: { type: 'boolean' },
    cracks_rejected: { type: 'boolean' },
    white_line_recall: { type: 'string', description: 'qualitative: are the real white lines traced?' },
    exposure_stable: { type: 'string', description: 'CV of lane px across the 7 exp_frames; <0.15 is good' },
    pass: { type: 'boolean' },
    changes_made: { type: 'array', items: { type: 'string' } },
    remaining_issues: { type: 'array', items: { type: 'string' } },
  },
}

// ============================================================================
phase('Research')
log('Deep research: web + official OpenCV docs + codebase contract + IGVC rules + real-frame characterization')

const RESEARCH_TASKS = [
  {
    label: 'web:advanced-lane-finding',
    prompt: `You are a CV research agent. Do DEEP ONLINE RESEARCH (use WebSearch + WebFetch — load the
schemas via ToolSearch first) on classic-to-advanced CANNY-BASED lane detection pipelines for
self-driving / ground robots. Cover: the Udacity "Advanced Lane Finding" lineage, color+gradient
combination (when to AND vs OR color masks with Canny edges), HLS/LAB white-line color thresholding,
region-of-interest masking. Find what makes these robust vs brittle. ${PROBLEM}
Return the structured schema with CONCRETE OpenCV calls and starting param values that would suit
this course: ${RULES}`,
  },
  {
    label: 'web:ipm-sliding-window',
    prompt: `You are a CV research agent. Do DEEP ONLINE RESEARCH (WebSearch + WebFetch) on INVERSE
PERSPECTIVE MAPPING (bird's-eye / top-down warp via cv2.getPerspectiveTransform + warpPerspective) and
the downstream lane-pixel extractors that run on the warped image: sliding-window histogram search,
polynomial (np.polyfit) lane fitting, and RANSAC line fitting. Explain how the ground-plane warp +
shape constraint REJECTS non-ground edges (barrel faces, crack speckle, raised white obstacles) that a
flat-image Canny cannot. Note how to pick the 4 source/destination points without a calibrated
homography, and the cost on an embedded GPU. ${PROBLEM}
Return the schema with concrete cv2 calls.`,
  },
  {
    label: 'docs:opencv-canny-hough',
    prompt: `You are an API-accuracy agent. Fetch the OFFICIAL OpenCV 4.x documentation (use context7:
resolve-library-id then query-docs for "opencv", and/or WebFetch docs.opencv.org/4.x) for the EXACT
signatures, parameter semantics, and gotchas of: cv2.Canny (aperture, L2gradient, the 2:1-3:1
low:high threshold guidance), cv2.HoughLinesP (rho/theta/threshold/minLineLength/maxLineGap),
cv2.getPerspectiveTransform + cv2.warpPerspective, cv2.cvtColor BGR2HLS/BGR2LAB channel order,
cv2.adaptiveThreshold, cv2.connectedComponentsWithStats, morphology. Flag any call that RAISES on bad
input (e.g. adaptiveThreshold needs 8-bit single channel + odd blockSize; GaussianBlur needs odd
kernel — these already bit this codebase). Return the schema; concrete_params = exact signatures.`,
  },
  {
    label: 'web:white-segmentation-exposure',
    prompt: `You are a CV research agent. Research robust WHITE-LINE color segmentation that survives
exposure changes and shadows on asphalt: HLS-L / LAB-L thresholding, adaptive/Otsu vs fixed, top-hat
morphology for bright thin features, and shadow handling. Also research how to distinguish painted
WHITE LINES from WHITE OBSTACLES (rule says obstacles include white) using shape/continuity/depth
rather than color. ${PROBLEM}
Return the schema with concrete cv2 calls + thresholds.`,
  },
  {
    label: 'code:contract-and-integration',
    agentType: 'Explore',
    prompt: `You are a codebase-analysis agent. Read these files IN FULL and produce the EXACT contract
+ integration recipe a new 'canny' pipeline must follow, quoting real line numbers:
  ${PKG}/pipelines/base.py
  ${PKG}/pipelines/adaptive.py  (study its ROI handling, param re-read pattern, sat gate, CC filter)
  ${PKG}/pipelines/sooner25.py
  ${PKG}/pipelines/__init__.py  (how build_pipeline + PIPELINES registry works)
  ${PKG}/perception_node.py     (how params are declared/surfaced; _PIPELINE_PARAM_NAMES; how mask is
                                 downsampled to cloud shape; the process_at_full_res path)
  ${REPO}/src/avros_perception/config/perception.yaml  (param style + the adaptive block)
${CONTRACT}
In recommended_techniques, give the precise edits (file + what to add) to register a 'canny' pipeline
and surface its params. In pitfalls, list every cv2 call that raises + the odd-kernel guards.
concrete_params = the exact param names/defaults the new pipeline should expose.`,
  },
  {
    label: 'rules:igvc-constraints',
    prompt: `You are a requirements agent. Read the IGVC 2026 rules text at ${RULES_FILE}
Extract EVERY hard constraint that affects a lane-perception pipeline for the AutoNav challenge: line
color, line width, continuous vs dashed, course surface, track-width range, obstacle colors (note the
WHITE obstacles explicitly), ramp, sinusoidal geometry, what counts as crossing a line / E-stop, and
any speed/timing that bounds acceptable perception latency. ${PROBLEM}
Return the schema; key_findings = the constraints quoted, concrete_params = numeric values (inches,
feet, ms) the design must respect.`,
  },
  {
    label: 'frames:characterize',
    prompt: `You are a quantitative CV agent with Bash + python3 + cv2 4.13.0. ${FRAMES}
ACTUALLY RUN python on these frames (do not guess). Measure and report:
  (a) white-line vs asphalt separation in HLS-L and HLS-S and LAB — histograms / medians / p95;
  (b) where the lane lines sit vertically (to inform the sky ROI and any IPM source quad);
  (c) frame-to-frame exposure drift across exp_frames/f00..f06 (median L per frame);
  (d) run a NAIVE cv2.Canny on 2-3 frames and COUNT how many edge px land on (i) the white line vs
      (ii) asphalt texture/cracks vs (iii) any barrels/clutter — quantify the noise problem;
  (e) then run Canny AND-ed with a white gate (low HLS-S & high HLS-L) and report how much asphalt/
      barrel noise drops vs how much line survives.
Save any debug PNGs under ${DOCDIR}/research_frames/. ${PROBLEM}
Return the schema with the real numbers in key_findings and the gate thresholds that worked in
concrete_params.`,
  },
]

const research = (await parallel(
  RESEARCH_TASKS.map(t => () => agent(t.prompt, {
    label: t.label, phase: 'Research', schema: RESEARCH_SCHEMA, agentType: t.agentType,
  }))
)).filter(Boolean)

const researchDigest = research.map(r =>
  `### ${r.topic}\nFINDINGS:\n- ${r.key_findings.join('\n- ')}\nTECHNIQUES:\n- ${r.recommended_techniques.join('\n- ')}\n` +
  `PARAMS: ${r.concrete_params.join(' | ')}\nPITFALLS: ${r.pitfalls.join(' | ')}\nCITES: ${r.citations.join(' | ')}`
).join('\n\n')
log(`Research complete (${research.length} reports). Designing.`)

// ============================================================================
phase('Design')

const DESIGN_VARIANTS = [
  { label: 'design:color-gated-canny', angle:
    `MINIMAL & FAST: Canny edges AND-ed with an exposure-invariant white color gate (HLS/LAB) + a
     connected-component length/aspect filter to drop crack speckle. No IPM. Prioritize Jetson perf
     and few params. Must still credibly reject white obstacles via shape (long thin line vs blob).` },
  { label: 'design:ipm-sliding-window', angle:
    `MOST ROBUST: full Udacity-style — undistort/ROI -> IPM bird's-eye warp -> color+Canny combined
     binary -> sliding-window/polyfit (or HoughLinesP) line extraction -> unwarp the line mask back to
     image space. Geometry rejects raised/white obstacles and cracks. Be honest about Jetson cost and
     the un-calibrated homography risk.` },
  { label: 'design:hybrid-depth', angle:
    `HYBRID: color-gated Canny + HoughLinesP for continuity, PLUS use the ZED depth arg (run(bgr,depth))
     to drop edges that are NOT on the ground plane — this is the principled way to reject WHITE
     obstacles. Address what happens when depth is None/sparse (graceful fallback to color+geometry).` },
]

const designs = (await parallel(DESIGN_VARIANTS.map(v => () => agent(
  `You are a senior CV architect. Propose a CONCRETE pipeline design for a new 'canny' lane pipeline.
ANGLE: ${v.angle}
${CONTRACT}
${PROBLEM}
${RULES}

RESEARCH DIGEST (use it):
${researchDigest}

Output the design schema. Every stage must name its cv2 call. Defaults must be exposure-invariant and
suited to 480x300 ZED asphalt frames. Be explicit about how each of the 4 adversaries is rejected.`,
  { label: v.label, phase: 'Design', schema: DESIGN_SCHEMA }
)))).filter(Boolean)

// Judge each design independently, then synthesize.
const judged = await parallel(designs.map(d => () => agent(
  `Score this lane-pipeline design for IGVC AutoNav. Be skeptical and concrete.
${RULES}
${PROBLEM}
DESIGN:
${JSON.stringify(d, null, 2)}
Return the judge schema.`,
  { label: `judge:${d.name}`, phase: 'Design', schema: JUDGE_SCHEMA }
)))

const scored = designs.map((d, i) => ({ design: d, score: judged[i] ? judged[i].total : 0, judge: judged[i] }))
  .sort((a, b) => b.score - a.score)
log(`Design scores: ${scored.map(s => `${s.design.name}=${s.score}`).join(', ')}`)

const finalDesign = await agent(
  `You are the lead architect. Synthesize ONE final, buildable design for the 'canny' pipeline.
Start from the highest-scored design but graft the best ideas from the others and fix every weakness
the judges raised. The result must be implementable in a single self-contained canny.py with NO new
heavy dependencies (cv2 + numpy only; ZED depth optional). It MUST conform to the contract and reject
all 4 adversaries. Keep it as simple as possible while robust — prefer a color-gated Canny + geometry
core with depth as an optional refinement, unless the evidence strongly favors full IPM.
${CONTRACT}
${PROBLEM}
${RULES}

CANDIDATES (best first):
${JSON.stringify(scored.map(s => ({ design: s.design, judge: s.judge })), null, 2)}

RESEARCH:
${researchDigest}

Return the design schema as the single final spec, with production-ready default param values.`,
  { label: 'design:synthesize', phase: 'Design', schema: DESIGN_SCHEMA }
)
log(`Final design: ${finalDesign.name} — ${finalDesign.one_liner}`)

// ============================================================================
phase('Implement')

const implReport = await agent(
  `You are a ROS2/OpenCV implementation engineer working in ${REPO}. Implement the 'canny' pipeline
EXACTLY per this final design, conforming to the contract. Do ALL of the following with real file
edits (Read before Edit), then RUN the validation harness and report what happened.

FINAL DESIGN:
${JSON.stringify(finalDesign, null, 2)}

${CONTRACT}
${PROBLEM}
${FRAMES}

STEPS:
1. Write ${PKG}/pipelines/canny.py — class CannyPipeline(Pipeline) implementing run(self, bgr, depth=None).
   - Reuse the _reshape_poly/_roi_polygon_px ROI pattern from adaptive.py (sky ROI zeroed LAST).
   - Read all tunables from self.params with the design's defaults (defaults must match the yaml block).
   - Odd-coerce any kernel/blockSize; clamp lower bounds (these RAISE in cv2 — see adaptive.py).
   - Output: mono8 class-ID mask (lane px -> class_id_lane), uint8 confidence, both input HxW.
   - Thorough module + inline docstrings explaining WHY each stage rejects which adversary (match the
     house style of adaptive.py).
2. Register it: edit ${PKG}/pipelines/__init__.py (import, __all__, PIPELINES['canny']).
3. Surface params: edit ${PKG}/perception_node.py — add new 'canny_*' names to _PIPELINE_PARAM_NAMES
   and add declare_parameter(...) calls (with ParameterDescriptor ranges like the adaptive ones).
4. Document params: add a clearly-commented "-------- Canny pipeline (pipeline:='canny') --------"
   block to ${REPO}/src/avros_perception/config/perception.yaml with the same defaults. Do NOT change
   the active 'pipeline:' selector (leave it as-is — canny is opt-in).
5. Write a SELF-CONTAINED harness ${DOCDIR}/validate_canny.py that:
   - imports CannyPipeline directly from the file via importlib.util (NO colcon/ROS needed),
   - runs it on ALL frames in ${REPO}/exp_frames/*.png and
     ${REPO}/docs/cv_adaptive_debug_2026_05_31/input_rgb_0*.png,
   - writes per-frame overlay PNGs (line px tinted) to ${DOCDIR}/validation/,
   - prints a JSON summary: per-frame lane-px count, frame-to-frame coefficient-of-variation across
     the 7 exp_frames (exposure stability), and a rough white-line-recall / asphalt-noise estimate.
6. RUN: python3 ${DOCDIR}/validate_canny.py  — capture the output.
Verify python3 -c "import ast; ast.parse(open('${PKG}/pipelines/canny.py').read())" passes and the
harness runs without exception. Return the validation schema describing the FIRST run's results.`,
  { label: 'implement+first-run', phase: 'Implement', schema: VALIDATION_SCHEMA }
)
log(`Implemented. First run ran_ok=${implReport && implReport.ran_ok} pass=${implReport && implReport.pass}`)

// ============================================================================
phase('Validate')

// Adversarial probes — each hammers one failure mode on the ACTUAL frames, in parallel.
const PROBES = [
  { a: 'white obstacle', p: `Probe whether a WHITE obstacle (rules say obstacles include white) would be
     falsely marked as a lane. Reason about the implemented canny.py geometry/depth rejection and, if
     possible, simulate a white blob on a frame with python+cv2 and run the pipeline.` },
  { a: 'asphalt cracks/texture', p: `Probe whether asphalt cracks/aggregate/tire-scuff texture survives
     into the mask. Run the pipeline on the real frames and count non-line edge px in the output. This
     is the documented failure that killed the old Canny attempt — verify it's actually fixed.` },
  { a: 'shadow + worn/low-contrast paint', p: `Probe robustness to shadows on asphalt and faint/worn
     paint and dashed-line gaps — does the pipeline either hallucinate a shadow edge as a line or drop
     the real faint line? Use the real frames.` },
  { a: 'exposure limit-cycle', p: `Probe exposure invariance: run the pipeline across exp_frames/f00..f06
     (which capture the ZED AE swing) and check the lane-px count is stable (low CV). A fixed-brightness
     assumption would flicker like sooner25 did.` },
]

const probes = (await parallel(PROBES.map(pr => () => agent(
  `You are an adversarial CV verifier with Bash + python3 + cv2. The implemented pipeline is at
${PKG}/pipelines/canny.py and a harness at ${DOCDIR}/validate_canny.py. ${pr.p}
${FRAMES}
${PROBLEM}
Default to is_real_problem=true unless the frames show it's genuinely handled.
Return the verdict schema with evidence from what you actually ran/observed.`,
  { label: `probe:${pr.a}`, phase: 'Validate', schema: VERDICT_SCHEMA }
)))).filter(Boolean)

const probeDigest = probes.map(p =>
  `- [${p.severity}] ${p.adversary}: real=${p.is_real_problem}. ${p.evidence} => FIX: ${p.mitigation}`
).join('\n')
log(`Adversarial probes done. Tuning to address findings.`)

// Tuning loop: up to 3 rounds. Each round runs the harness, addresses probe findings + metrics,
// edits canny.py / perception.yaml defaults, re-runs. Stops when pass.
let val = implReport
for (let round = 1; round <= 3; round++) {
  val = await agent(
    `You are tuning the implemented 'canny' pipeline (${PKG}/pipelines/canny.py, harness
${DOCDIR}/validate_canny.py) on the REAL frames. Round ${round}/3.
ADVERSARIAL FINDINGS to resolve:
${probeDigest}

PREVIOUS RUN: ${JSON.stringify(val)}
${FRAMES}
${PROBLEM}

Run the harness, read the overlays/metrics, and adjust pipeline code and/or the perception.yaml +
in-code DEFAULT param values to: (1) trace the real white lines, (2) reject asphalt cracks & barrels &
white obstacles, (3) keep lane-px count stable across the exposure-swing frames (CV < 0.15). Keep
in-code defaults and the yaml block in sync. Re-run the harness to confirm. Return the validation
schema. Set pass=true ONLY if white lines are traced AND cracks/barrels are rejected AND exposure is
stable. If already passing, make no changes and report pass=true.`,
    { label: `tune:round${round}`, phase: 'Validate', schema: VALIDATION_SCHEMA }
  )
  log(`Round ${round}: pass=${val && val.pass} | ${val && val.remaining_issues ? val.remaining_issues.join('; ') : ''}`)
  if (val && val.pass) break
}

// ============================================================================
phase('Report')

const reportPath = `${DOCDIR}/CANNY_PIPELINE.md`
const report = await agent(
  `You are a technical writer + CV engineer. Write ${reportPath} documenting the new 'canny' pipeline.
Include: (1) the problem & why naive Canny was rejected before and how THIS design fixes it; (2) the
final design — every stage with its cv2 call and WHY it rejects each adversary (barrels, cracks, white
obstacles, shadows/exposure); (3) the full param table with defaults; (4) VALIDATION EVIDENCE — the
measured numbers and overlay image paths under ${DOCDIR}/validation/; (5) IGVC-rules compliance notes
(white ~3in continuous/dashed lines, white obstacles, sinusoidal, ramp); (6) exact integration steps
to enable it (set pipeline:'canny', build, run) and the files changed; (7) honest known limits +
when to prefer adaptive/yolopv2 instead. Match the house doc style.
FINAL DESIGN:
${JSON.stringify(finalDesign, null, 2)}

VALIDATION:
${JSON.stringify(val, null, 2)}

ADVERSARIAL PROBES:
${probeDigest}

Return a concise 8-12 line summary for the human: what was built, the validation verdict, the files
changed, how to enable it, and whether you recommend making it the default pipeline.`,
  { label: 'report', phase: 'Report' }
)

return {
  finalDesign,
  designScores: scored.map(s => ({ name: s.design.name, score: s.score })),
  validation: val,
  adversarialProbes: probes,
  reportPath,
  summary: report,
}
