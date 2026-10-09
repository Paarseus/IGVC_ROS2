export const meta = {
  name: 'research-topics',
  description: 'Map, research, gap-fill, independently verify (claims + sources in parallel) and correct any list of research topics',
  whenToUse: 'Build or extend evidence-based, cited topic write-ups that follow a STANDARDS.md and TEMPLATE_TOPIC.md in a research folder',
  phases: [
    { title: 'Map', detail: 'break the topic into subtopics; identify foundational references; write SCOPE.md' },
    { title: 'Research', detail: 'download foundational references first, then supporting sources; write README.md' },
    { title: 'Gap check', detail: 'independent pass: compare README with SCOPE.md, search and fill gaps' },
    { title: 'Verify', detail: 'two independent reviewers in parallel: every claim, and every source' },
    { title: 'Correct', detail: 'fix or remove flagged claims and sources; mark Verified' },
  ],
}

// ============================================================================
// Generic research workflow — reusable in any project.
// See research/tools/README.md for how to set up a research folder and run it.
//
// Per topic:  Map → Research → Gap check → Verify (claims ‖ sources) → Correct
// Topics run in parallel; each topic's stages run in order.
//
// Args:
//   root         research folder containing STANDARDS.md, TEMPLATE_TOPIC.md and topics/
//   date         YYYY-MM-DD for "Last updated" and "Accessed"
//   context      optional: scope/background paragraph for all topics
//   contextFile  optional: path to a file holding that paragraph (read by each agent)
//   topics       [{ id, name, dir, questions: [..], notCovered, hints, mode?: 'new' | 'extend' }]
//                'new' (default) writes README.md from scratch; 'extend' keeps an existing one and adds to it.
//   stages       optional subset for all topics, e.g. ['verify', 'correct'] to re-check existing topics only.
//                Default: all five. A topic can override it with its own `stages` field.
// ============================================================================
const ROOT = args.root
const DATE = args.date
const STAGES = args.stages || ['map', 'research', 'gaps', 'verify', 'correct']

const CONTEXT = args.context
  ? `CONTEXT (scope/background — use only to judge relevance):\n${args.context}`
  : args.contextFile
    ? `CONTEXT: read ${args.contextFile} first (scope/background — use only to judge relevance).`
    : `CONTEXT: none given. Cover general principles first, product-specific material last.`

const RULES = `Rules (read both before starting): ${ROOT}/STANDARDS.md (evidence levels, source acceptance checklist, foundational references, file formats, naming, IDs, citations, verification) and ${ROOT}/TEMPLATE_TOPIC.md (README structure). Research only: report what sources, papers and codebases say, with no statements about the reader's own system and no recommendations tailored to it. Plain language: explain unavoidable terms briefly, no filler.`

const DOWNLOAD = `Downloading: curl -L into sources/. Papers, textbooks, theses, standards and datasheets are saved as PDF whenever any open PDF exists (author copy, arXiv, university repository, open-access publisher). For an arXiv preprint, look for the peer-reviewed version and cite that venue (keep the open PDF). Web documentation → clean markdown/text; code/config → raw file pinned to a tag or commit (record it). After every download run \`file\` and read the first lines: a "PDF" that is really HTML (bot-check, login or error page) is deleted and retried from another open copy. Skip files >25 MB (cite them, mark "not downloaded").`

const FILES = t => `${ROOT}/topics/${t.dir}`

const MAP_SCHEMA = { type: 'object', properties: {
  subtopics: { type: 'array', items: { type: 'object', properties: {
    name: { type: 'string' }, questions: { type: 'array', items: { type: 'string' } } }, required: ['name', 'questions'] } },
  foundational: { type: 'array', items: { type: 'object', properties: {
    citation: { type: 'string' }, why: { type: 'string' }, open_copy: { type: 'string' } }, required: ['citation', 'why', 'open_copy'] } },
  summary: { type: 'string' } }, required: ['subtopics', 'foundational', 'summary'] }
const RESEARCH_SCHEMA = { type: 'object', properties: {
  sources_downloaded: { type: 'integer' }, foundational_downloaded: { type: 'integer' }, findings: { type: 'integer' },
  summary: { type: 'string' } }, required: ['sources_downloaded', 'foundational_downloaded', 'findings', 'summary'] }
const GAP_SCHEMA = { type: 'object', properties: {
  gaps_found: { type: 'integer' }, gaps_filled: { type: 'integer' }, sources_added: { type: 'integer' },
  remaining_gaps: { type: 'array', items: { type: 'string' } }, summary: { type: 'string' } },
  required: ['gaps_found', 'gaps_filled', 'sources_added', 'remaining_gaps', 'summary'] }
const CLAIMS_SCHEMA = { type: 'object', properties: {
  checked: { type: 'integer' }, verified: { type: 'integer' }, partly_supported: { type: 'integer' },
  not_supported: { type: 'integer' }, summary: { type: 'string' } },
  required: ['checked', 'verified', 'partly_supported', 'not_supported', 'summary'] }
const SOURCES_SCHEMA = { type: 'object', properties: {
  sources: { type: 'integer' }, failing: { type: 'array', items: { type: 'string' } },
  needs_correction: { type: 'array', items: { type: 'string' } },
  pdf_upgrades: { type: 'integer' }, uncovered_subtopics: { type: 'array', items: { type: 'string' } },
  summary: { type: 'string' } }, required: ['sources', 'failing', 'needs_correction', 'pdf_upgrades', 'uncovered_subtopics', 'summary'] }
const CORRECT_SCHEMA = { type: 'object', properties: {
  fixed: { type: 'integer' }, removed: { type: 'integer' }, final_findings: { type: 'integer' },
  final_sources: { type: 'integer' }, status: { type: 'string' }, summary: { type: 'string' } },
  required: ['fixed', 'removed', 'final_findings', 'final_sources', 'status', 'summary'] }

const topicHeader = t => `TOPIC ${t.id} — ${t.name}
Folder: ${FILES(t)}/  (downloads go in its sources/ subfolder)
Starting research questions:
${t.questions.map((q, i) => `${i + 1}. ${q}`).join('\n')}
Not covered here (belongs to other topics): ${t.notCovered}
Where to look first: ${t.hints}

${CONTEXT}`

// ---- Stage 1: Map ---------------------------------------------------------
const mapPrompt = t => `You are a senior researcher mapping a topic BEFORE anyone collects sources. Your job is understanding and structure, not writing findings.

${topicHeader(t)}

${RULES}

DO:
1. Read broadly (textbook tables of contents, survey/review papers, course syllabi, official documentation indexes) to understand the whole field around this topic. Run many different searches.
2. Break the topic into 6–12 SUBTOPICS that together leave no gap. The starting questions are a seed, not a limit: add aspects they miss (theory, modelling, measurement, calibration, failure modes, testing, how other domains do it, product specifics last). For each subtopic write 2–5 concrete questions.
3. Identify the FOUNDATIONAL references: the standard textbooks, seminal/most-cited papers, survey papers, official standards and primary specifications the field builds on. For every method or algorithm central to the topic, trace it back to the paper that FIRST introduced it: follow the reference lists of later papers backwards, check the first authors' earliest arXiv and conference versions, and the later journal version of the same work (cite the journal; keep the open PDF). Do not stop at the best-known or most-cited paper. Aim for 5–12. For each: full citation, one line on why it is foundational, and the URL of an open copy or "no open copy".
4. ${t.mode === 'extend' ? 'A README already exists: read it and mark which subtopics it covers well and which are thin or missing.' : 'If files already exist in sources/, note which subtopics they cover.'}

Write ${FILES(t)}/SCOPE.md: a one-paragraph overview; a Subtopics table | # | Subtopic | Questions | Covered already? |; a Foundational references table | Citation | Why foundational | Open copy |; and a Search log (queries run). Do not download anything and do not edit README.md. Return the structured result.`

// ---- Stage 2: Research ----------------------------------------------------
const researchPrompt = t => `You are a research analyst. Produce one evidence-based topic file.

${topicHeader(t)}

${RULES}

INPUT: ${FILES(t)}/SCOPE.md (subtopics and foundational references). Cover EVERY subtopic in it.

METHOD:
1. FOUNDATIONAL FIRST: download every foundational reference in SCOPE.md that has an open copy. If none is listed, search again (author pages, university repositories, Google Scholar "all versions", arXiv). If still none, list it in the Sources table marked "not downloaded" and cite only what you could actually read.
2. Then SUPPORTING sources: several different searches per subtopic. Prefer level A/B; use C/D only to fill gaps. Include work from other domains when the principle carries over (say which).
3. ${DOWNLOAD}
4. Files already in sources/: check each is real content; keep and cite the useful ones, delete error pages or irrelevant files.
5. Write ${FILES(t)}/README.md following the template: one Findings sub-section per subtopic, general first, product-specific last. Every finding cites [${t.id}-Snn, page/section]. Status: Draft. Last updated: ${DATE}. Accessed: ${DATE}.
6. Aim for 15–30 sources. Any subtopic question without a good source goes under Open questions.
7. SELF-CHECK before returning: re-open the cited page/line for every number, unit, quote and strong claim ("always", "commonly", "must") you wrote, and fix wording or locations that do not match. Reviewers will check every item.

Write only inside ${FILES(t)}/. Return counts and a 5-line summary.`

const extendPrompt = t => `You are a research analyst EXTENDING an existing, already-reviewed topic.

${topicHeader(t)}

${RULES}

INPUT: ${FILES(t)}/SCOPE.md (subtopics, what is covered already, foundational references) and the existing README.md.

METHOD:
1. FOUNDATIONAL FIRST: download every foundational reference in SCOPE.md not already in sources/ that has an open copy (search hard for one). If none exists, list it marked "not downloaded" and cite only what you could read. Fill the README's "Foundational references" section.
2. Then research every subtopic SCOPE.md marks as thin or missing, several searches each.
3. ${DOWNLOAD}
4. Keep all existing findings and sources unless they are wrong. New sources take the next free IDs (${t.id}-Snn). New findings go in the sub-section for their subtopic, general before product-specific. Update Summary, Recommended practice, Key numbers, How it is tested, Common mistakes, Disagreements, Open questions and the Sources table.
5. Status: Draft. Last updated: ${DATE}.
6. SELF-CHECK before returning: re-open the cited page/line for every number, unit, quote and strong claim ("always", "commonly", "must") you wrote, and fix wording or locations that do not match. Reviewers will check every item.

Write only inside ${FILES(t)}/. Return counts and a 5-line summary.`

// ---- Stage 3: Gap check ---------------------------------------------------
const gapPrompt = t => `You are an independent researcher doing a COMPLETENESS pass on a topic someone else just wrote.

${topicHeader(t)}

${RULES}

Files: ${FILES(t)}/SCOPE.md, README.md, sources/.
1. For every subtopic and question in SCOPE.md, decide whether README.md answers it with cited findings. For every foundational reference, check it is downloaded (or marked "not downloaded" after a real search) and actually used.
2. Think about what an expert in this field would expect to see that is missing from both files: a standard method, a well-known failure mode, a key paper, a disagreement in the literature. Add these to SCOPE.md as new rows.
3. For each gap, search in two ways. (a) Direct terms. (b) The general or theoretical terms the literature uses for the same problem. Examples: "model–plant mismatch", "parameter uncertainty", "robust / adaptive / offset-free control", "disturbance estimation", "sensitivity analysis". Also search neighbouring fields that face the same problem (e.g. agricultural, automotive, aerospace, process control). A gap is "no source found" only after both kinds of search. Then download sources (${DOWNLOAD}), and add cited findings to README.md with the next free source IDs, keeping the template structure and general-first order.
4. Gaps with no reputable source go under Open questions.
5. SELF-CHECK before returning: re-open the cited page/line for every number, unit, quote and strong claim ("always", "commonly", "must") you added, and fix wording or locations that do not match. Reviewers will check every item.
6. Append "Gap check (${DATE})" to SCOPE.md: gaps found, how each was filled, what remains.

Write only inside ${FILES(t)}/. Return counts and a short summary.`

// ---- Stage 4: Verify (two independent reviewers, in parallel) -------------
const verifyClaimsPrompt = t => `You are an independent reviewer of CLAIMS. Check, do not add.

Files: ${FILES(t)}/README.md, sources in ${FILES(t)}/sources/. Rules: ${ROOT}/STANDARDS.md section 5.

For EVERY cited item in README.md (Summary, Foundational references, Findings, Recommended practice, Key numbers, How it is tested, Common mistakes, Disagreements, cited Open questions): open the cited source at the cited location (pdftotext -layout for PDFs; grep for code) and decide Verified / Partly supported / Not supported. Be strict: numbers, units and conditions must match exactly; wording must not overstate (e.g. "commonly" vs "can"); inferences must be labelled as inferences; the cited location must be where the text actually is. Also flag any uncited factual statement.

Write ${FILES(t)}/VERIFICATION.md (replace any old version): header (topic, date ${DATE}, reviewer: independent — claims), a counts table, then | # | Claim (short) | Citation | Status | Evidence (short quote + page/line) | Correction needed |. List every item, not only problems. Do not edit README.md. Another reviewer audits the sources separately; do not grade sources. Return counts and a short summary listing every Partly supported / Not supported item.`

const verifySourcesPrompt = t => `You are an independent reviewer of SOURCES, FORMAT and COVERAGE. Check, do not add.

Files: ${FILES(t)}/README.md, SCOPE.md, sources/. Rules: ${ROOT}/STANDARDS.md sections 2–5.

1. SOURCE QUALITY: grade EVERY source in the Sources table against the "Source acceptance checklist" (author/publisher, venue, traceability, relevance, currency, real content) and confirm its evidence level (A–D) follows STANDARDS.md consistently across the topic. Mark each as foundational or supporting, and general or product-specific.
2. FILES: run \`file\` on every file in sources/. Fail any file whose type does not match its extension, or that is an error, login or bot-check page. Every file must be in the Sources table, and every table row must have a file or be marked "not downloaded".
3. CITATIONS: authors, title, year, venue, link and version/commit are correct. Preprints: check whether a peer-reviewed version exists and give its venue.
4. FORMAT: list papers, books, theses, standards or datasheets saved as text where an open PDF exists (give the URL).
5. FOUNDATIONAL: every foundational reference in SCOPE.md is downloaded and cited, or marked "not downloaded" with a reason.
6. COVERAGE: every subtopic in SCOPE.md has cited findings; findings are general first; give the share of general vs product-specific sources.

Report two separate lists: "failing" = sources that fail the checklist and must be removed; "needs_correction" = sources that pass but need a fix (citation details, level, venue, file name).

Write ${FILES(t)}/SOURCE_AUDIT.md (replace any old version): header (topic, date ${DATE}, reviewer: independent — sources), then | ID | File | Publisher/venue | Level | Foundational? | General/product | Passes checklist? | Reason |, then Files, Format, Foundational and Coverage sections. Do not edit README.md. Return the structured result.`

// ---- Stage 5: Correct -----------------------------------------------------
const correctPrompt = t => `You are the topic editor. Apply both independent reviews.

Files: ${FILES(t)}/README.md, VERIFICATION.md (claims), SOURCE_AUDIT.md (sources), SCOPE.md; sources in ${FILES(t)}/sources/.
- Partly supported claims: rewrite each to state exactly what the source supports (re-check the source first).
- Not supported claims: remove, or replace with a correctly sourced statement if the right source is in sources/.
- Sources needing correction: fix citation details, venue, level or file name as the audit says.
- Failing sources: delete the file and its table row; remove or re-source every claim that cited it.
- Format: replace text copies with the open PDF where the audit gives one (same source ID; re-check cited locations). Apply published-venue citations for preprints.
- Fix evidence levels, citation details and Sources-table problems. Complete the Foundational references section.
- Uncovered subtopics that cannot be filled from existing sources go under Open questions.
- Keep the template and writing rules. Set Status "Verified" only if no Not supported or Partly supported claims and no failing sources remain; otherwise "Draft".
- Final mechanical check: \`file\` on every file in sources/ matches its extension; every file is in the Sources table; every cited ID exists in the table; source IDs have no gaps or duplicates.
- Append "Corrections applied (${DATE})" to VERIFICATION.md, listing what changed in claims and sources.
Report final_findings as the number of cited bullets and table rows in README.md, and final_sources as the number of rows in the Sources table. Return counts and status.`

// ---- Pipeline ---------------------------------------------------------------
// Stage runs only if it is in the topic's own `stages` (or the run-wide default).
const step = (name, fn) => (acc, t, i) => ((t.stages || STAGES).includes(name) ? fn(acc, t, i) : acc)

const results = await pipeline(
  args.topics,
  step('map', (_, t) => agent(mapPrompt(t), { label: `map:${t.id}`, phase: 'Map', schema: MAP_SCHEMA, agentType: 'general-purpose' })
    .then(m => ({ map: m }))),
  (acc, t) => (acc === t ? {} : acc),   // topic skipped Map: start from an empty result
  step('research', (acc, t) => agent(t.mode === 'extend' ? extendPrompt(t) : researchPrompt(t), { label: `research:${t.id}`, phase: 'Research', schema: RESEARCH_SCHEMA, agentType: 'general-purpose' })
    .then(r => ({ ...acc, research: r }))),
  step('gaps', (acc, t) => agent(gapPrompt(t), { label: `gaps:${t.id}`, phase: 'Gap check', schema: GAP_SCHEMA, agentType: 'general-purpose' })
    .then(g => ({ ...acc, gaps: g }))),
  step('verify', (acc, t) => parallel([
    () => agent(verifyClaimsPrompt(t), { label: `verify-claims:${t.id}`, phase: 'Verify', schema: CLAIMS_SCHEMA, agentType: 'general-purpose' }),
    () => agent(verifySourcesPrompt(t), { label: `verify-sources:${t.id}`, phase: 'Verify', schema: SOURCES_SCHEMA, agentType: 'general-purpose' }),
  ]).then(([c, s]) => ({ ...acc, verify: { claims: c, sources: s } }))),
  step('correct', (acc, t) => agent(correctPrompt(t), { label: `correct:${t.id}`, phase: 'Correct', schema: CORRECT_SCHEMA, agentType: 'general-purpose' })
    .then(c => ({ ...acc, correct: c }))),
)

// Compact status table for the caller; full per-agent results stay in the journal.
return args.topics.map((t, i) => {
  const r = results[i] || {}
  return {
    id: t.id,
    status: r.correct?.status ?? 'incomplete',
    findings: r.correct?.final_findings ?? r.research?.findings ?? null,
    sources: r.correct?.final_sources ?? r.research?.sources_downloaded ?? null,
    claims: r.verify?.claims ? `${r.verify.claims.verified}/${r.verify.claims.checked} verified, ${r.verify.claims.partly_supported} partly, ${r.verify.claims.not_supported} not` : null,
    failing_sources: r.verify?.sources?.failing ?? null,
    remaining_gaps: r.gaps?.remaining_gaps ?? null,
    detail: r,
  }
})
