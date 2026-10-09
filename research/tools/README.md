# Research tools

`research_topics.workflow.js` is a reusable workflow for evidence-based research write-ups. It works for any project and any subject. Nothing in it is specific to this project; project-specific scope goes in a context file.

## What it does

Each topic goes through five stages, each run by a separate agent. Topics run in parallel.

| Stage | Agent's job | Writes |
|---|---|---|
| 1. Map | Understand the field, break the topic into 6–12 subtopics, identify foundational references | `SCOPE.md` |
| 2. Research | Download foundational references first, then supporting sources; write findings for every subtopic | `README.md`, `sources/` |
| 3. Gap check | An independent researcher compares the result with the map, adds what an expert would expect, and fills the gaps | updates all three |
| 4. Verify | Two independent reviewers work in parallel: one checks **every claim** against its source, the other audits **every source** (quality, real file, PDF where available, citation details, foundational coverage, subtopic coverage) | `VERIFICATION.md`, `SOURCE_AUDIT.md` |
| 5. Correct | Fix or remove flagged claims and sources, run the final file checks, and mark the topic Verified | final `README.md` |

## Setting up a research folder (any project)

```
<research-root>/
├── STANDARDS.md        rules: evidence levels, source checklist, formats, naming, verification
├── TEMPLATE_TOPIC.md   structure of every topic README
├── tools/
│   ├── research_topics.workflow.js
│   └── context_<name>.md   optional scope/background for a set of topics
└── topics/<ID>_<name>/     created by the workflow
```

Copy `STANDARDS.md`, `TEMPLATE_TOPIC.md` and the workflow from this folder, then edit the examples in `STANDARDS.md` (evidence-level examples, scope section) for the new field.

## Running it

Ask Claude Code to run the workflow at `tools/research_topics.workflow.js` with these arguments:

```json
{
  "root": "/abs/path/to/research",
  "date": "YYYY-MM-DD",
  "contextFile": "/abs/path/to/research/tools/context_<name>.md",
  "topics": [
    { "id": "A1", "name": "Topic name", "dir": "A1_topic_name",
      "questions": ["research question 1", "research question 2"],
      "notCovered": "what belongs to other topics",
      "hints": "where to look first: key authors, docs, repositories",
      "mode": "new" }
  ]
}
```

- `mode: "extend"` adds to an existing topic instead of rewriting it.
- `stages: ["verify", "correct"]` re-checks existing topics without new research. Any subset of `map`, `research`, `gaps`, `verify`, `correct` works.
- `context` (a string) can replace `contextFile`.
- Run 2–3 topics per run. Each topic uses 6 agents; larger batches take longer and are harder to review.

The workflow returns a status table (status, findings, sources, claim-check counts, failing sources, remaining gaps). Full per-agent results are in the run's journal.

## Checking results

After each run, read each topic's `VERIFICATION.md` (Corrections applied) and `SOURCE_AUDIT.md`, then update the status table in the research folder's README.
