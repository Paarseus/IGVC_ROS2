# Subsystem Folder Template

Every folder in `subsystems/` uses this layout and this README structure. Research only: what the sources say. Comparing it against our codebase is a later phase.

## Folder layout
```
subsystems/NN_name/
├── README.md      ← findings, written with the structure below
└── sources/       ← downloaded papers, manuals, docs, config files
```

**File names in `sources/`:** `<org-or-author>_<year>_<short-topic>.<ext>`
e.g. `rev_2025_sparkmax_closed_loop.md`, `mandow_2007_skid_steer_kinematics.pdf`, `clearpath_husky_ekf.yaml`.
Web pages are saved as plain text or markdown; GitHub files as the raw file.

## README structure

```markdown
# <Subsystem name>

**Covers:** one sentence on what this subsystem is.
**Question:** one sentence on what we need to know to make it correct and precise.

## Key findings
- One fact per bullet, plain language, with its source: [file name]
- Most important first

## Recommended practice
- What the sources recommend doing, in order, as short action bullets: [file name]

## Numbers worth knowing
| Item | Value | Source |
|---|---|---|

## How others test it
- Test methods and pass criteria used in the sources: [file name]

## Common mistakes
- Failure modes the sources warn about: [file name]

## Sources
| File | Author / organisation | Year | Link | What it covers |
|---|---|---|---|---|

## Open questions
- What the sources did not answer or disagree on
```

## Writing rules
- Plain language. If a technical term is unavoidable, explain it in a few words the first time.
- No filler, no opinions without a source.
- Every finding cites a file in `sources/`.
- Only reputable sources: manufacturer documentation, official ROS / Nav2 documentation, peer-reviewed papers, well-known open-source robot stacks, IGVC design reports.
- Mark anything not confirmed by a source as **(unverified)**.
