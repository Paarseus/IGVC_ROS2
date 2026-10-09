# Research Standards

Every topic in this folder follows these rules. They keep the research clean, checkable and evidence-based.

## 1. Scope
- **Research only.** Record what published sources, manufacturer documentation and open-source codebases say.
- **No comparison with our robot or code yet.** That is a later phase. Our field measurements live separately in `evidence/`.
- **General first, specific second.** Each topic starts with the general principles, methods and lessons that apply to mobile robots and vehicles broadly: control theory, estimation, calibration, testing, and how other robots and vehicles do it. Material on the specific hardware and software we use (e.g. a given motor controller or GNSS receiver) is included as one section, not the whole topic.
- **Wider sources are welcome.** Research from other vehicle types (field robots, agricultural and planetary rovers, automotive, drones, industrial AGVs) is included when the principle carries over. Say what vehicle type it comes from.

## 2. Evidence
Every finding needs a source, and every source gets an evidence level.

| Level | What counts | Examples |
|---|---|---|
| **A** | Peer-reviewed research, textbooks from academic publishers, official standards, manufacturer specifications (including machine-readable specs) | IEEE/Springer papers, REP standards, datasheets, user manuals, a vendor's official protocol spec |
| **B** | Official documentation, application notes, and source code from the manufacturer or the official project | REV, Xsens, u-blox, ROS 2, Nav2, robot_localization docs; REVLib, ros2_controllers, robot_localization source |
| **C** | Other established open-source code and team reports | Clearpath configs, third-party driver libraries, IGVC design reports |
| **D** | Expert community material, used only when A–C are missing | Maintainer answers on GitHub issues or ROS Discourse |

Blogs, forum guesses, AI-generated text and marketing pages are not used.

- **Preprints** (e.g. arXiv) are level A only when a peer-reviewed version exists. Cite that venue and keep the open PDF. Otherwise they are level C.
- **Code** is cited at a pinned tag or commit, never a moving branch.

### Source acceptance checklist
A source is kept only if it passes all of these:

| Check | Accept | Reject |
|---|---|---|
| **Author / publisher** | Named authors at a university, research lab or company; the manufacturer itself; an official project (ROS 2, Nav2, robot_localization, ros2_controllers) | Anonymous or unclear authorship; content farms; tutorial aggregators |
| **Venue** | Peer-reviewed journal or conference (IEEE, Springer, Elsevier, MDPI with DOI, ION, ICRA/IROS); official documentation site; official repository | Personal blogs, Medium, forum threads (except maintainer answers, level D), video transcripts |
| **Traceability** | DOI, official URL or repository commit; citation details can be confirmed | No way to confirm who wrote it or when |
| **Relevance** | Answers a research question of the topic, including general principles that apply across vehicle types | Off-topic, or only applies to a setting that clearly does not carry over |
| **Currency** | Documentation matches current product and software versions; older papers kept when still the standard reference | Documentation for superseded product versions (unless stated) |
| **Content** | The downloaded file is the real content | Error pages, login walls, abstracts only when the finding needs the full text |

When two sources conflict, the higher evidence level wins, and the conflict is recorded under *Disagreements between sources*.

## 3. Sources

### Foundational references come first
Foundational references are the works a field is built on: standard textbooks, seminal or most-cited papers, survey papers, official standards and primary manufacturer specifications. They are the most important sources in every topic.
- Each topic identifies them before collecting anything else (in `SCOPE.md`), downloads them first, and lists them in the README's *Foundational references* section.
- If no open copy exists after a real search (author pages, university repositories, arXiv), the reference is listed as *not downloaded*, and only what could actually be read is cited.
- Other reputable sources (A–D) are then added to support, update or extend them.

### File formats
| Source type | Saved as | Why |
|---|---|---|
| Papers, textbooks, theses, standards, datasheets | **PDF**, whenever any open PDF exists | Keeps page numbers for citations |
| Web documentation (REV, WPILib, ROS, Nav2…) | Clean markdown or text of the page | These are web pages; there is no PDF |
| Documentation stored as source files (e.g. `.rst`) | The original file | Cleaner than the rendered page |
| Code and configuration | The raw file (`.cpp`, `.py`, `.java`, `.yaml`) | The actual code other robots run |

Every download is checked with `file`: a "PDF" that is really a web page (error, login or bot-check page) is deleted.

### Storage and citation
- Every source is **downloaded** into the topic's `sources/` folder: PDF when available, otherwise the page saved as plain text/markdown, or the raw file for code and configs.
- **File name:** `<author-or-org>_<year>_<short-topic>.<ext>`, lowercase, e.g. `mandow_2007_skid_steer_kinematics.pdf`.
- **Source ID:** `<TOPIC>-S<nn>`, e.g. `C1-S03`. Used for all citations.
- Each source is listed in the topic README's Sources table with: ID, full citation (authors, title, publisher, year), link, date accessed, file name, evidence level.
- Paywalled items are listed with their citation and marked *not downloaded*, and only if an open copy of the content was used.

## 4. Findings
- One fact per bullet, in plain language. A technical term is explained in a few words the first time it is used.
- Every bullet ends with its citation and location: `[C1-S03, p. 12]` or `[C1-S03, section "Velocity Control"]`.
- Numbers are quoted exactly as in the source, with units.
- Where sources disagree, both views are stated with both citations.
- Nothing is stated without a source. Anything that cannot be sourced is dropped, or listed under Open questions.

## 5. Process and verification
Each topic goes through five steps, each done by a separate agent:

| Step | What happens | Output |
|---|---|---|
| 1. Map | Understand the field, break the topic into 6–12 subtopics, identify foundational references | `SCOPE.md` |
| 2. Research | Download foundational references first, then supporting sources; write the findings | `README.md`, `sources/` |
| 3. Gap check | An independent researcher compares the README with the map, looks for anything an expert would expect, and fills the gaps | updates to all three |
| 4. Verify | Two independent reviewers, in parallel: one checks every claim, the other audits every source | `VERIFICATION.md`, `SOURCE_AUDIT.md` |
| 5. Correct | Fix or remove what the reviewer flagged | final `README.md` |

In step 4, the reviewer checks:
1. Every finding is checked against the cited file and location.
2. Each finding gets a status:

| Status | Meaning |
|---|---|
| **Verified** | The source says this, at the cited location |
| **Partly supported** | The source supports part of it, or the wording overstates it |
| **Not supported** | The source does not say this, or the citation is wrong |

3. Source files are checked to be real content (not error or login pages), in the right format, and citation details are checked to be correct.
4. Foundational references are present, and every subtopic in `SCOPE.md` is covered.
5. Results go in `VERIFICATION.md` (claims) and `SOURCE_AUDIT.md` (sources).
6. Findings marked *partly supported* are corrected, and findings marked *not supported* are removed, before the topic is marked complete.

## 6. Topic folder layout
```
topics/<ID>_<name>/
├── SCOPE.md           subtopics, foundational references, search log, gap check
├── README.md          findings (structure in TEMPLATE_TOPIC.md)
├── VERIFICATION.md    reviewer's check of every claim, plus corrections applied
├── SOURCE_AUDIT.md    reviewer's audit of every source, file, format and coverage
└── sources/           downloaded sources
```
