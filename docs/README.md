# docs/ — index

Numbered, one job per file. Same system as BARQ-Rebuild (v2); adopted for v1
by D-007.

| File | Job |
|---|---|
| [00_OVERVIEW.md](00_OVERVIEW.md) | what the robot is, architecture, stage roadmap |
| [01_STATUS.md](01_STATUS.md) | where we are: snapshot, done-list, how-to-run, next |
| [02_DECISIONS.md](02_DECISIONS.md) | ADR log, newest first, `D-NNN`: context → call → why |
| [03_CHANGELOG.md](03_CHANGELOG.md) | dated entries, newest first: what concretely changed |
| [04_OPEN_QUESTIONS.md](04_OPEN_QUESTIONS.md) | `Q-NNN` pending ambiguities (resolved entries stay) |
| [05_RESEARCH_LOG.md](05_RESEARCH_LOG.md) | why each change was an improvement — the publication record |
| [HANDOFF.md](HANDOFF.md) | cold-start bootstrap: read order, what is PROVEN, frontier |
| `research/` | dated one-off reports (created when the first report lands) |

Read order for a cold start: `01 → 00 → 02 → 04 → 05`.

Artifacts (sim CSVs, snapshots, plots) live OUTSIDE the repo in
`~/barq_v1/artifacts/` — referenced by docs, never committed.
