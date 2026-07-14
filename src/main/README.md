# src/main — Active Development

Fresh development base. New work goes here.

## Layout

- `src/main/` — **this folder**: active/new development.
- `src/working_legacy/` — the deprecated previous `main` (the verified web-control
  stack and everything that was in the old `src/main`). Kept for reference and as a
  dependency-intact working copy. **Do not develop here**; treat it as read-only.

## Related branches

- `Working` — locked, self-contained snapshot of the web-control stack (fallback).
- `Legacy` — full archive of the entire repo's prior state (all history preserved).
- `Dev` — this active development line.
- `master` — approved/stable line; approved work from `Dev` merges here.

## Running the legacy web controller

The verified controller still lives under `src/working_legacy/` and runs unchanged
(imports are relative, so the rename did not break anything):

```bash
cd src/working_legacy
python web_control.py
```
