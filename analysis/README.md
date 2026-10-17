# Analysis tools

## Setup (once)

Python 3 with a venv (the repo's `.gitignore` already ignores `.venv/`):

```bash
python3 -m venv .venv
.venv/bin/pip install -r analysis/requirements.txt
```

## Files

- `live_posture.py` — live view of the chair unit (`firmware/chair`, the brain)
  over serial. Left: body IMU `(dpitch, droll)` + trail, alert circles,
  alert/LED state. Right: the chair from above
  (4 seat + 4 backrest spots filled by FSR load, centre of pressure, L/R and
  front/rear split) + a 10 s trace per channel C8..C15. `c`/`r` calibrate/clear;
  `m` maps the FSRs (press each spot's sensor when asked; or click one spot to
  re-map it). `--log` also saves the stream (with the map as a `# fsr map` line).
- `chair_map.py` — the spot names, the press-to-map logic and
  `chair_map.json` (channel ↔ spot; written by `live_posture.py`). How-to (PT-BR):
  [`../docs/guia-live-posture.md`](../docs/guia-live-posture.md).

`archive/` — midterm (June 2026) IMU and FSR402 analysis: [`archive/README.md`](archive/README.md).
