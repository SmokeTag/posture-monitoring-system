# Analysis tools

## Setup (once)

Python 3 with a venv (the repo's `.gitignore` already ignores `.venv/`):

```bash
python3 -m venv .venv
.venv/bin/pip install -r analysis/requirements.txt
```

## Files

- `ble_chair.py` — scans the chair BLE beacon with bleak and reconstructs CSV/events;
  `test_ble_chair.py` tests decoding, clock wrap and transitions. One-way, best effort:
  use the chair button to calibrate. `mostra.py --ble` runs without USB;
  `--ble-address AA:BB:CC:DD:EE:FF` selects a chair. BLE timing tolerates gaps up to
  2 s; USB/replay keep the 0.25 s limit. Use USB for complete captures.

- `live_posture.py` — live view of the chair unit (`firmware/chair`, the brain)
  over serial. Left: body IMU `(dpitch, droll)` + trail, alert circles,
  the chair's decision and what the body reports it applied (chair/fallback,
  age, ✓/≠ against the deciding row, motor: alert or test). Right: the chair from above
  (4 seat + 4 backrest spots filled by FSR load, centre of pressure, L/R and
  front/rear split) + a 10 s trace per channel C8..C15, under the recorded
  postures of the 2026-10-21 captures. UI in PT-BR. `c`/`r` calibrate/clear;
  with `--fsr-placement`, `m` maps the FSRs (press each spot's sensor when
  asked; or click one spot to re-map it). With `--log`, `0`–`9` set the
  posture tag (`0` = transition; the badge times the hold) and the stream is
  saved, with the tag as a last `tag` column and the map / tag names as
  `# fsr map` / `# tag map` lines.
- `mostra.py` — the science-fair version: fullscreen, for visitors (status
  band with the posture's name, deviation target, chair pressure, how it
  works, per-visitor and per-day numbers; a visit = a calibration until the
  seat stays empty). `--replay <capture>` plays a recorded run instead of the
  serial port. How-to (PT-BR): [`../docs/guia-mostra.md`](../docs/guia-mostra.md).
- `test_mostra.py` — `mostra.py`'s visitor logic (categories, visits, day numbers).
- `tags.json` — posture tag names (`{"1": "...", ...}`), defined in capture
  run 1 ([`../docs/protocolo-captura.md`](../docs/protocolo-captura.md)).
- `test_live_posture.py` — `live_posture.py`'s parsing and body-status logic,
  on the real chair firmware's output (builds and runs `firmware/test/test_system`):
  `QT_QPA_PLATFORM=offscreen .venv/bin/python -m unittest discover -s analysis`.
- `captures.py` — loader for the `data/` captures: rows, `#` events, FSR map →
  spot names, tag map, calibrations (calid + 1 s capture window → FSR baseline),
  tag segments and settled samples (2 s after each tag change dropped).
- `fusion.py` — the IMU + FSR alert rule as the chair would run it (`decide()` +
  timing gate, ready to port), replay of all runs, current vs fusion tables and
  leave-one-subject-out search: `.venv/bin/python analysis/fusion.py [--quick]`;
  `--fixture` rewrites the C++ cross-check (`firmware/test/fixtures/`).
  Findings (PT-BR): [`../docs/decisao-fusao-imu-fsr.md`](../docs/decisao-fusao-imu-fsr.md).
- `test_fusion.py` — loader and rule tests (incl. replay ≡ logged firmware alert).
- `chair_map.py` — the spot names, the press-to-map logic and
  `chair_map.json` (channel ↔ spot; written by `live_posture.py --fsr-placement`). How-to (PT-BR):
  [`../docs/guia-live-posture.md`](../docs/guia-live-posture.md).

`archive/` — midterm (June 2026) IMU and FSR402 analysis: [`archive/README.md`](archive/README.md).
