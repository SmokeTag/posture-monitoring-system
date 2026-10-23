# Capture data

Full-system captures (IMU + 8 FSRs) recorded with
`analysis/live_posture.py --log data/<file>.csv`. Schema: the column line
in the file itself (documented in the header of `firmware/chair/chair.ino`;
captures before ESB protocol v2 have the shorter v1 columns, with `led`
instead of the body's `b*` report); the FSR channel ↔ chair spot map is
saved as a `# fsr map` line. The live tool appends the posture tag as a last
`tag` column (`0` = transition, dropped in analysis), with the names as a
`# tag map` line. Capture protocol (PT-BR): [`../docs/protocolo-captura.md`](../docs/protocolo-captura.md).

**FSR map, 2026-10-21 runs:** the chair was mapped facing it, so left/right
came out as the viewer's. The `# fsr map` lines were mirrored on 2026-10-22
(marked by a `# note:` line) and now use the sitter's left/right, like
`analysis/chair_map.json`.

## Run registry

| run | file | date | subject | notes |
|----:|------|------|---------|-------|
| 1 | `2026-10-21_Andre-Monteiro_run1.csv` | 2026-10-21 | André Monteiro | marked series only, 1 pass in order 1→9 (no shuffle/subtle/re-placement) |
| 1 | `2026-10-21_Felipe-godoi_run1.csv` | 2026-10-21 | Felipe Godoi | same; one `3`→`4` mistag fixed (t≈138–160 s); dpitch ≈ +10° drift after tag 6 |
| 1 | `2026-10-21_Thiago-M_run1.csv` | 2026-10-21 | Thiago M | same; back_lower_left (c12) ≈ 0 even upright |
| 1 | `2026-10-21_Vinicius-Beleti_run1.csv` | 2026-10-21 | Vinicius Beleti | same; recalibrated after a 2nd `ereto`; slid forward during tag 9 |
| 1 | `2026-10-21_Thiago-Fita_run1.csv` | 2026-10-21 | Thiago Fita | 2 passes in order (2nd without 8/9), recalibrated between; a 2.2 s `4` mis-press before the first `3` retagged to `0` |
| 1 | `2026-10-21_Ryan-Akira_run1.csv` | 2026-10-21 | Ryan Akira | 2 passes in order, recalibrated between; in pass 2 the `5` is still slid forward from the `4` (mixed posture) |
| 2 | `2026-10-21_Andre-Monteiro_run2.csv` | 2026-10-21 | André Monteiro | 2 passes in order, then stood up + short series (1, 2, 7) on the old calibration, recalibrated + 1, 3, ~3 min untagged; dpitch ≈ −8° drift in pass 1 after the `4`. Logged into Thiago-Fita's file by mistake (append), split out |
| 1 | `2026-10-21_Gustavo-Socce_run1.csv` | 2026-10-21 | Gustavo Socce | short protocol, sheet P1 pass A only (shuffled, `1` between postures); recalibrated before the `5` and the `2`; the `1` after `7` and the `9` kept droll ≈ +10° (alert on the `9`) |
| 1 | `2026-10-21_Rafael-Herleng_run1.csv` | 2026-10-21 | Rafael Herleng | short protocol, sheet P2 pass A without `7`/`3` (5 6 2 9 8 4); recalibrated after the `6`; back_lower_left (c12) ≈ 0 |

`validation/` — hardware validation sessions of the chair's fusion rule
(firmware on the boards, motor active; each file's `#` header has the session
notes):

| run | file | date | subject | notes |
|----:|------|------|---------|-------|
| 1 | `validation/2026-10-22_Andre-Monteiro_run1.csv` | 2026-10-22 | André Monteiro | fusion rule, firmware `c4ae8f2` on both boards, motor active; guided, fixed order 1, 7 (light + stronger), 4, 1, 2, 5, 6, 3, 8, 9, empty chair, collar re-placed + recalibrated, 1, 7, 4, 1 (~16 s holds, no shuffle/subtle). 0 lost, IMU OK throughout. Alert 100 % of the time on 2–6, 0 % on 1, 7, 8, 9; vibration felt and stopping on return upright. Not an unseen subject; 3 s latency not measured; collar/chair details not recorded |
| 1 | `validation/2026-10-22_Lucas-Santos_run1.csv` | 2026-10-22 | Lucas Santos | first unseen subject; firmware `b3c29b1` (proto 5) on both boards, motor active; validation protocol with sheet P3, partial: 1, 5, 4, 8, 1, 2, 6, then ~10 s empty chair (no 3/7/9, pass B, subtle or re-placement), one calibration. 0 lost, IMU OK. Alert 100 % on 2 and 4, 0 % on 1 and empty chair. `5` (dev ≈ 12°) judged bad throughout, but alerted only 12 % (logged; 9 % in the replay): the motor's first ~100–150 ms shake the IMU ±10°, one sample under CLEAR_DEG cleared the alert and the 3 s hold restarted (6×). `6` missed: dpitch only +5.5° (other subjects +10…+28°) with the seat load shifted left; upright reference drifted ~3.5° left by the 2nd `1`. `8`: 3 short LEAN alerts (dpitch ≈ −5.5° at the 5° LEAN_DEG edge) |

`archive/` — midterm captures (June 2026: bare-XIAO IMU run + FSR402 bench
characterization): [`archive/README.md`](archive/README.md).
