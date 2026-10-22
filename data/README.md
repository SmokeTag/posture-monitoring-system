# Capture data

Full-system captures (IMU + 8 FSRs) recorded with
`analysis/live_posture.py --log data/<file>.csv`. Schema: the column line
in the file itself (documented in the header of `firmware/chair/chair.ino`;
captures before ESB protocol v2 have the shorter v1 columns, with `led`
instead of the body's `b*` report); the FSR channel ↔ chair spot map is
saved as a `# fsr map` line. The live tool appends the posture tag as a last
`tag` column (`0` = transition, dropped in analysis), with the names as a
`# tag map` line. Capture protocol (PT-BR): [`../docs/protocolo-captura.md`](../docs/protocolo-captura.md).

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

`archive/` — midterm captures (June 2026: bare-XIAO IMU run + FSR402 bench
characterization): [`archive/README.md`](archive/README.md).
