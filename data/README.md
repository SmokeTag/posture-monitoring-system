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

`archive/` — midterm captures (June 2026: bare-XIAO IMU run + FSR402 bench
characterization): [`archive/README.md`](archive/README.md).
