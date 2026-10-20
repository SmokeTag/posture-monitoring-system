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

`archive/` — midterm captures (June 2026: bare-XIAO IMU run + FSR402 bench
characterization): [`archive/README.md`](archive/README.md).
