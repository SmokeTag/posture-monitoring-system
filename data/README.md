# Capture data

Full-system captures (IMU + 8 FSRs) recorded with
`analysis/live_posture.py --log data/<file>.csv`. Schema: header of
`firmware/chair/chair.ino`; the FSR channel ↔ chair spot map is saved as a
`# fsr map` line. Capture protocol: to be written (TASKS.md, task 4).

## Run registry

| run | file | date | subject | notes |
|----:|------|------|---------|-------|

`archive/` — midterm captures (June 2026: bare-XIAO IMU run + FSR402 bench
characterization): [`archive/README.md`](archive/README.md).
