# Capture data — IMU posture runs

Raw serial captures from `firmware/xiao_imu_test/xiao_imu_test.ino`. One CSV per
capture run. The procedure that produces these files:
[`../docs/protocolo-captura-imu.md`](../docs/protocolo-captura-imu.md).

## File naming

```
YYYY-MM-DD_<subject>_<mount>_<chair>.csv
```

- `subject` — short name (e.g. `andre`)
- `mount` — `no-neck-mount` (bare XIAO strapped on) or `neck-mount`
- `chair` — `own-chair`, `office-chair-A`, …
- Add `-run2`, `-run3` if more than one capture in a day with the same setup.

The analysis scripts default to the **newest** file here (ISO dates sort
chronologically); override with `--csv data/<file>.csv`.

## Schema

`t_ms,tag,cal,pitch,roll,dpitch,droll,ax,ay,az,gx,gy,gz` — see
[`../analysis/postures.py`](../analysis/postures.py) for full column docs.

Tag map: `0` upright · `1` forward slouch · `2` lean left · `3` lean right ·
`4` head-down · `5` reclined/laid-back · `9` transition/moving (dropped in
analysis). `#`-prefixed lines are comments (calibration events, tag stamps).

## Run registry

| run | file | date | subject | mount | chair | notes |
|----:|------|------|---------|-------|-------|-------|
| 1 | `2026-06-19_andre_no-neck-mount_own-chair.csv` | 2026-06-19 | André Thomas Monteiro | none (bare XIAO) | own office chair | 6 postures × 5 rounds + 7 re-don calibrations. Validated **15 PASS / 3 WARN / 0 FAIL** (`../analysis/VALIDATION.md`). Postures performed *pronounced* → separation is optimistic. First calibration was an immediate double-press. |
