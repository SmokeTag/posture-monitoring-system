# Midterm Measurement Plan — Partial Report

> Working / reference doc. Created **2026-06-18**. Midterm (*entrega parcial*)
> deadline ≈ **2026-06-20** (~2 days).
> Companion to [`decisao-calibracao-vs-limiar-absoluto.md`](decisao-calibracao-vs-limiar-absoluto.md)
> and the project `CLAUDE.md`. Source academic doc: `INTRODUÇÃO_TCC.pdf`.

## TL;DR — where am I?
For the midterm we do **NOT** assemble the full system. We bench-characterize
each sensing channel **independently**, using only hardware already on hand, and
produce plots for a **written report**. Two campaigns:

- **A — IMU upper-back posture** (headline result; reuses the working XIAO sketch)
- **B — FSR402 load** (next item already in the testing plan)

Everything on the buy/build list becomes the report's **"Próximas etapas"**
section, not a blocker.

**▶ Immediate next step (critical path):** add CSV pitch/roll logging to
`firmware/xiao_imu_test`, and write a 6-channel FSR sketch for the Uno. These two
firmware changes unblock every capture below.

---

## Constraints — the "why" of this plan

**Blocked until parts arrive — NOT part of this delivery (this is "Próximas etapas"):**
- [ ] 2nd XIAO nRF52840 (chair MCU) → no ESB chair↔brain link yet
- [ ] batteries (both units) → everything stays USB-tethered (fine at a desk)
- [ ] CD74HC4067 mux → can't read all 10 FSRs at once yet
- [ ] built wearable enclosure → strap the **bare** XIAO to the back instead
- [ ] instrumented chair + final FSR placement → use a bench rig instead

**On hand & verified working:**
- XIAO nRF52840 **Sense** — IMU streaming (`firmware/xiao_imu_test`)
- Arduino **Uno** — single-FSR resistance read (`firmware/uno_fsr_test`)
- **7–10 × FSR402** — Uno reads up to **6 at once** on A0–A5, *no mux needed*
- resistors (10k divider baseline), USB cables

---

## How this maps to the TCC objectives (from `INTRODUÇÃO_TCC.pdf`)

**Problema de pesquisa:** *"como sistemas inteligentes podem ser utilizados para
monitorar e melhorar a ergonomia no ambiente de trabalho, contribuindo para a
prevenção de distúrbios osteomusculares?"*

| TCC objective (PT-BR) | Evidence this plan produces |
|---|---|
| **OE1** — Compreender as variações anatômicas dos usuários | **Re-don / mounting-drift experiment** (Campaign A): the "upright" IMU reading shifts with re-mounting → fixed absolute thresholds misfire → per-user calibration justified. This is the data behind `decisao-calibracao-vs-limiar-absoluto.md`. |
| **OE2** — Desenvolvimento do sistema integrado com sensores para monitoramento postural | IMU pitch/roll + FSR characterization = the **sensing layer characterized channel-by-channel** — *evidência para a viabilidade* de cada canal. ⚠️ This parcial does **not** yet build or validate the *integrated* system (no fusion, chair, or radio); integration is deferred (Próximas etapas). The honest integration argument is already concrete: the IMU **alone** cannot separate slouch (1) from head-down (4) (gap 6.9° vs ≥22° for every other pair) → the chair FSRs are required. |
| **OE4** — Avaliar a eficiência do sistema proposto | Posture-separation plots + FSR saturation curve = a first **feasibility precursor** (single-channel, single-subject). A true *efficiency evaluation* of the proposed system requires the fused build and is deferred (Próximas etapas). |

> **Escopo desta entrega parcial:** OE3 (implementação da comunicação com o
> usuário) e as partes de *integração / eficiência* de OE2/OE4 são
> **deliberadamente adiadas** para a próxima fase — esta parcial caracteriza cada
> canal de sensoriamento **isoladamente**, em bancada. Nenhuma estatística
> inferencial (IC / significância) é reivindicada neste N (1 sujeito, 3 sensores);
> os resultados são de **viabilidade descritiva**, com margens de separação
> *otimistas* (posturas pronunciadas). Limitações consolidadas na seção própria do
> relatório (`analysis/PROJECT_REVIEW.md` e o relatório LaTeX).

**Academic grounding to cite (already in the lit review §2.3–2.4):**
- **RULA** (McAtamney & Corlett, 1993) and **ROSA** (Song & Qu, 2014) score
  posture risk by **trunk/neck inclination angles** → grounds the IMU pitch/roll
  alert thresholds in established ergonomic criteria instead of arbitrary numbers.
- **BackWatch** (Ferreira et al., 2023) monitors trunk + head inclination with
  sensors → direct **prior art** the IMU channel extends. Our novelty: **fusing
  chair pressure with the worn IMU**, which BackWatch does not do.

---

## Campaign A — IMU upper-back posture characterization (do first)

**Goal:** show the IMU can distinguish upright vs. slouched upper-back posture,
and that the per-user "upright" reference drifts with mounting (→ calibration).

**Setup:** strap the bare XIAO Sense to the upper back, just below the neck (its
final location), USB-tethered to the laptop.

**Firmware change:** in `xiao_imu_test`, derive tilt from the accelerometer and
log CSV:
- `pitch = atan2(-ax, sqrt(ay*ay + az*az))`
- `roll  = atan2(ay, az)`
- print `millis,pitch,roll` per sample (exact axis mapping confirmed once mounted)
- Static posture from **accel** has **no drift** → ignore the gyro entirely (gyro
  bias gY ≈ -3°/s only matters when integrating gyro to angle). Note this
  simplification in the report.
- The existing serial `'c'` calibrate stub captures the current pitch/roll as the
  per-user "upright" reference.

**Tag map (`tag` column):** `0`=upright, `1`=forward slouch, `2`=lean L,
`3`=lean R, `4`=head-down, `5`=laid-back (hips forward, torso back),
**`9`=junk/moving (reserved)**. Logging is continuous
once CSV is on; the tag holds until you change it, and you time each posture
(~10 s) yourself — there is no per-tag timer.

**Protocol:**
1. Press **`9` during every transition** between postures — moving contaminates
   the data twice: the angle sweeps through in-between values, and while moving
   the accelerometer reads motion + gravity so the tilt math is briefly wrong.
   Then ~10 s steady per posture:
   `0` upright → `9` → `1` slouch → `9` → `2` lean L → `9` → `3` lean R → `9` →
   `4` head-down → `9` → `5` laid-back. → plot shows clear angular separation =
   detection works.
2. **Re-don experiment (the strong result):** remove and re-attach the sensor
   4–5×, re-send `c` and recapture *upright* each time (`9` between). The
   "upright" pitch/roll drifts several degrees from mounting offset alone →
   proves a fixed threshold misfires → validates calibration (OE1).

**Analysis:** drop all `tag == 9` rows (transitions); optionally also drop the
first ~0.5–1 s after each tag change before averaging each steady segment.

**Deliverables:** pitch/roll-per-posture plot; a deviation metric from reference
(e.g. `sqrt(Δpitch² + Δroll²)`) and a proposed alert angle; re-don drift table.

---

## Campaign B — FSR402 load characterization (do second)

**Goal:** map resistance vs. load, find saturation, confirm "relative pressure /
contact, not calibrated force" framing, pick the divider resistor.

**Firmware:** `uno_fsr_test` prints raw ADC + resistance (ohms) and now has a
**CSV/load-stamp mode** (`'l'`; type the scale reading in kg) for the R-vs-load
curve. The 6-channel weight-map variant is `uno_fsr6_test` (A0–A5).
**Step-by-step procedure:** [`protocolo-caracterizacao-fsr.md`](protocolo-caracterizacao-fsr.md).
Analysis: `analysis/plot_fsr_curve.py` (curve + saturation knee).

**Protocol:**
1. **R-vs-load curve:** FSR on a hard surface, press through a rigid flat puck
   sitting on a **bathroom scale** (scale = applied force in kg, FSR = resistance,
   read both together). Sweep from light touch up to ~body weight.
   → plot R vs. force, find the **knee where it flattens** = saturation.
   → confirms FSRs saturate under seat loads (treat as relative pressure).
2. **Divider choice:** 10k baseline may not be optimal for the sub-saturation
   region of interest — note the dynamic-range trade-off.
3. **6-channel weight map (bonus figure):** 6 FSRs on A0–A5 in a rough seat
   pattern, capture seated vs. empty → left/right + front/back contrast bar chart.
   Zero extra parts (no mux). Strong visual.

**Portability rule:** record **ohms, not raw ADC** — resistance is board-voltage
independent, so Uno data transfers directly to the 3.3 V XIAO later.

**Deliverables:** R-vs-load curve with saturation knee; chosen divider + rationale;
6-channel weight-distribution figure.

---

## Data capture & plotting
- Log serial straight to file: `arduino-cli monitor -p <port> > run.csv`
  (the line-buffering quirk only affects *typed input*, not output logging).
- Plot in Python/matplotlib or a spreadsheet.
- Ports: XIAO is native USB CDC; Uno is typically `/dev/ttyACM0` or `/dev/ttyUSB0`.

## 2-day schedule
- **Day 1 (2026-06-18 → 19):** IMU firmware tweak → capture all postures +
  re-don experiment. Then FSR R-vs-load sweep.
- **Day 2 (2026-06-19 → 20):** 6-channel weight map → reduce all data to plots →
  write up methodology + partial results + "Próximas etapas".

## Report integration (where each result lands)
- **Metodologia:** bench setups for A and B, accel-angle math, ohms-not-ADC rule.
- **Resultados parciais:** posture-separation plot (OE2/OE4), re-don drift (OE1),
  FSR saturation curve, 6-channel weight map.
- **Próximas etapas:** the blocked checklist above (2nd XIAO/ESB, mux, batteries,
  wearable build, chair instrumentation, sensor fusion).

---

## Tracking checklist
- [x] `xiao_imu_test`: add CSV pitch/roll logging (`l` toggle, `0`–`9` tags) — compiles
- [x] Uno: 6-channel FSR sketch (A0–A5) → `firmware/uno_fsr6_test` — compiles
- [x] Capture postures (IMU) — **6 postures × 5 rounds**, run 1 (André, no neck mount);
      `data/2026-06-19_andre_no-neck-mount_own-chair.csv`. Procedure: `protocolo-captura-imu.md`
- [x] Re-don drift experiment (IMU) — 7 re-dons; upright drifts 7.7° pitch / ~21° roll (OE1)
- [ ] FSR R-vs-load sweep + saturation knee
- [ ] 6-channel weight map
- [x] Reduce data → plots — `analysis/` (validator + 6 figures); validated 15 PASS/3 WARN/0 FAIL
- [ ] Re-capture with subtler postures + neck mount + other subjects (true separation margin)
- [ ] Write partial-report sections
