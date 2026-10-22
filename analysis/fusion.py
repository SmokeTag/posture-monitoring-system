"""IMU + FSR fusion alert rule for the chair unit, and its evaluation on the captures.

The rule runs on the chair (the brain) once per body sample (20 Hz). Inputs:
the body's (dpitch, droll) from the calibrated upright reference, the 8 FSRs
(raw 12-bit ADC, by spot), and an FSR baseline captured with the IMU reference
(mean of each FSR over the same 1 s calibration window). With the current
collar mounting droll is the sagittal axis (+ = backwards) and dpitch the
lateral one (+ = right). Per sample (decide(); C++ twin: firmware/libraries/PostureLink/Fusion.h):

  seat   = sum of the 4 seat FSRs                  (raw counts)
  seat < EMPTY_RAW                                -> EMPTY: no-one seated, gate reset
  rear   = (seat_rear_left + seat_rear_right) / baseline of the same   [if baseline >= BASE_MIN]
  rear < SLUMP_FRAC                               -> SLUMP (hips slid forward: escorregado)
  droll > 0 and rear >= RECLINE_FRAC              -> backward tilt ignored (RECLINE_OK)
  dev    = hypot(dpitch, droll or 0)
  dev > ALERT_DEG                                 -> IMU_DEV
  |dpitch| >= LEAN_DEG and seat L/R shift (vs baseline) >= LEAN_LR towards it -> LEAN
  bad = any of SLUMP / IMU_DEV / LEAN; ok (alert may clear) = not bad and dev < CLEAR_DEG

Timing gate (same semantics as PostureLink AlertGate): bad continuously for
HOLD_S -> alert; while alerted, an ok sample clears it; neither keeps the
state. EMPTY, an invalid IMU sample or a new calibration reset it.

CURRENT (the firmware's rule today) is the same code with the FSR terms off.

    .venv/bin/python analysis/fusion.py            # tables: current vs fusion, LOSO
    .venv/bin/python analysis/fusion.py --quick    # skip the LOSO grid search
    .venv/bin/python analysis/fusion.py --fixture  # regenerate the C++ cross-check fixture
"""

from __future__ import annotations

import argparse
import itertools
import math
from dataclasses import dataclass, field, replace

import numpy as np
import pandas as pd

import captures as C

INF = math.inf
BAD_TAGS = [2, 3, 4, 5, 6]                    # should alert
OK_TAGS = [1, 7]                              # must not alert
LEG_TAGS = [8, 9]                             # undecided: counted as false alarms for now

# reason codes (also the C++ enum)
OK, IMU_DEV, SLUMP, LEAN, RECLINE_OK, EMPTY, INVALID = range(7)
CODES = ["OK", "IMU_DEV", "SLUMP", "LEAN", "RECLINE_OK", "EMPTY", "INVALID"]

# spot indices into the 8-FSR vector (captures.SPOTS order)
SFL, SFR, SRL, SRR, BLL, BLR, BUL, BUR = range(8)


@dataclass(frozen=True)
class Params:
    alert_deg: float = 10.0                   # IMU deviation that is bad (deg)
    clear_deg: float = 7.0                    # alert clears below this (deg)
    hold_s: float = 3.0                       # bad this long -> alert
    slump_frac: float = 0.3                   # seat rear / baseline below this = slid forward
    recline_frac: float = 0.7                 # seat rear / baseline at least this: droll > 0 ignored
    lean_deg: float = 5.0                     # |dpitch| for an FSR-confirmed lean (deg)
    lean_lr: float = 0.08                     # seat (L-R)/(L+R) shift vs baseline towards the lean
    empty_raw: float = 2000.0                 # seat sum below this = no-one seated (raw counts)
    base_min: float = 1000.0                  # a 2-sensor baseline below this is unusable (raw)


CURRENT = Params(slump_frac=-INF, recline_frac=INF, lean_deg=INF, empty_raw=-INF)
FUSION = Params()


@dataclass(frozen=True)
class Baseline:
    """Captured at calibration from the per-spot FSR means."""
    rear: float                               # seat_rear_left + seat_rear_right (raw)
    lr: float                                 # seat (L-R)/(L+R)
    rear_ok: bool
    lr_ok: bool

    @staticmethod
    def of(means, p: Params = FUSION) -> "Baseline":
        b = [float(x) for x in means]
        rear = b[SRL] + b[SRR]
        left, right = b[SFL] + b[SRL], b[SFR] + b[SRR]
        lr_ok = left + right >= 2 * p.base_min
        return Baseline(rear, (left - right) / (left + right) if lr_ok else 0.0,
                        rear >= p.base_min, lr_ok)


# --- the decision: one sample, no state (port this 1:1) ---

def decide(dp: float, dr: float, f, base: Baseline, p: Params) -> tuple[int, bool, bool]:
    """-> (reason code, bad, ok). f = the 8 FSRs (raw), captures.SPOTS order."""
    if not (math.isfinite(dp) and math.isfinite(dr)):
        return INVALID, False, False
    seat = f[SFL] + f[SFR] + f[SRL] + f[SRR]
    if seat < p.empty_raw:
        return EMPTY, False, False
    rear = (f[SRL] + f[SRR]) / base.rear if base.rear_ok else 1.0
    slump = base.rear_ok and rear < p.slump_frac
    recline = base.rear_ok and dr > 0 and rear >= p.recline_frac
    dev = math.hypot(dp, 0.0 if recline else dr)
    lean = False
    if base.lr_ok and abs(dp) >= p.lean_deg:
        lr = (f[SFL] + f[SRL] - f[SFR] - f[SRR]) / seat - base.lr   # + = load moved left
        lean = (lr if dp < 0 else -lr) >= p.lean_lr                   # dpitch < 0 = leaning left
    if slump:
        return SLUMP, True, False
    if dev > p.alert_deg:
        return IMU_DEV, True, False
    if lean:
        return LEAN, True, False
    code = RECLINE_OK if recline and math.hypot(dp, dr) > p.alert_deg else OK
    return code, False, dev < p.clear_deg


def hold_ms(p: Params) -> int:
    return int(round(p.hold_s * 1000))


class Gate:
    """bad for hold_ms -> alert; ok clears it; reset() on EMPTY / invalid / new calibration.
    Times in integer ms (chair millis()), as on the MCU."""

    def __init__(self, hold_ms: int):
        self.hold_ms = hold_ms
        self.reset()

    def reset(self):
        self.alert = False
        self.over_since = None

    def update(self, bad: bool, ok: bool, t: int) -> bool:
        if bad:
            if self.over_since is None:
                self.over_since = t
            if not self.alert and t - self.over_since >= self.hold_ms:
                self.alert = True
        else:
            self.over_since = None
            if self.alert and ok:
                self.alert = False
        return self.alert


class FusionRule:
    """Stateful rule as the chair runs it: calibrate() once, then step() per sample."""

    def __init__(self, p: Params = FUSION):
        self.p, self.gate, self.base = p, Gate(hold_ms(p)), None

    def calibrate(self, fsr_means):
        self.base = Baseline.of(fsr_means, self.p)
        self.gate.reset()

    def step(self, t: int, dp: float, dr: float, f) -> tuple[bool, int]:
        if self.base is None:
            return False, INVALID
        code, bad, ok = decide(dp, dr, f, self.base, self.p)
        if code in (EMPTY, INVALID):
            self.gate.reset()
            return False, code
        return self.gate.update(bad, ok, t), code


# --- replay on a capture ---

@dataclass
class Inputs:
    """Per-row arrays of one run, as the chair would have seen them."""
    run: C.Run
    t: np.ndarray                             # s (chair millis)
    t_ms: np.ndarray                          # the same, integer ms (the gate's clock)
    dp: np.ndarray
    dr: np.ndarray
    f: np.ndarray                             # (n, 8) raw, SPOTS order
    base: np.ndarray                          # (n, 8) baseline means of the calibration in effect
    calibrated: np.ndarray                    # chair has a reference
    recal: np.ndarray                         # first row of a new calibration (gate reset)
    frozen: np.ndarray                        # capture window before the event: gate not updated
    segs: list = field(default_factory=list)  # tagged segments (Seg), for scoring


def inputs(run: C.Run) -> Inputs:
    r = run.rows
    n = len(r)
    base = np.full((n, 8), np.nan)
    calid = r["calid"].to_numpy()
    for cal in run.cals:
        m = calid == cal.calid
        base[m] = [run.fsr_baseline(cal)[s] for s in C.SPOTS]
    recal = np.zeros(n, bool)
    frozen = np.zeros(n, bool)
    for cal in run.cals:
        if cal.in_file:
            recal[cal.row] = True
            frozen[cal.start:cal.row] = True
    ok_imu = (r["imu"] == 0).to_numpy() if "imu" in r else np.ones(n, bool)
    dp = np.where(ok_imu, r["dpitch"].to_numpy(float), np.nan)
    dr = np.where(ok_imu, r["droll"].to_numpy(float), np.nan)
    t_ms = r["t_ms"].to_numpy(np.int64)
    return Inputs(run, t_ms / 1000.0, t_ms, dp, dr, run.fsr().to_numpy(float),
                  base, (r["cal"] == 1).to_numpy(), recal, frozen, tagged_segments(r))


def replay(x: Inputs, p: Params) -> tuple[np.ndarray, np.ndarray]:
    """Sample by sample with FusionRule (the reference). -> (alert, code) per row."""
    rule = FusionRule(p)
    n = len(x.t)
    alert, code = np.zeros(n, bool), np.full(n, INVALID)
    cur = None
    for i in range(n):
        if x.frozen[i]:
            alert[i] = rule.gate.alert
            continue
        if not x.calibrated[i] or np.isnan(x.base[i, 0]):
            rule.gate.reset()
            cur = None
            continue
        if x.recal[i] or cur is None or not np.array_equal(x.base[i], cur):
            cur = x.base[i]
            rule.calibrate(cur)
        alert[i], code[i] = rule.step(int(x.t_ms[i]), x.dp[i], x.dr[i], x.f[i])
    return alert, code


# --- cross-check fixture for the C++ port (firmware/test/test_fusion.cpp) ---

FIXTURE = C.ROOT / "firmware" / "test" / "fixtures" / "fusion_cases.txt"
FIXTURE_TAGS = [4, 7, 5]                       # first segment of each, per run (slump, recline, lean)
FIXTURE_ROWS = (40, 30)                         # rows before / from the start


def fixture_lines(xs: list[Inputs], p: Params = FUSION) -> list[str]:
    """Real log excerpts replayed by FusionRule: per run, the first segment of each
    FIXTURE_TAGS and the first empty-seat stretch. Per excerpt 'B' = baseline means by
    spot + gate state at its start (alert, ms since the hold started or -1); per row
    'S' = ms from the excerpt start, dpitch, droll, 8 FSRs by spot, reason code, alert."""
    out = [f"# {FIXTURE.name}: generated by analysis/fusion.py --fixture; do not edit",
           "# B base[8 by spot] alert held_ms   |   S dt_ms dpitch droll f[8 by spot] reason alert"]
    for x in xs:
        rule, cur, n = FusionRule(p), None, len(x.t)
        state, res = [None] * n, [None] * n       # gate before the row, (alert, code) after
        for i in range(n):
            g = rule.gate
            state[i] = (g.alert, -1 if g.over_since is None else int(x.t_ms[i] - g.over_since))
            if x.frozen[i]:
                continue
            if not x.calibrated[i] or np.isnan(x.base[i, 0]):
                rule.gate.reset(); cur = None
                continue
            if x.recal[i] or cur is None or not np.array_equal(x.base[i], cur):
                cur = x.base[i]
                rule.calibrate(cur)
            res[i] = rule.step(int(x.t_ms[i]), x.dp[i], x.dr[i], x.f[i])
        first = {}
        for sg in x.segs:
            first.setdefault(sg.tag, sg.i0)
        starts = [first[t] - FIXTURE_ROWS[0] for t in FIXTURE_TAGS if t in first]
        empty = [i for i in range(n) if res[i] and res[i][1] == EMPTY]
        if empty:
            starts.append(empty[0] - FIXTURE_ROWS[0])
        for i0 in sorted(starts):
            rows = range(i0, min(i0 + sum(FIXTURE_ROWS), n))
            if i0 < 0 or any(x.recal[i] or res[i] is None for i in rows) \
                    or np.any(x.base[i0] != x.base[rows[-1]]):
                continue                          # keep excerpts within one calibration
            out.append(f"# {x.run.name} rows {i0}..{rows[-1]}")
            out.append("B " + " ".join(f"{v:.2f}" for v in x.base[i0])
                       + f" {int(state[i0][0])} {state[i0][1]}")
            for i in rows:
                f = " ".join(str(int(v)) for v in x.f[i])
                out.append(f"S {x.t_ms[i] - x.t_ms[i0]} {x.dp[i]:.1f} {x.dr[i]:.1f} {f} "
                           f"{res[i][1]} {int(res[i][0])}")
    return out


def decide_v(x: Inputs, p: Params):
    """decide() over all rows at once (for the grid search). -> code, bad, ok."""
    f, b = x.f, x.base
    seat = f[:, :4].sum(1)
    brear = b[:, SRL] + b[:, SRR]
    bl, br = b[:, SFL] + b[:, SRL], b[:, SFR] + b[:, SRR]
    rear_ok = brear >= p.base_min
    lr_ok = bl + br >= 2 * p.base_min
    with np.errstate(invalid="ignore", divide="ignore"):
        rear = np.where(rear_ok, (f[:, SRL] + f[:, SRR]) / brear, 1.0)
        blr = np.where(lr_ok, (bl - br) / (bl + br), 0.0)
        lr = (f[:, SFL] + f[:, SRL] - f[:, SFR] - f[:, SRR]) / seat - blr
    slump = rear_ok & (rear < p.slump_frac)
    recline = rear_ok & (x.dr > 0) & (rear >= p.recline_frac)
    dev = np.hypot(x.dp, np.where(recline, 0.0, x.dr))
    lean = lr_ok & (np.abs(x.dp) >= p.lean_deg) & (np.where(x.dp < 0, lr, -lr) >= p.lean_lr)
    invalid = ~(np.isfinite(x.dp) & np.isfinite(x.dr))
    empty = ~invalid & (seat < p.empty_raw)
    live = ~invalid & ~empty
    bad = live & (slump | (dev > p.alert_deg) | lean)
    ok = live & ~bad & (dev < p.clear_deg)
    code = np.select([invalid, empty, slump, dev > p.alert_deg, lean,
                      recline & (np.hypot(x.dp, x.dr) > p.alert_deg)],
                     [INVALID, EMPTY, SLUMP, IMU_DEV, LEAN, RECLINE_OK], OK)
    return code, bad, ok


def replay_v(x: Inputs, p: Params) -> np.ndarray:
    """Vectorised equivalent of replay()'s alert (checked equal in the tests)."""
    code, bad, ok = decide_v(x, p)
    n = len(bad)
    usable = x.calibrated & ~np.isnan(x.base[:, 0])
    prev_usable = np.r_[False, usable[:-1]]
    changed = np.r_[True, np.any(x.base[1:] != x.base[:-1], 1)]
    newbase = usable & ~x.frozen & (x.recal | ~prev_usable | changed)
    live = usable & ~x.frozen
    run_bad = live & bad
    idx = np.arange(n)
    first = run_bad & (~np.r_[False, run_bad[:-1]] | newbase)
    start = np.maximum.accumulate(np.where(first, idx, 0))
    held = run_bad & (x.t_ms - x.t_ms[start] >= hold_ms(p))
    ev = np.full(n, np.nan)                                 # nan = state unchanged
    ev[~x.frozen & ~usable] = 0
    ev[live & ((code == EMPTY) | (code == INVALID))] = 0
    ev[live & ok] = 0
    ev[newbase] = 0                                         # reset, then this sample can't be held yet
    ev[held & ~newbase] = 1
    return pd.Series(ev).ffill().fillna(0).to_numpy() == 1


# --- evaluation ---

@dataclass
class Seg:
    tag: int
    i0: int                                   # rows [i0, i1) of the tagged segment
    i1: int
    settled: np.ndarray                       # its settled rows (scored)


def tagged_segments(r: pd.DataFrame) -> list[Seg]:
    tag, seg, st = r["tag"].to_numpy(), r["seg"].to_numpy(), r["settled"].to_numpy()
    edges = np.flatnonzero(np.diff(seg)) + 1
    out = []
    for i0, i1 in zip(np.r_[0, edges], np.r_[edges, len(seg)]):
        idx = np.arange(i0, i1)[st[i0:i1]]
        if tag[i0] != 0 and len(idx):
            out.append(Seg(int(tag[i0]), int(i0), int(i1), idx))
    return out


def seg_table(x: Inputs, alert: np.ndarray) -> pd.DataFrame:
    """Per tagged segment: fraction of settled samples alerted (frac, over n), fired at
    all within the segment, latency from its start (s; 0 = already on)."""
    out = []
    for s in x.segs:
        on = np.flatnonzero(alert[s.i0:s.i1])
        out.append((x.run.subject, x.run.name, s.tag, alert[s.settled].mean(), len(on) > 0,
                    x.t[s.i0 + on[0]] - x.t[s.i0] if len(on) else np.nan, len(s.settled)))
    return pd.DataFrame(out, columns=["subject", "run", "tag", "frac", "fired", "latency", "n"])


def objective(segs: pd.DataFrame) -> float:
    """Balanced segment score: bad segments' alerted fraction + ok/legs segments' quiet fraction."""
    bad = segs[segs["tag"].isin(BAD_TAGS)]["frac"].mean()
    ok = 1 - segs[segs["tag"].isin(OK_TAGS + LEG_TAGS)]["frac"].mean()
    return (bad + ok) / 2


def evaluate(xs: list[Inputs], p: Params) -> pd.DataFrame:
    return pd.concat([seg_table(x, replay_v(x, p)) for x in xs], ignore_index=True)


# Candidate values; "off" is the axis end that disables the term. clear = alert - 3°.
GRID = dict(alert_deg=[8.0, 9.0, 10.0, 11.0, 12.0],
            slump_frac=[-INF, 0.2, 0.3, 0.4, 0.5],
            recline_frac=[0.5, 0.6, 0.7, 0.8, 0.9, INF],
            lean_deg=[5.0, 6.0, 7.0, 8.0, INF],
            lean_lr=[0.03, 0.05, 0.08])
SHAPE = tuple(len(v) for v in GRID.values())


def grid() -> list[Params]:
    return [replace(FUSION, clear_deg=v[0] - 3, **dict(zip(GRID, v)))
            for v in itertools.product(*GRID.values())]


def search(xs: list[Inputs], ps: list[Params]) -> pd.DataFrame:
    """Segment table of every candidate, once; folds then just filter by subject."""
    return pd.concat([evaluate(xs, p).assign(k=k) for k, p in enumerate(ps)], ignore_index=True)


def select(score: pd.Series) -> int:
    """Best candidate after averaging each score with its grid neighbours (+-1 step on
    every axis): the score is flat over wide plateaus, so this picks a plateau's middle
    instead of whichever tied edge comes first."""
    a = score.sort_index().to_numpy().reshape(SHAPE)
    pad = np.pad(a, 1, constant_values=np.nan)
    acc, cnt = np.zeros(SHAPE), np.zeros(SHAPE)
    for off in itertools.product(range(3), repeat=len(SHAPE)):
        v = pad[tuple(slice(o, o + n) for o, n in zip(off, SHAPE))]
        acc += np.nan_to_num(v)
        cnt += ~np.isnan(v)
    return int(np.argmax(acc / cnt))


def scores(segs: pd.DataFrame) -> pd.Series:
    return segs.groupby("k")[["tag", "frac"]].apply(objective)


def loso(segs_all: pd.DataFrame, ps: list[Params]) -> tuple[pd.DataFrame, dict]:
    """Choose on 7 subjects, test on the 8th. -> held-out segment rows, chosen params per subject."""
    held, chosen = [], {}
    for subj in sorted(segs_all["subject"].unique()):
        k = select(scores(segs_all[segs_all["subject"] != subj]))
        chosen[subj] = ps[k]
        held.append(segs_all[(segs_all["subject"] == subj) & (segs_all["k"] == k)])
    return pd.concat(held, ignore_index=True), chosen


# --- report ---

def tag_name(t: int) -> str:
    return f"{t} {C_TAGS.get(t, '?')}"


C_TAGS: dict[int, str] = {}


def time_table(segs: pd.DataFrame) -> pd.DataFrame:
    """% of settled time alerted, tag x subject (settled samples)."""
    w = segs.assign(on=segs["frac"] * segs["n"])
    g = w.groupby(["tag", "subject"])
    t = (g["on"].sum() / g["n"].sum() * 100).unstack("subject")
    tot = w.groupby("tag")["on"].sum() / w.groupby("tag")["n"].sum() * 100
    t.insert(0, "ALL", tot)
    t.index = [tag_name(i) for i in t.index]
    return t.round(0)


def summary(segs: pd.DataFrame) -> pd.DataFrame:
    """Per tag: segments fired / total, median latency (s), % time alerted."""
    out = {}
    for tag, d in segs.groupby("tag"):
        out[tag_name(tag)] = {"fired": f"{int(d['fired'].sum())}/{len(d)}",
                              "lat_med_s": round(d["latency"].median(), 1) if d["fired"].any() else np.nan,
                              "time_%": round((d["frac"] * d["n"]).sum() / d["n"].sum() * 100)}
    return pd.DataFrame(out).T


def headline(segs: pd.DataFrame) -> str:
    bad = segs[segs["tag"].isin(BAD_TAGS)]
    ok = segs[segs["tag"].isin(OK_TAGS)]
    legs = segs[segs["tag"].isin(LEG_TAGS)]
    pct = lambda d: (d["frac"] * d["n"]).sum() / d["n"].sum() * 100
    return (f"bad segments fired {bad['fired'].sum()}/{len(bad)} ({bad['fired'].mean():.0%}), "
            f"bad time alerted {pct(bad):.0f}% | ok (1,7) time alerted {pct(ok):.0f}%, "
            f"segments fired {ok['fired'].sum()}/{len(ok)} | legs (8,9) time alerted {pct(legs):.0f}% "
            f"| score {objective(segs):.3f}\n  score per subject: "
            + ", ".join(f"{k} {v:.2f}" for k, v in
                        segs.groupby("subject")[["tag", "frac"]].apply(objective).items()))


def fmt(p: Params) -> str:
    lean = "off" if p.lean_deg == INF else f"{p.lean_deg:g}°/{p.lean_lr:g}"
    return (f"alert {p.alert_deg:g}°/clear {p.clear_deg:g}°, slump<{p.slump_frac:g}, "
            f"recline>={p.recline_frac:g}, lean {lean}")


def legs_report(xs: list[Inputs]) -> pd.DataFrame:
    """How far crossed legs move the seat FSRs from ereto (per subject, settled medians)."""
    rows = []
    for x in xs:
        r = x.run.rows
        m = r["settled"].to_numpy() & np.isfinite(x.base[:, 0])
        f, b = x.f[m], x.base[m]
        seat = f[:, :4].sum(1)
        lr = (f[:, SFL] + f[:, SRL] - f[:, SFR] - f[:, SRR]) / seat
        blr = (b[:, SFL] + b[:, SRL] - b[:, SFR] - b[:, SRR]) / b[:, :4].sum(1)
        front = (f[:, SFL] + f[:, SFR]) / (b[:, SFL] + b[:, SFR])
        rear = (f[:, SRL] + f[:, SRR]) / (b[:, SRL] + b[:, SRR])
        d = pd.DataFrame({"subject": x.run.subject, "tag": r["tag"].to_numpy()[m],
                          "dLR": lr - blr, "front": front, "rear": rear})
        rows.append(d[d["tag"].isin([1, 8, 9])])
    d = pd.concat(rows)
    return d.groupby(["subject", "tag"]).median().unstack("tag").round(2)


def empty_report(xs: list[Inputs], p: Params) -> pd.DataFrame:
    """Rows with no-one seated (seat sum < empty_raw): how many, and alerted by current / fusion."""
    out = []
    for x in xs:
        empty = x.f[:, :4].sum(1) < p.empty_raw
        out.append({"run": x.run.name, "empty_s": empty.sum() / 20,
                    "current_alert_s": (empty & replay_v(x, CURRENT)).sum() / 20,
                    "fusion_alert_s": (empty & replay_v(x, p)).sum() / 20})
    return pd.DataFrame(out).set_index("run")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--quick", action="store_true", help="skip the LOSO grid search")
    ap.add_argument("--fixture", action="store_true",
                    help=f"write the C++ cross-check fixture ({FIXTURE.relative_to(C.ROOT)}) and exit")
    args = ap.parse_args()
    if args.fixture:
        FIXTURE.parent.mkdir(exist_ok=True)
        FIXTURE.write_text("\n".join(fixture_lines([inputs(r) for r in C.runs()])) + "\n")
        print(f"wrote {FIXTURE}")
        return 0
    pd.set_option("display.width", 200)
    pd.set_option("display.max_columns", 20)
    runs = C.runs()
    C_TAGS.update(runs[0].tags)
    xs = [inputs(r) for r in runs]

    print("== sanity: replayed current rule vs the logged `alert` column")
    for x in xs:
        a = replay_v(x, CURRENT)
        logged = x.run.rows["alert"].to_numpy() == 1
        i0 = int(np.argmax(~logged))              # the state before logging is unknown
        print(f"  {x.run.name:22s} {np.mean(a[i0:] == logged[i0:]):.2%} rows agree")

    res = {"current": evaluate(xs, CURRENT), "fusion": evaluate(xs, FUSION)}
    for name, p in [("current", CURRENT), ("fusion", FUSION)]:
        s = res[name]
        print(f"\n== {name}: {fmt(p)}, hold {p.hold_s:g} s\n  {headline(s)}")
        print(summary(s).to_string())
        print(f"\n  % of settled time alerted, tag x subject ({name})")
        print(time_table(s).to_string())

    print("\n== crossed legs vs ereto, seat FSRs (settled medians): dLR = seat (L-R)/(L+R) "
          "shift from baseline, front/rear = seat front/rear sum / baseline")
    print(legs_report(xs).to_string())
    print("\n== no-one seated (seat sum < %g raw): seconds per run" % FUSION.empty_raw)
    print(empty_report(xs, FUSION).to_string())

    if args.quick:
        return 0
    ps = grid()
    print(f"\n== LOSO over {len(ps)} candidates (grid: {', '.join(GRID)}; clear = alert - 3°)")
    all_segs = search(xs, ps)
    held, chosen = loso(all_segs, ps)
    for subj, p in chosen.items():
        print(f"  without {subj:16s} -> {fmt(p)}")
    print(f"\n  held-out (each subject scored with params chosen without it):\n  {headline(held)}")
    print(summary(held).to_string())
    print("\n  % of settled time alerted, tag x subject (LOSO held-out)")
    print(time_table(held).to_string())
    k_all = select(scores(all_segs))
    print(f"\n  best on all 8 subjects (in-sample): {fmt(ps[k_all])}")
    print(f"  {headline(all_segs[all_segs['k'] == k_all])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
