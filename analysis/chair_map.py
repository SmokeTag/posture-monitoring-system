"""Chair FSR map: which mux channel (C8..C15) sits at which seat/backrest spot.

The chair PCB wires the 8 FSRs to C8..C15 in whatever order the cables ended
up; this module names the 4 seat + 4 backrest positions and builds the map
interactively — press each sensor when asked and the channel whose conductance
jumps is assigned (`MapSession`). live_posture.py drives it and keeps the result in
chair_map.json. Pure logic, no plotting.

Conductance G = 1/R (µS) is the load variable throughout: it is ~linear in
force for the FSR402 (archive/FSR_FINDINGS.md §3), and an open FSR is simply G = 0.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

CHANNELS = list(range(8, 16))                 # mux channels with an FSR
MAP_PATH = Path(__file__).with_name("chair_map.json")

# slot -> (plate, short label, x, y); x: 0 = sitter's left .. 1 = right,
# y: seat 0 = front edge .. 1 = rear, backrest 0 = lower .. 1 = upper
SLOTS = {
    "seat_rear_left":   ("seat", "rear L", 0, 1),
    "seat_rear_right":  ("seat", "rear R", 1, 1),
    "seat_front_left":  ("seat", "front L", 0, 0),
    "seat_front_right": ("seat", "front R", 1, 0),
    "back_upper_left":  ("back", "upper L", 0, 1),
    "back_upper_right": ("back", "upper R", 1, 1),
    "back_lower_left":  ("back", "lower L", 0, 0),
    "back_lower_right": ("back", "lower R", 1, 0),
}
WIZARD_ORDER = ["seat_rear_left", "seat_rear_right", "seat_front_left",
                "seat_front_right", "back_lower_left", "back_lower_right",
                "back_upper_left", "back_upper_right"]

RISE_US = 50.0       # press = G this far above baseline (open -> R below ~20 kΩ)
DOMINANCE = 2.0      # ... and this many times the runner-up's rise
HOLD_S = 0.4         # ... continuously for this long
RELEASE_US = 25.0    # released = back within this of the baseline
RELEASE_S = 0.3
SETTLE_S = 0.6       # baseline window before each press (hands off)


def slot_name(slot: str) -> str:
    plate, label, _, _ = SLOTS[slot]
    return f"{'seat' if plate == 'seat' else 'backrest'} {label}"


def ohms_to_us(r: float) -> float:
    """Firmware ohms (-1 = open) -> conductance in µS."""
    return 1e6 / r if r > 0 else 0.0


def load_map(path: Path = MAP_PATH) -> dict[str, int]:
    """slot -> channel; unknown slots / channels and duplicates are dropped."""
    try:
        raw = json.loads(path.read_text())
    except (OSError, ValueError):
        return {}
    m: dict[str, int] = {}
    for slot, ch in raw.items():
        if slot in SLOTS and ch in CHANNELS and ch not in m.values():
            m[slot] = ch
    return m


def save_map(m: dict[str, int], path: Path = MAP_PATH) -> None:
    ordered = {s: m[s] for s in SLOTS if s in m}
    path.write_text(json.dumps(ordered, indent=2) + "\n")


def assign(m: dict[str, int], slot: str, ch: int) -> dict[str, int]:
    """Put `ch` at `slot`; if `ch` was elsewhere, that slot gets `slot`'s old channel."""
    m = dict(m)
    old = m.get(slot)
    for s, c in list(m.items()):
        if c == ch and s != slot:
            if old is None:
                del m[s]
            else:
                m[s] = old
    m[slot] = ch
    return m


def plate_balance(m: dict[str, int], g: dict[int, float], plate: str):
    """Centre of pressure of one plate from its mapped channels.

    Returns (x, y, total_us) with x, y in 0..1 (SLOTS convention), or None when
    the plate has no mapped channel or carries no load.
    """
    pts = [(SLOTS[s][2], SLOTS[s][3], g.get(c, 0.0))
           for s, c in m.items() if SLOTS[s][0] == plate]
    total = sum(w for _, _, w in pts)
    if not pts or total <= 0:
        return None
    return (sum(x * w for x, _, w in pts) / total,
            sum(y * w for _, y, w in pts) / total, total)


class MapSession:
    """Assigns each slot in `slots` to the channel the user presses for it.

    Per slot: hands off for SETTLE_S (baseline = median G per channel), then
    wait for one channel to rise RISE_US above it, DOMINANCE x ahead of the
    runner-up, for HOLD_S; assign it, wait for release, next slot. Channels
    picked earlier in the session are not offered again. Feed it every FSR
    sample; `feed` returns (slot, channel) when one is assigned.
    """

    def __init__(self, slots: list[str]):
        self.slots = list(slots)
        self.i = 0
        self.taken: set[int] = set()
        self._settle(None)

    @property
    def done(self) -> bool:
        return self.i >= len(self.slots)

    @property
    def slot(self) -> str | None:
        return None if self.done else self.slots[self.i]

    @property
    def prompt(self) -> str:
        if self.done:
            return ""
        step = f"[{self.i + 1}/{len(self.slots)}] " if len(self.slots) > 1 else ""
        name = slot_name(self.slot)
        if self.state == "settle":
            return f"{step}hands off the sensors …  (next: {name})"
        if self.state == "press":
            return f"{step}press and hold the {name.upper()} sensor"
        return f"{step}got it — release"

    def candidate(self) -> int | None:
        """Channel currently winning the press test (for highlighting)."""
        return self._lead if self.state == "press" else None

    def _settle(self, t):
        self.state, self.t0, self._buf = "settle", t, []
        self._lead, self._lead_since, self.base = None, None, {}

    def feed(self, t: float, g: dict[int, float]):
        if self.done:
            return None
        if self.state == "settle":
            if self.t0 is None:
                self.t0 = t
            self._buf.append([g.get(c, 0.0) for c in CHANNELS])
            if t - self.t0 >= SETTLE_S:
                med = np.median(np.array(self._buf), axis=0)
                self.base = dict(zip(CHANNELS, med))
                self.state = "press"
            return None

        if self.state == "press":
            rise = sorted(((g.get(c, 0.0) - self.base[c], c)
                           for c in CHANNELS if c not in self.taken), reverse=True)
            (best, ch), runner = rise[0], (rise[1][0] if len(rise) > 1 else 0.0)
            if best >= RISE_US and best >= DOMINANCE * max(runner, 0.0):
                if ch != self._lead:
                    self._lead, self._lead_since = ch, t
                elif t - self._lead_since >= HOLD_S:
                    return self.pick(ch, t)
            else:
                self._lead = None
            return None

        # release: the picked channel back near its baseline for RELEASE_S
        ch = self._picked
        if g.get(ch, 0.0) - self.base[ch] < RELEASE_US:
            if self._rel_since is None:
                self._rel_since = t
            elif t - self._rel_since >= RELEASE_S:
                self.i += 1
                self._settle(t)
        else:
            self._rel_since = None
        return None

    def pick(self, ch: int, t: float):
        """Assign `ch` to the current slot (also used for a manual pick)."""
        slot = self.slot
        self.taken.add(ch)
        self._picked, self._rel_since = ch, None
        if self.base:
            self.state = "release"
        else:                                  # manual pick before a baseline
            self.i += 1
            self._settle(t)
        return slot, ch
