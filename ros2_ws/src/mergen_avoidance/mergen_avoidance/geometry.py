"""Aci ve aralik (interval) yardimcilari.

Burada ROS'a bagli hicbir sey yok -> `pytest` ile ROS kurulu olmadan test edilebilir.
Tum acilar radyan ve robot govde cercevesinde (base_link):
  0      = ileri (+x)
  +pi/2  = sol   (+y)
  -pi/2  = sag   (-y)
"""
from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

TAU = 2.0 * math.pi
Interval = Tuple[float, float]


def normalize_angle(a: float) -> float:
    """Aciyi (-pi, pi] araligina tasir."""
    a = math.fmod(a, TAU)
    if a > math.pi:
        a -= TAU
    elif a <= -math.pi:
        a += TAU
    return a


def angle_diff(target: float, source: float) -> float:
    """target - source, (-pi, pi] araliginda. Isaret: + ise target solda."""
    return normalize_angle(target - source)


def clamp(x: float, lo: float, hi: float) -> float:
    if lo > hi:  # cagiran tarafta parametre hatasi varsa sessizce bozulmayalim
        raise ValueError(f"clamp: lo({lo}) > hi({hi})")
    return lo if x < lo else hi if x > hi else x


def merge_intervals(intervals: Iterable[Interval], gap_tol: float = 0.0) -> List[Interval]:
    """Ust uste binen / `gap_tol` kadar yakin aralikları birlestirir.

    Dikkat: burada acilar duz sayi olarak ele alinir, -pi/+pi sarmasi YAPILMAZ.
    Bunu bilerek boyle biraktik: aday rotalari her zaman +-`max_steer_angle`
    (< pi) icinde arayacagiz, yani sarma bolgesine hic girmiyoruz.
    """
    items = sorted((lo, hi) for lo, hi in intervals if hi >= lo)
    if not items:
        return []
    out: List[Interval] = [items[0]]
    for lo, hi in items[1:]:
        last_lo, last_hi = out[-1]
        if lo <= last_hi + gap_tol:
            out[-1] = (last_lo, max(last_hi, hi))
        else:
            out.append((lo, hi))
    return out


def in_intervals(x: float, intervals: Sequence[Interval]) -> bool:
    for lo, hi in intervals:
        if lo <= x <= hi:
            return True
    return False


def free_intervals(blocked: Sequence[Interval], lo: float, hi: float) -> List[Interval]:
    """[lo, hi] icinde `blocked` ile kapanmayan bosluklari dondurur."""
    out: List[Interval] = []
    cursor = lo
    for b_lo, b_hi in merge_intervals(blocked):
        if b_hi < lo or b_lo > hi:
            continue
        b_lo = max(b_lo, lo)
        if b_lo > cursor:
            out.append((cursor, b_lo))
        cursor = max(cursor, min(b_hi, hi))
        if cursor >= hi:
            break
    if cursor < hi:
        out.append((cursor, hi))
    return [(a, b) for a, b in out if b > a]
