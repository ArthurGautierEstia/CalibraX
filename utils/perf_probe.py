"""Instrumentation de performance, active uniquement si CALIBRAX_PERF est défini.

Usage :
    $env:CALIBRAX_PERF=1
    .venv/Scripts/python.exe main.py

Aucun overhead quand la variable d'env est absente.
"""
from __future__ import annotations

import os
import time
from contextlib import contextmanager
from typing import Generator

_ENABLED = bool(os.environ.get("CALIBRAX_PERF"))


class _ProbeAccumulator:
    def __init__(self) -> None:
        self._totals: dict[str, float] = {}
        self._counts: dict[str, int] = {}
        self._maxes: dict[str, float] = {}

    def record(self, name: str, elapsed_s: float) -> None:
        self._totals[name] = self._totals.get(name, 0.0) + elapsed_s
        self._counts[name] = self._counts.get(name, 0) + 1
        self._maxes[name] = max(self._maxes.get(name, 0.0), elapsed_s)

    def dump_and_reset(self) -> None:
        if not self._counts:
            return
        print("\n=== CALIBRAX_PERF ===")
        names = sorted(self._counts.keys())
        for name in names:
            n = self._counts[name]
            total_ms = self._totals[name] * 1000.0
            avg_ms = total_ms / n
            max_ms = self._maxes[name] * 1000.0
            print(f"  {name:<45s}  n={n:5d}  total={total_ms:8.1f}ms  avg={avg_ms:6.2f}ms  max={max_ms:6.2f}ms")
        print("=====================\n")
        self._totals.clear()
        self._counts.clear()
        self._maxes.clear()


_accumulator = _ProbeAccumulator()


@contextmanager
def probe(name: str) -> Generator[None, None, None]:
    if not _ENABLED:
        yield
        return
    t0 = time.perf_counter()
    try:
        yield
    finally:
        _accumulator.record(name, time.perf_counter() - t0)


def dump_and_reset() -> None:
    if _ENABLED:
        _accumulator.dump_and_reset()


class FpsCounter:
    """Compteur FPS pour paintGL. Affiche une ligne par seconde."""

    def __init__(self, label: str = "paintGL") -> None:
        self._label = label
        self._count = 0
        self._window_start = time.perf_counter()

    def tick(self) -> None:
        if not _ENABLED:
            return
        self._count += 1
        now = time.perf_counter()
        elapsed = now - self._window_start
        if elapsed >= 1.0:
            fps = self._count / elapsed
            print(f"[CALIBRAX_PERF] {self._label}: {fps:.1f} FPS")
            self._count = 0
            self._window_start = now
