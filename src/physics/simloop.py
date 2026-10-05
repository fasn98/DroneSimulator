"""
Shared simulation loop for every vehicle template.

`SimulationLoop` owns the clock, the fixed-step run loop and the telemetry
recording cadence. A vehicle simulator (multirotor, helicopter, ...) subclasses
it and supplies three things:

    step(command)  advance the plant one physics step (dt) under `command`
    record()       append one telemetry row
    observe()      the state the guidance function sees (true or estimated)

`WindModel` (mean wind + Gauss-Markov gusts) is shared too, so every template
flies in the same atmosphere and random-number discipline: one seeded
generator per run, hence the same seed gives the same trajectory.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List

import numpy as np

from .atmosphere import Body


class WindModel:
    """Mean wind plus first-order Gauss-Markov gusts (a simplified Dryden model)."""

    def __init__(self, body: Body, rng: np.random.Generator, correlation_time: float = 3.0):
        self.mean = body.wind_mean * np.array([np.cos(np.radians(body.wind_direction_deg)),
                                               np.sin(np.radians(body.wind_direction_deg)), 0.0])
        self.sigma = body.wind_gust_std
        self.tau = correlation_time
        self.rng = rng
        self.gust = np.zeros(3)

    def step(self, dt: float) -> np.ndarray:
        if self.sigma > 0:
            noise = self.rng.standard_normal(3) * np.array([1.0, 1.0, 0.3])
            self.gust += -self.gust * dt / self.tau + self.sigma * np.sqrt(2 * dt / self.tau) * noise
        return self.mean + self.gust


@dataclass
class Telemetry:
    rows: List[Dict[str, float]] = field(default_factory=list)

    def column(self, key: str) -> np.ndarray:
        return np.array([r[key] for r in self.rows])


class SimulationLoop:
    """Fixed-step loop: `run` calls step() every dt and record() every `record_every` seconds."""

    dt: float
    t: float
    telemetry: Telemetry

    def step(self, command: Any) -> None:  # pragma: no cover - abstract
        raise NotImplementedError

    def record(self) -> None:  # pragma: no cover - abstract
        raise NotImplementedError

    def observe(self) -> np.ndarray:  # pragma: no cover - abstract
        raise NotImplementedError

    def run(self, duration: float, command_fn: Callable[[float, np.ndarray], Any],
            record_every: float = 0.1) -> Telemetry:
        rec_k = max(int(round(record_every / self.dt)), 1)
        steps = int(round(duration / self.dt))
        for i in range(steps):
            self.step(command_fn(self.t, self.observe()))
            if i % rec_k == 0:
                self.record()
        self.record()
        return self.telemetry
