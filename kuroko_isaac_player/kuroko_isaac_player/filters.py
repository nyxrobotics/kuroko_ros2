from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LowPassFilter:
    """Simple first-order low-pass filter.

    y <- y + alpha * (x - y)
    alpha in (0,1]. alpha=1 => no filtering.
    """

    alpha: float
    y: float = 0.0
    initialized: bool = False

    def reset(self):
        self.y = 0.0
        self.initialized = False

    def update(self, x: float) -> float:
        if not self.initialized:
            self.y = x
            self.initialized = True
            return self.y
        self.y = self.y + self.alpha * (x - self.y)
        return self.y
