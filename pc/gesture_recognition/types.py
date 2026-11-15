from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field

LABELS = ("ONE", "TWO", "THREE", "OK", "FIST", "OPEN_PALM")
IGNORED_LABELS = {"NONE", "UNKNOWN"}


@dataclass(frozen=True)
class Landmark:
    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True)
class GesturePrediction:
    label: str
    confidence: float
    scores: Mapping[str, float] = field(default_factory=dict)
    details: Mapping[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class HandObservation:
    landmarks: tuple[Landmark, ...]
    handedness: str
    handedness_score: float = 0.0


@dataclass(frozen=True)
class StabilizedGesture:
    state: str
    candidate: str
    confidence: float
    progress: float
    triggered: GesturePrediction | None = None
    locked: bool = False
