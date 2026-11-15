from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from .geometry import clamp, finger_extension_scores, hand_quality, pair_distance_ratio
from .types import GesturePrediction, Landmark


def _smoothstep(value: float, low: float, high: float) -> float:
    x = clamp((value - low) / max(high - low, 1e-6))
    return x * x * (3.0 - 2.0 * x)


class HeuristicGestureClassifier:
    """Classify five stable teaching gestures from MediaPipe landmarks.

    Supported: ONE, TWO, THREE, OK, FIST, OPEN_PALM.
    The probability is a calibrated support score, not a neural-network softmax.
    """

    def __init__(self, minimum_support: float = 0.58) -> None:
        self.minimum_support = minimum_support

    def predict(self, landmarks: Sequence[Landmark]) -> GesturePrediction:
        if len(landmarks) != 21:
            raise ValueError(f"Expected 21 landmarks, got {len(landmarks)}")

        ext = finger_extension_scores(landmarks)
        p = {name: _smoothstep(score, 0.38, 0.68) for name, score in ext.items()}
        pinch_ratio = pair_distance_ratio(landmarks, 4, 8)
        pinch_probability = _smoothstep(0.55 - pinch_ratio, 0.03, 0.25)

        supports = {
            "ONE": np.mean([p["index"], 1 - p["middle"], 1 - p["ring"], 1 - p["pinky"]]),
            "TWO": np.mean([p["index"], p["middle"], 1 - p["ring"], 1 - p["pinky"]]),
            "THREE": np.mean([p["index"], p["middle"], p["ring"], 1 - p["pinky"]]),
            "OK": np.mean(
                [
                    1 - p["index"],
                    p["middle"],
                    p["ring"],
                    p["pinky"],
                    pinch_probability,
                ]
            ),
            "FIST": np.mean([1 - p["index"], 1 - p["middle"], 1 - p["ring"], 1 - p["pinky"]]),
            "OPEN_PALM": np.mean([p["thumb"], p["index"], p["middle"], p["ring"], p["pinky"]]),
        }
        labels = tuple(supports)
        raw = np.asarray([supports[label] for label in labels], dtype=np.float64)
        shifted = raw - raw.max()
        probabilities = np.exp(shifted * 7.0)
        probabilities /= probabilities.sum()

        best_index = int(np.argmax(raw))
        best_label = labels[best_index]
        best_support = float(raw[best_index])
        quality = hand_quality(landmarks)
        confidence = float(probabilities[best_index] * quality)
        scores = {label: float(probabilities[i]) for i, label in enumerate(labels)}

        if best_support < self.minimum_support or confidence < 0.35:
            return GesturePrediction(
                label="UNKNOWN",
                confidence=confidence,
                scores=scores,
                details={
                    **ext,
                    "pinch_ratio": pinch_ratio,
                    "best_support": best_support,
                    "quality": quality,
                },
            )

        return GesturePrediction(
            label=best_label,
            confidence=confidence,
            scores=scores,
            details={
                **ext,
                "pinch_ratio": pinch_ratio,
                "best_support": best_support,
                "quality": quality,
            },
        )
