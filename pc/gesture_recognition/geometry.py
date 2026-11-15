from __future__ import annotations

import math
from collections.abc import Sequence

from .types import Landmark

FINGER_INDICES = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}


def clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return max(low, min(high, value))


def distance(a: Landmark, b: Landmark) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def angle_deg(a: Landmark, b: Landmark, c: Landmark) -> float:
    """Return the angle ABC in degrees."""
    ba = (a.x - b.x, a.y - b.y, a.z - b.z)
    bc = (c.x - b.x, c.y - b.y, c.z - b.z)
    norm_ba = math.sqrt(sum(v * v for v in ba))
    norm_bc = math.sqrt(sum(v * v for v in bc))
    if norm_ba < 1e-8 or norm_bc < 1e-8:
        return 0.0
    cosine = sum(x * y for x, y in zip(ba, bc)) / (norm_ba * norm_bc)
    return math.degrees(math.acos(clamp(cosine, -1.0, 1.0)))


def palm_size(landmarks: Sequence[Landmark]) -> float:
    return max(distance(landmarks[0], landmarks[9]), 1e-6)


def pair_distance_ratio(landmarks: Sequence[Landmark], first: int, second: int) -> float:
    return distance(landmarks[first], landmarks[second]) / palm_size(landmarks)


def _linear_score(value: float, low: float, high: float) -> float:
    return clamp((value - low) / max(high - low, 1e-6))


def finger_extension_score(landmarks: Sequence[Landmark], finger: str) -> float:
    """Estimate extension in [0, 1] using joint angles and chain straightness."""
    mcp_i, pip_i, dip_i, tip_i = FINGER_INDICES[finger]
    mcp, pip, dip, tip = (
        landmarks[mcp_i],
        landmarks[pip_i],
        landmarks[dip_i],
        landmarks[tip_i],
    )

    pip_angle = angle_deg(mcp, pip, dip)
    dip_angle = angle_deg(pip, dip, tip)
    bend_score = 0.5 * _linear_score(pip_angle, 105.0, 170.0)
    bend_score += 0.5 * _linear_score(dip_angle, 105.0, 170.0)

    chain_length = distance(mcp, pip) + distance(pip, dip) + distance(dip, tip)
    straightness = distance(mcp, tip) / max(chain_length, 1e-6)
    straight_score = _linear_score(straightness, 0.68, 0.92)

    wrist = landmarks[0]
    reach = distance(tip, wrist) / max(distance(pip, wrist), 1e-6)
    reach_score = _linear_score(reach, 0.90, 1.35)

    if finger == "thumb":
        abduction = pair_distance_ratio(landmarks, 4, 5)
        abduction_score = _linear_score(abduction, 0.35, 0.90)
        return clamp(0.40 * bend_score + 0.35 * straight_score + 0.25 * abduction_score)

    return clamp(0.45 * bend_score + 0.40 * straight_score + 0.15 * reach_score)


def finger_extension_scores(landmarks: Sequence[Landmark]) -> dict[str, float]:
    return {name: finger_extension_score(landmarks, name) for name in FINGER_INDICES}


def normalized_feature_vector(landmarks: Sequence[Landmark]) -> list[float]:
    """Translation/scale-normalized 21x3 landmarks plus five extension scores."""
    if len(landmarks) != 21:
        raise ValueError(f"Expected 21 landmarks, got {len(landmarks)}")
    wrist = landmarks[0]
    scale = palm_size(landmarks)
    features: list[float] = []
    for point in landmarks:
        features.extend(
            (
                (point.x - wrist.x) / scale,
                (point.y - wrist.y) / scale,
                (point.z - wrist.z) / scale,
            )
        )
    scores = finger_extension_scores(landmarks)
    features.extend(scores[name] for name in ("thumb", "index", "middle", "ring", "pinky"))
    return features


def hand_quality(landmarks: Sequence[Landmark]) -> float:
    """Confidence prior based on apparent hand size and finite landmarks."""
    if len(landmarks) != 21:
        return 0.0
    if any(not math.isfinite(v) for p in landmarks for v in (p.x, p.y, p.z)):
        return 0.0
    size = palm_size(landmarks)
    return clamp((size - 0.035) / 0.10, 0.15, 1.0)
