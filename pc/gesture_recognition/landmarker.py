from __future__ import annotations

import shutil
import tempfile
from pathlib import Path

import mediapipe as mp
import numpy as np

from .types import HandObservation, Landmark


def _ascii_safe_model_path(model_path: Path) -> Path:
    """Work around MediaPipe's Windows ANSI path limitation."""
    if str(model_path).isascii():
        return model_path

    target_dir = Path(tempfile.gettempdir()) / "gesture_recognition_models"
    target_dir.mkdir(parents=True, exist_ok=True)
    target = target_dir / model_path.name
    if not target.is_file() or target.stat().st_size != model_path.stat().st_size:
        shutil.copy2(model_path, target)
    if not str(target).isascii():
        raise RuntimeError(
            "MediaPipe cannot read this model path because it contains non-ASCII characters. "
            "Move TEMP to an ASCII-only directory or pass an ASCII model path with --model."
        )
    return target


class HandLandmarkDetector:
    """Thin wrapper around MediaPipe Tasks HandLandmarker."""

    def __init__(
        self,
        model_path: str | Path,
        max_hands: int = 1,
        detection_confidence: float = 0.55,
        presence_confidence: float = 0.55,
        tracking_confidence: float = 0.55,
    ) -> None:
        model_path = Path(model_path)
        if not model_path.is_file():
            raise FileNotFoundError(
                f"MediaPipe model not found: {model_path}. Run scripts/download_model.ps1."
            )

        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision

        self._vision = vision
        safe_model_path = _ascii_safe_model_path(model_path)
        base_options = mp_python.BaseOptions(model_asset_path=str(safe_model_path))
        options = vision.HandLandmarkerOptions(
            base_options=base_options,
            running_mode=vision.RunningMode.VIDEO,
            num_hands=max_hands,
            min_hand_detection_confidence=detection_confidence,
            min_hand_presence_confidence=presence_confidence,
            min_tracking_confidence=tracking_confidence,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)
        self._last_timestamp_ms = -1

    def detect(
        self, frame_rgb: np.ndarray, timestamp_ms: int | None = None
    ) -> tuple[HandObservation | None, object]:
        if timestamp_ms is None:
            timestamp_ms = self._last_timestamp_ms + 1
        if timestamp_ms <= self._last_timestamp_ms:
            timestamp_ms = self._last_timestamp_ms + 1
        self._last_timestamp_ms = timestamp_ms

        rgb = np.ascontiguousarray(frame_rgb, dtype=np.uint8)
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        result = self._landmarker.detect_for_video(image, timestamp_ms)
        if not result.hand_landmarks:
            return None, result

        hand = result.hand_landmarks[0]
        landmarks = tuple(Landmark(point.x, point.y, point.z) for point in hand)
        hand_name = "Unknown"
        hand_score = 0.0
        if result.handedness and result.handedness[0]:
            category = result.handedness[0][0]
            hand_name = category.category_name or "Unknown"
            hand_score = float(category.score or 0.0)
        return HandObservation(landmarks, hand_name, hand_score), result

    def close(self) -> None:
        self._landmarker.close()

    def __enter__(self) -> HandLandmarkDetector:  # noqa: PYI034
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
