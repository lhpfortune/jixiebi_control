from __future__ import annotations

from dataclasses import dataclass, field

from .types import GesturePrediction, StabilizedGesture


@dataclass
class GestureDebouncer:
    """Require consecutive confident frames before emitting a one-shot command.

    LOCKED is a release interlock: the user must show no recognisable gesture for
    release_frames before the next command can be emitted. This prevents the same
    held hand from firing repeatedly while a robot action is running.
    """

    confidence_threshold: float = 0.72
    confirm_frames: int = 5
    release_frames: int = 8
    candidate: str = ""
    run_length: int = 0
    confidence_ema: float = 0.0
    locked: bool = False
    release_count: int = 0
    _last_result: StabilizedGesture = field(
        init=False,
        default=StabilizedGesture("WAIT", "NONE", 0.0, 0.0, None, False),
    )

    def reset(self) -> None:
        self.candidate = ""
        self.run_length = 0
        self.confidence_ema = 0.0
        self.locked = False
        self.release_count = 0
        self._last_result = StabilizedGesture("WAIT", "NONE", 0.0, 0.0, None, False)

    def update(self, prediction: GesturePrediction) -> StabilizedGesture:
        valid = prediction.label not in {"NONE", "UNKNOWN"} and (
            prediction.confidence >= self.confidence_threshold
        )

        if self.locked:
            if valid:
                self.release_count = 0
            else:
                self.release_count += 1
            if self.release_count >= self.release_frames:
                self.locked = False
                self.candidate = ""
                self.run_length = 0
                self.confidence_ema = 0.0
                self.release_count = 0
                self._last_result = StabilizedGesture(
                    "WAIT", "NONE", prediction.confidence, 0.0, None, False
                )
            else:
                progress = self.release_count / max(self.release_frames, 1)
                self._last_result = StabilizedGesture(
                    "LOCKED",
                    prediction.label,
                    prediction.confidence,
                    progress,
                    None,
                    True,
                )
            return self._last_result

        if not valid:
            self.candidate = ""
            self.run_length = 0
            self.confidence_ema = 0.0
            self.release_count = 0
            self._last_result = StabilizedGesture(
                "WAIT", prediction.label, prediction.confidence, 0.0, None, False
            )
            return self._last_result

        if prediction.label == self.candidate:
            self.run_length += 1
            alpha = 0.35
            self.confidence_ema = alpha * prediction.confidence + (1 - alpha) * self.confidence_ema
        else:
            self.candidate = prediction.label
            self.run_length = 1
            self.confidence_ema = prediction.confidence

        if self.run_length >= self.confirm_frames:
            triggered = GesturePrediction(
                label=self.candidate,
                confidence=self.confidence_ema,
                scores=prediction.scores,
                details=prediction.details,
            )
            self.locked = True
            self.release_count = 0
            self._last_result = StabilizedGesture(
                "TRIGGERED", self.candidate, self.confidence_ema, 1.0, triggered, True
            )
        else:
            self._last_result = StabilizedGesture(
                "TRACKING",
                self.candidate,
                self.confidence_ema,
                self.run_length / max(self.confirm_frames, 1),
                None,
                False,
            )
        return self._last_result
