"""
policy_model.py

this file holds:
- the Action enum
- a simple frame-rate gate
- two policies that turn a camera frame into an action:
    LEFT, RIGHT, FORWARD, BACKWARD, or STOP

available policies:
- LocalHeuristicPolicy: OpenCV-only logic, no model needed
- YOLOHeuristicPolicy: YOLOv8 for obstacle awareness, with heuristic fallback

factory:
- make_policy(kind, fps) -> returns a policy instance

supported kinds:
- "heuristic" / "local"
- "yolo" / "hybrid"
"""

import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

import cv2
import numpy as np


class Action(str, Enum):
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    FORWARD = "FORWARD"
    BACKWARD = "BACKWARD"
    STOP = "STOP"


@dataclass
class FrameGate:
    """
    simple timing helper.

    call .ready() before running anything expensive.
    it returns True only if enough time has passed since the last run.
    """
    min_interval_s: float = 0.2
    _last_time: float = 0.0

    def ready(self) -> bool:
        now = time.time()
        if now - self._last_time >= self.min_interval_s:
            self._last_time = now
            return True
        return False


class LocalHeuristicPolicy:
    """
    simple OpenCV-based policy.

    it looks at the lower half of the frame, finds edges, and makes a rough
    decision about whether the car should go forward, turn, or stop.
    not fancy, but fast and always available.
    """

    def __init__(self, gate: Optional[FrameGate] = None):
        self.gate = gate
        self.last_action: Action = Action.STOP

    def _preprocess(self, frame_bgr: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        h, w = frame_bgr.shape[:2]
        roi = frame_bgr[int(h * 0.5):h, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 1.2)
        edges = cv2.Canny(gray, 60, 150, apertureSize=3, L2gradient=True)
        return roi, edges

    def predict(self, frame_bgr: Optional[np.ndarray]) -> Optional[Action]:
        if self.gate and (not self.gate.ready()):
            return None

        if frame_bgr is None or frame_bgr.size == 0:
            self.last_action = Action.STOP
            return self.last_action

        roi, edges = self._preprocess(frame_bgr)
        eh, ew = edges.shape[:2]

        # check the very bottom first
        # if that part is full of edges, something is probably right in front
        bottom_strip = edges[int(eh * 0.85):eh, :]
        edge_density_bottom = np.mean(bottom_strip > 0)

        if edge_density_bottom > 0.20:
            self.last_action = Action.STOP
            return self.last_action

        # split the view into left / centre / right
        # then compare how cluttered each bit looks
        left_region = edges[:, :ew // 3]
        centre_region = edges[:, ew // 3: 2 * ew // 3]
        right_region = edges[:, 2 * ew // 3:]

        left_score = np.mean(left_region > 0)
        centre_score = np.mean(centre_region > 0)
        right_score = np.mean(right_region > 0)

        # if the whole scene looks pretty empty, just keep moving
        if max(left_score, centre_score, right_score) < 0.02:
            action = Action.FORWARD
        else:
            # more edges usually means more stuff on that side
            # so try to steer toward the cleaner side
            if left_score > right_score * 1.1:
                action = Action.RIGHT
            elif right_score > left_score * 1.1:
                action = Action.LEFT
            else:
                # if the middle looks much more blocked, stop
                # otherwise just carry on
                if centre_score > max(left_score, right_score) * 1.2:
                    action = Action.STOP
                else:
                    action = Action.FORWARD

        self.last_action = action
        return action


class YOLOHeuristicPolicy:
    """
    hybrid policy that uses YOLO first, then falls back to the local heuristic.

    idea is:
    - run YOLO on the frame
    - focus on detections near the bottom of the image
    - estimate whether left, centre, or right is more blocked
    - if YOLO gives a clear answer, use it
    - otherwise let the OpenCV heuristic decide
    """

    def __init__(
        self,
        gate: Optional[FrameGate] = None,
        model_path: str = "yolov8n.pt",
        conf_threshold: float = 0.25,
    ):
        self.gate = gate
        self.last_action: Action = Action.STOP

        # fallback policy does not need its own gate
        self.heuristic = LocalHeuristicPolicy(gate=None)

        try:
            from ultralytics import YOLO  # type: ignore
        except Exception as e:
            raise ImportError(
                "ultralytics is not installed. Run 'pip install ultralytics' first."
            ) from e

        self._YOLO = YOLO
        self.model = self._YOLO(model_path)
        self.conf_threshold = conf_threshold

    def _analyse_boxes(self, frame_bgr: np.ndarray, result) -> Optional[Action]:
        h, w = frame_bgr.shape[:2]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return None

        try:
            xyxy = boxes.xyxy.cpu().numpy()
            conf = boxes.conf.cpu().numpy()
        except Exception:
            return None

        # ignore weak detections
        mask = conf >= self.conf_threshold
        xyxy = xyxy[mask]
        if xyxy.size == 0:
            return None

        left_w = 0.0
        centre_w = 0.0
        right_w = 0.0

        for box in xyxy:
            x1, y1, x2, y2 = box
            box_w = max(0.0, x2 - x1)
            box_h = max(0.0, y2 - y1)
            if box_h <= 0 or box_w <= 0:
                continue

            # only care about objects that are low in the frame
            # those are more likely to be close to the car
            y_bottom = y2
            if y_bottom < h * 0.6:
                continue

            # use box area as a rough guess for how much space is blocked
            area_norm = (box_w * box_h) / (w * h + 1e-6)
            cx = 0.5 * (x1 + x2)

            if cx < w / 3:
                left_w += area_norm
            elif cx > 2 * w / 3:
                right_w += area_norm
            else:
                centre_w += area_norm

        total = left_w + centre_w + right_w
        if total <= 0.0:
            return None

        # if the centre looks clearly blocked, stop
        if centre_w > 0.12 and centre_w >= max(left_w, right_w) * 1.2:
            return Action.STOP

        # if one side looks heavier, turn away from it
        if left_w > right_w * 1.2:
            return Action.RIGHT
        if right_w > left_w * 1.2:
            return Action.LEFT

        # YOLO saw something, just not enough to make a solid call
        return None

    def predict(self, frame_bgr: Optional[np.ndarray]) -> Optional[Action]:
        if self.gate and (not self.gate.ready()):
            return None

        if frame_bgr is None or frame_bgr.size == 0:
            self.last_action = Action.STOP
            return self.last_action

        # YOLO wants RGB, not BGR
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        results = self.model(frame_rgb, verbose=False)
        result = results[0]

        yolo_action = self._analyse_boxes(frame_bgr, result)

        if yolo_action is not None:
            self.last_action = yolo_action
            return yolo_action

        heuristic_action = self.heuristic.predict(frame_bgr)
        if heuristic_action is not None:
            self.last_action = heuristic_action
            return heuristic_action

        return self.last_action


def make_policy(kind: str = "heuristic", fps: float = 5.0):
    """
    build and return a policy.

    Parameters
    ----------
    kind : str
        "heuristic" / "local"  -> LocalHeuristicPolicy
        "yolo" / "hybrid"      -> YOLOHeuristicPolicy

    fps : float
        target decision rate. the FrameGate limits how often the policy
        actually does the heavier work.
    """
    if fps <= 0:
        min_interval = 1.0
    else:
        min_interval = max(1e-6, 1.0 / fps)

    gate = FrameGate(min_interval_s=min_interval)
    kind_norm = (kind or "").lower()

    if kind_norm in ("heuristic", "local"):
        return LocalHeuristicPolicy(gate=gate)
    elif kind_norm in ("yolo", "hybrid", "yolo_heuristic", "yolo+heuristic"):
        return YOLOHeuristicPolicy(gate=gate)
    else:
        raise ValueError(f"Unknown policy kind: {kind}")