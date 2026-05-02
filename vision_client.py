"""
Camera capture + vision inference (HTTP to Vision FastAPI service).
"""

from __future__ import annotations

import base64
import datetime
import json
import logging
from typing import Any, List, Optional

import requests

import config

logger = logging.getLogger(__name__)

_picamera2_mod: Any = None


def _get_picamera2():
    global _picamera2_mod
    if _picamera2_mod is False:
        return None
    if _picamera2_mod is not None:
        return _picamera2_mod
    try:
        from picamera2 import Picamera2  # type: ignore

        _picamera2_mod = Picamera2
        return Picamera2
    except Exception:
        _picamera2_mod = False
        return None


class RobotVision:
    def __init__(
        self,
        detect_url: str = config.VISION_SERVICE_URL,
        prefer_picamera: bool = True,
    ) -> None:
        self.detect_url = detect_url
        self.prefer_picamera = prefer_picamera
        self._picam = None
        self._cv2 = None

    def start(self) -> None:
        Picamera2 = _get_picamera2() if self.prefer_picamera else None
        if Picamera2 is not None:
            self._picam = Picamera2()
            logger.info("camera: Picamera2")
            return
        try:
            import cv2  # type: ignore

            self._cv2 = cv2
            logger.info("camera: OpenCV fallback")
        except Exception as e:
            logger.warning("no camera backend: %s", e)

    def stop(self) -> None:
        if self._picam is not None:
            try:
                self._picam.stop()
            except Exception:
                pass
            self._picam = None
        self._cv2 = None

    def capture_file(self, path: Optional[str] = None) -> str:
        if path is None:
            path = f"capture-{datetime.datetime.now().isoformat()}.jpg"
        if self._picam is not None:
            self._picam.configure(self._picam.create_still_configuration())
            self._picam.start()
            self._picam.capture_file(path)
            self._picam.stop()
            return path
        if self._cv2 is not None:
            cap = self._cv2.VideoCapture(0)
            try:
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError("OpenCV capture failed")
                self._cv2.imwrite(path, frame)
            finally:
                cap.release()
            return path
        raise RuntimeError("camera not initialized; call start()")

    def detect_objects(self, objects: Optional[List[str]]) -> bool:
        path = self.capture_file()
        return self.detect_image(path, objects)

    def detect_image(self, image_path: str, objects: Optional[List[str]]) -> bool:
        with open(image_path, "rb") as f:
            im_b64 = base64.b64encode(f.read()).decode("utf-8")
        payload: dict = {"image": im_b64, "objects": objects}
        headers = {"content-type": "application/json"}
        try:
            r = requests.post(self.detect_url, headers=headers, json=payload, timeout=30)
            r.raise_for_status()
            data = r.json()
            return bool(data.get("response", False))
        except Exception as e:
            logger.error("vision HTTP error: %s", e)
            return False

    def scan_for_objects(
        self,
        objects: List[str],
        move,
        interval_deg: float = 15.0,
        span_deg: float = 180.0,
    ) -> None:
        """
        Pan the robot, capture at each step, turn toward first positive detection.
        `move` is an ArduinoMovement instance.
        """
        half = span_deg / 2.0
        left_total = 0.0
        right_total = 0.0
        pictures: List[str] = []
        angles: List[float] = []

        while left_total < half:
            move.left(interval_deg)
            left_total += interval_deg
            pictures.append(self.capture_file())
            angles.append(-left_total)

        move.right(left_total)

        while right_total < half:
            move.right(interval_deg)
            right_total += interval_deg
            pictures.append(self.capture_file())
            angles.append(right_total)

        move.left(right_total)

        target_angle = 0.0
        for path, ang in zip(pictures, angles):
            if self.detect_image(path, objects):
                target_angle = ang
                break

        if target_angle < 0:
            move.left(abs(target_angle))
        elif target_angle > 0:
            move.right(target_angle)
