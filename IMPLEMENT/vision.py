# -*- coding: utf-8 -*-
"""
카메라 입력과 YOLO 추론, 그리고 프레임 안에서 표적 하나를 선택하는 로직.
(클래스 필터링 + 직전 위치에서 너무 멀리 튄 후보 배제)
"""

import math

import cv2
from ultralytics import YOLO


class Camera:
    def __init__(self, camera_index, width, height):
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.cap.isOpened():
            raise RuntimeError(f"카메라를 열 수 없습니다: index={camera_index}")

    def read(self):
        return self.cap.read()

    def release(self):
        self.cap.release()


class Detector:
    def __init__(self, model_path, conf_threshold, target_class):
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.target_class = target_class or None  # 빈 문자열이면 클래스 필터링 없음

    def predict(self, frame):
        return self.model.predict(frame, conf=self.conf_threshold, verbose=False)[0]


def pick_target(result, target_class, last_center=None, max_jump_px=None):
    """프레임에서 표적 하나를 선택한다 (단일 표적 실험이므로 기본은 최고 신뢰도 박스).

    last_center/max_jump_px가 주어지면, 직전에 표적이 있던 위치에서 max_jump_px보다
    멀리 떨어진 후보는 (가까운 후보가 하나라도 있으면) 제외한다. 학습 환경과 다른
    곳에서 엉뚱한 물체가 순간적으로 더 높은 신뢰도로 잡혀 그쪽으로 튀는 것을 막기 위함.
    """
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return None

    candidates = []
    for box in boxes:
        cls_id = int(box.cls[0])
        cls_name = result.names[cls_id]
        if target_class is not None and cls_name != target_class:
            continue
        conf = float(box.conf[0])
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        candidates.append({
            "conf": conf,
            "cls_name": cls_name,
            "x1": x1, "y1": y1, "x2": x2, "y2": y2,
            "cx": (x1 + x2) / 2.0,
            "cy": (y1 + y2) / 2.0,
            "w": x2 - x1,
            "h": y2 - y1,
        })

    if not candidates:
        return None

    if last_center is not None and max_jump_px:
        near = [
            c for c in candidates
            if math.hypot(c["cx"] - last_center[0], c["cy"] - last_center[1]) <= max_jump_px
        ]
        if near:
            candidates = near  # 가까운 후보가 있으면 그중에서만 고름, 없으면 전체에서 고름(재포착)

    return max(candidates, key=lambda c: c["conf"])
