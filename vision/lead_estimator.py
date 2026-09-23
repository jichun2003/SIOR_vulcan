# -*- coding: utf-8 -*-
"""
표적의 카메라 기준 베어링(각도)과 bbox 폭으로부터 표적의 3D 위치를 추정하고,
ballistic/lead_solver.py의 BallisticModel로 선도각(리드샷)을 계산하는 헬퍼.

아직 거리 센서가 없는 단계라, 표적의 실제 폭(target_width_m)을 안다고 가정하고
카메라 화각(FOV)과 bbox 폭으로 거리(사거리)를 역산하는 단안(monocular) 추정을
쓴다. bbox 폭은 프레임마다 흔들리므로 정밀 사격용 거리값은 아니고, 리드샷
로직(BallisticModel)을 지금 단계에서 미리 연결/검증해두는 용도다. 나중에 실제
거리 센서가 붙으면 estimate_range_m() 부분만 그 값으로 교체하면 된다.
"""

from collections import deque

import numpy as np


class LeadEstimator:
    def __init__(self, ballistic_model, target_width_m, history_len=8, min_samples=3, solve_interval=0.2):
        self.model = ballistic_model
        self.target_width_m = target_width_m
        self.history = deque(maxlen=history_len)
        self.min_samples = min_samples
        self.solve_interval = solve_interval
        self._last_solve_time = 0.0
        self._cached_solution = None

    def estimate_range_m(self, bbox_w_px, frame_w_px, hfov_deg):
        """bbox 폭(px)과 화각으로 사거리(m)를 역산 (핀홀 카메라 모델)."""
        if bbox_w_px <= 1e-6:
            return None
        focal_px = (frame_w_px / 2.0) / np.tan(np.radians(hfov_deg / 2.0))
        return (self.target_width_m * focal_px) / bbox_w_px

    def update(self, now, yaw_bearing_deg, pitch_bearing_deg, range_m):
        """표적 위치를 기록하고, solve_interval마다 리드샷 계산을 갱신한다.

        좌표계는 ballistic/lead_solver.py와 동일하게 맞춘다:
        x=전방, y=높이(위+), z=측면(오른쪽+). yaw_bearing_deg는 오른쪽이 양수,
        pitch_bearing_deg는 위쪽이 양수인 베어링 각도.

        반환: BallisticModel.solve_lead_angle()의 결과 dict, 계산할 데이터가
        부족하거나 아직 재계산 주기가 안 됐으면 이전 캐시(또는 None).
        """
        yaw_rad = np.radians(yaw_bearing_deg)
        pitch_rad = np.radians(pitch_bearing_deg)

        x = range_m * np.cos(yaw_rad) * np.cos(pitch_rad)
        y = range_m * np.sin(pitch_rad)
        z = range_m * np.sin(yaw_rad) * np.cos(pitch_rad)

        self.history.append((now, x, y, z))

        if now - self._last_solve_time < self.solve_interval:
            return self._cached_solution
        if len(self.history) < self.min_samples:
            return None

        vx, vy, vz = self._estimate_velocity()

        try:
            solution = self.model.solve_lead_angle(
                turret_pos=(0.0, 0.0, 0.0),
                target_pos=(x, y, z),
                target_velocity=(vx, vy, vz),
            )
        except ValueError:
            # 사거리를 벗어나는 등 해를 못 찾은 경우, 이전 값을 유지 (급격한 튐 방지)
            self._last_solve_time = now
            return self._cached_solution

        solution["range_m"] = range_m
        self._cached_solution = solution
        self._last_solve_time = now
        return solution

    def _estimate_velocity(self):
        """최근 샘플에 대해 축별 1차 선형회귀 기울기를 속도로 사용.
        노이즈 큰 단안 거리값을 그대로 두 점 미분하는 것보다 안정적이다."""
        pts = list(self.history)
        t = np.array([p[0] for p in pts])
        t = t - t[0]
        vx = np.polyfit(t, [p[1] for p in pts], 1)[0]
        vy = np.polyfit(t, [p[2] for p in pts], 1)[0]
        vz = np.polyfit(t, [p[3] for p in pts], 1)[0]
        return float(vx), float(vy), float(vz)

    def reset(self):
        self.history.clear()
        self._cached_solution = None
        self._last_solve_time = 0.0
