# -*- coding: utf-8 -*-
"""
표적의 픽셀 오차 -> 각도 오차 변환, 방위각(베어링)·사거리(bbox 기반)·3D 위치·속도
추정, 그리고 "목표각 - 현재 서보각"(turret_control.py의 PID에 넣을 최종 오차) 계산.

영점(pan_zero/tilt_zero) 오프셋, fire_control이 넘겨준 선도각(리드샷) 결과,
속도 피드포워드를 모두 반영한다. 여기서 계산하는 오차는 하드웨어 장착 방향과
무관한 "월드 프레임" 오차이며, pan_sign/tilt_sign을 이용한 실제 서보 방향 변환은
turret_control.py에서 한 곳에서만 적용한다.
"""

from collections import deque

import numpy as np


class TargetTracker:
    def __init__(self, config):
        self.cfg = config
        # (t, yaw_bearing_deg, pitch_bearing_deg, x, y, z)
        self.history = deque(maxlen=config.velocity_history_len)

    # ---------- 픽셀 -> 각도 ----------
    def pixel_error_deg(self, target, frame_w, frame_h):
        err_x_px = target["cx"] - frame_w / 2.0
        err_y_px = target["cy"] - frame_h / 2.0
        err_x_deg = (err_x_px / (frame_w / 2.0)) * (self.cfg.hfov_deg / 2.0)
        err_y_deg = (err_y_px / (frame_h / 2.0)) * (self.cfg.vfov_deg / 2.0)
        return err_x_px, err_y_px, err_x_deg, err_y_deg

    # ---------- 방위각(베어링) ----------
    def bearing_deg(self, pan_angle, tilt_angle, err_x_deg, err_y_deg):
        """현재 서보각 + 영점 오프셋 + 잔여 픽셀오차 -> 표적의 절대 베어링."""
        yaw_bearing_deg = (
            (pan_angle - self.cfg.pan_center_deg - self.cfg.pan_zero_deg)
            + self.cfg.pan_sign * err_x_deg
        )
        pitch_bearing_deg = -(
            (tilt_angle - self.cfg.tilt_center_deg - self.cfg.tilt_zero_deg)
            + self.cfg.tilt_sign * err_y_deg
        )
        return yaw_bearing_deg, pitch_bearing_deg

    # ---------- 사거리(단안, bbox 폭 기반) ----------
    def estimate_range_m(self, bbox_w_px, frame_w_px):
        """bbox 폭(px)과 화각으로 사거리(m)를 역산 (핀홀 카메라 모델)."""
        if bbox_w_px <= 1e-6 or self.cfg.target_width_m is None:
            return None
        focal_px = (frame_w_px / 2.0) / np.tan(np.radians(self.cfg.hfov_deg / 2.0))
        return (self.cfg.target_width_m * focal_px) / bbox_w_px

    # ---------- 3D 위치 기록 ----------
    def update_position(self, now, yaw_bearing_deg, pitch_bearing_deg, range_m):
        yaw_rad = np.radians(yaw_bearing_deg)
        pitch_rad = np.radians(pitch_bearing_deg)
        x = range_m * np.cos(yaw_rad) * np.cos(pitch_rad)
        y = range_m * np.sin(pitch_rad)
        z = range_m * np.sin(yaw_rad) * np.cos(pitch_rad)
        self.history.append((now, yaw_bearing_deg, pitch_bearing_deg, x, y, z))
        return x, y, z

    def _regress_slope(self, idx):
        pts = list(self.history)
        t = np.array([p[0] for p in pts])
        t = t - t[0]
        return float(np.polyfit(t, [p[idx] for p in pts], 1)[0])

    def estimate_velocity(self):
        """최근 샘플에 대해 축별(x,y,z) 1차 선형회귀 기울기를 속도로 사용.
        노이즈 큰 단안 거리값을 그대로 두 점 미분하는 것보다 안정적이다."""
        if len(self.history) < self.cfg.velocity_min_samples:
            return 0.0, 0.0, 0.0
        return self._regress_slope(3), self._regress_slope(4), self._regress_slope(5)

    def estimate_bearing_rate_deg_s(self):
        """속도 피드포워드용: 베어링(yaw/pitch)의 변화율(deg/s)을 같은 방식으로 추정."""
        if len(self.history) < self.cfg.velocity_min_samples:
            return 0.0, 0.0
        return self._regress_slope(1), self._regress_slope(2)

    def reset(self):
        self.history.clear()

    # ---------- 목표각 - 현재 서보각 (PID 입력 오차) ----------
    def pid_errors(self, pan_angle, tilt_angle, err_x_deg, err_y_deg,
                    lead_solution=None, feedforward_deg_per_s=(0.0, 0.0)):
        """lead_solution이 있으면 화면 중앙 정렬 대신 선도각(yaw/pitch)을 목표로 삼는다
        (정지 표적이면 자동으로 기존 중앙 정렬과 동일하게 수렴). 속도 피드포워드
        (표적 베어링 각속도 * feedforward_lead_time_s)를 더해 응답 지연을 줄인다."""
        pan_error = err_x_deg
        tilt_error = err_y_deg

        if lead_solution is not None:
            pan_setpoint = self.cfg.pan_center_deg + lead_solution["yaw_deg"]
            tilt_setpoint = self.cfg.tilt_center_deg - lead_solution["pitch_deg"]
            pan_error = self.cfg.pan_sign * (pan_setpoint - pan_angle)
            tilt_error = self.cfg.tilt_sign * (tilt_setpoint - tilt_angle)

        ff_yaw_deg_s, ff_pitch_deg_s = feedforward_deg_per_s
        pan_error += self.cfg.feedforward_lead_time_s * ff_yaw_deg_s
        tilt_error += self.cfg.feedforward_lead_time_s * ff_pitch_deg_s

        return pan_error, tilt_error
