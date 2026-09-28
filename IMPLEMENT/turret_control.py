# -*- coding: utf-8 -*-
"""
2축(pan/tilt) PID 컨트롤러와 포탑 각도 상태 관리.
서보 한계 클램핑, 표적을 놓쳤을 때의 처리(PID 리셋, 각도 유지), 수동 리셋을 담당.

pan_sign/tilt_sign은 "월드(실세계) 프레임 오차 -> 실제 서보 각도 변화 방향"을
변환하는 하드웨어 장착 방향 보정값이다 (여기 한 곳에서만 적용). track_target.py는
이 부호와 무관한 월드 프레임 오차만 넘기면 된다.
"""

import time


class PID:
    """출력 클램핑과 적분 와인드업 방지가 포함된 간단한 PID 컨트롤러."""

    def __init__(self, kp, ki, kd, output_limits=(-10.0, 10.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min, self.out_max = output_limits
        self._integral = 0.0
        self._prev_error = None
        self._prev_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = None
        self._prev_time = None

    def update(self, error, now=None):
        now = now if now is not None else time.monotonic()
        dt = 0.0 if self._prev_time is None else max(now - self._prev_time, 1e-3)

        derivative = 0.0 if self._prev_error is None else (error - self._prev_error) / dt

        candidate_integral = self._integral + error * dt
        output = self.kp * error + self.ki * candidate_integral + self.kd * derivative

        # 출력이 한계 안에 있을 때만 적분값을 갱신 (안티 와인드업)
        if self.out_min <= output <= self.out_max:
            self._integral = candidate_integral
        output = max(self.out_min, min(self.out_max, output))

        self._prev_error = error
        self._prev_time = now
        return output


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class TurretControl:
    """pan/tilt 2축 PID + 서보각 상태."""

    def __init__(self, config):
        self.cfg = config
        self.pid_pan = PID(config.kp, config.ki, config.kd,
                            output_limits=(-config.max_step_deg, config.max_step_deg))
        self.pid_tilt = PID(config.kp, config.ki, config.kd,
                             output_limits=(-config.max_step_deg, config.max_step_deg))
        self.pan_angle = config.pan_center_deg
        self.tilt_angle = config.tilt_center_deg
        self.lost_frames = 0

    def update(self, pan_error_deg, tilt_error_deg, now):
        """pan/tilt PID 오차(목표각-현재각, track_target.py가 계산)를 받아 서보각을 갱신."""
        self.pan_angle += self.cfg.pan_sign * self.pid_pan.update(pan_error_deg, now)
        self.tilt_angle += self.cfg.tilt_sign * self.pid_tilt.update(tilt_error_deg, now)
        self.pan_angle = clamp(self.pan_angle, *self.cfg.pan_limits)
        self.tilt_angle = clamp(self.tilt_angle, *self.cfg.tilt_limits)
        self.lost_frames = 0
        return self.pan_angle, self.tilt_angle

    def handle_lost(self):
        """표적을 놓쳤을 때 호출. lost_target_hold_frames만큼 이어지면 PID를 리셋하고
        현재 각도를 유지한다 (재포착 시 급격히 튀는 것 방지). 리셋이 막 발생했는지 여부를
        반환한다."""
        self.lost_frames += 1
        just_reset = False
        if self.lost_frames == self.cfg.lost_target_hold_frames:
            self.pid_pan.reset()
            self.pid_tilt.reset()
            just_reset = True
        return just_reset

    def reset(self):
        self.pan_angle = self.cfg.pan_center_deg
        self.tilt_angle = self.cfg.tilt_center_deg
        self.pid_pan.reset()
        self.pid_tilt.reset()
        self.lost_frames = 0
