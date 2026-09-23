# -*- coding: utf-8 -*-
"""출력 클램핑과 적분 와인드업 방지가 포함된 간단한 PID 컨트롤러."""

import time


class PID:
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
