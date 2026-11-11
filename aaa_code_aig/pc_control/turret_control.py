# -*- coding: utf-8 -*-
"""
2축(pan/tilt) 추적 제어기와 포탑 각도 상태 관리.

오차 정의: 화면에서 "표적 중심 - 카메라(화면) 중심"을 각도로 바꾼 값 (track_target.py).
  pan 오차  > 0 : 표적이 화면 중심보다 오른쪽
  tilt 오차 > 0 : 표적이 화면 중심보다 아래쪽

제어 방식 (속도형 PID):
  서보각 변화율[deg/s] = Kp*e + Ki*∫e + Kd*de/dt  ->  매 프레임 서보각 += 변화율 * dt
  - 서보 명령각을 계속 누적하는 구조라, 누적 자체가 적분 역할을 한다. 그래서 P만으로도
    정지 표적에 오차 0으로 수렴하고, Ki는 기본 0 (켜면 지연과 겹쳐 진동이 커짐).
  - 게인이 "프레임당"이 아니라 "초당"이라 FPS가 바뀌어도 반응 속도가 같다.
    (기존 코드는 매 프레임 0.35*오차씩 움직여서, 카메라/YOLO 지연 동안 오래된 오차로
     계속 밀어붙이다가 지나쳐 진동했다)
  - 오차 저역통과 필터(EMA): YOLO 박스 떨림이 서보로 그대로 가지 않게
  - 불감대(deadband): 중심 근처 작은 오차에는 반응하지 않아 미세 떨림 제거
  - 최대 회전 속도 제한: 펌웨어 슬루(약 66deg/s)보다 약간 낮게
  - 미분항은 필터된 오차로 계산 + 추가 필터 (노이즈 증폭 방지)

안정 조건 참고: 지연이 L초일 때 Kp*L이 0.5 정도면 오버슈트 없이 수렴, 1.5를 넘으면 진동한다.
  CPU 추론 + 웹캠 지연 L ≈ 0.2~0.3초 기준 Kp = 2.0 (Kp*L ≈ 0.5)

pan_sign/tilt_sign은 "화면 오차 -> 실제 서보각 증가 방향" 하드웨어 장착 방향 보정값이다
(여기 한 곳에서만 적용).
"""

import math
import time


class RatePID:
    """화면 각도 오차(deg) -> 서보각 변화량(deg)을 내는 속도형 PID."""

    def __init__(self, kp, ki, kd, deadband_deg=1.0, max_rate_deg_s=60.0,
                 err_alpha=0.5, d_alpha=0.3, integral_limit_deg_s=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.deadband = deadband_deg
        self.max_rate = max_rate_deg_s
        self.err_alpha = err_alpha      # 1이면 필터 없음, 작을수록 부드럽지만 느려짐
        self.d_alpha = d_alpha
        self.integral_limit = integral_limit_deg_s
        self.reset()

    def reset(self):
        self._err_f = None
        self._prev_err_f = None
        self._deriv_f = 0.0
        self._integral = 0.0
        self._prev_time = None

    def update(self, error, now=None, ff_rate=0.0):
        """ff_rate: 피드포워드 회전속도(deg/s). 오차와 무관하게 출력에 그대로 더해진다
        (불감대·필터·게인을 거치지 않음 = 2자유도 구조). 기존 방식은 0."""
        now = now if now is not None else time.monotonic()
        if self._prev_time is None:
            dt = 1.0 / 30.0                       # 첫 프레임: 보통 프레임 간격으로 가정
        else:
            dt = min(max(now - self._prev_time, 1e-3), 0.2)  # 렉으로 dt가 튀어도 한 번에 크게 안 움직이게
        self._prev_time = now

        # 1) 오차 저역통과 필터
        self._err_f = error if self._err_f is None else (
            self.err_alpha * error + (1.0 - self.err_alpha) * self._err_f)
        e = self._err_f

        # 2) 불감대: 중심 근처면 멈춤 (경계에서 튀지 않게 불감대만큼 빼서 연속적으로)
        if abs(e) <= self.deadband:
            e_ctrl = 0.0
        else:
            e_ctrl = e - math.copysign(self.deadband, e)

        # 3) 미분 (필터된 오차 기준 + 한 번 더 필터)
        if self._prev_err_f is None:
            raw_d = 0.0
        else:
            raw_d = (e - self._prev_err_f) / dt
        self._prev_err_f = e
        self._deriv_f = self.d_alpha * raw_d + (1.0 - self.d_alpha) * self._deriv_f

        # 4) 변화율 계산 + 포화
        rate = self.kp * e_ctrl + self.ki * self._integral + self.kd * self._deriv_f + ff_rate
        saturated = abs(rate) >= self.max_rate
        rate = max(-self.max_rate, min(self.max_rate, rate))

        # 5) 적분 (불감대 안이거나 포화 중이면 쌓지 않음 = 안티 와인드업)
        if self.ki > 0 and e_ctrl != 0.0 and not saturated:
            self._integral += e_ctrl * dt
            lim = self.integral_limit / self.ki
            self._integral = max(-lim, min(lim, self._integral))

        return rate * dt


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class TurretControl:
    """pan/tilt 2축 추적 제어기 + 서보각 상태."""

    def __init__(self, config):
        self.cfg = config
        kwargs = dict(
            deadband_deg=config.deadband_deg,
            max_rate_deg_s=config.max_rate_deg_s,
            err_alpha=config.error_filter_alpha,
        )
        self.pid_pan = RatePID(config.kp, config.ki, config.kd, **kwargs)
        self.pid_tilt = RatePID(config.kp, config.ki, config.kd, **kwargs)
        self.pan_angle = config.pan_center_deg
        self.tilt_angle = config.tilt_center_deg
        self.lost_frames = 0

    def update(self, pan_error_deg, tilt_error_deg, now):
        """화면 오차(표적-중심, deg)를 받아 서보각을 갱신."""
        self.pan_angle += self.cfg.pan_sign * self.pid_pan.update(pan_error_deg, now)
        self.tilt_angle += self.cfg.tilt_sign * self.pid_tilt.update(tilt_error_deg, now)
        self.pan_angle = clamp(self.pan_angle, *self.cfg.pan_limits)
        self.tilt_angle = clamp(self.tilt_angle, *self.cfg.tilt_limits)
        self.lost_frames = 0
        return self.pan_angle, self.tilt_angle

    def update_2dof(self, pan_error_deg, tilt_error_deg, pan_ff_deg_s, tilt_ff_deg_s, now):
        """2자유도 제어 (--control 2dof). 오차와 피드포워드는 이미 "서보각 공간"이라 부호 보정 없음.
        회전속도 = 피드포워드(표적 각속도, 게인 1) + PID(예측 표적각 - 현재 서보각)."""
        self.pan_angle += self.pid_pan.update(pan_error_deg, now, ff_rate=pan_ff_deg_s)
        self.tilt_angle += self.pid_tilt.update(tilt_error_deg, now, ff_rate=tilt_ff_deg_s)
        self.pan_angle = clamp(self.pan_angle, *self.cfg.pan_limits)
        self.tilt_angle = clamp(self.tilt_angle, *self.cfg.tilt_limits)
        self.lost_frames = 0
        return self.pan_angle, self.tilt_angle

    def coast(self, pan_rate_deg_s, tilt_rate_deg_s, dt):
        """표적을 놓친 동안 예측 각속도로 계속 회전 (피드백 없음, 속도 상한 적용)."""
        lim = self.cfg.max_rate_deg_s
        self.pan_angle += clamp(pan_rate_deg_s, -lim, lim) * dt
        self.tilt_angle += clamp(tilt_rate_deg_s, -lim, lim) * dt
        self.pan_angle = clamp(self.pan_angle, *self.cfg.pan_limits)
        self.tilt_angle = clamp(self.tilt_angle, *self.cfg.tilt_limits)
        return self.pan_angle, self.tilt_angle

    def handle_lost(self):
        """표적을 놓쳤을 때 호출. 현재 각도를 유지하고, lost_target_hold_frames만큼 이어지면
        제어기 상태(필터/미분/적분)를 리셋한다 (재포착 시 예전 값 때문에 튀는 것 방지).
        리셋이 막 발생했는지 여부를 반환한다."""
        self.lost_frames += 1
        # 한 프레임이라도 놓치면 시간 간격이 끊기므로 다음 프레임 dt가 커지지 않게 시간만 초기화
        self.pid_pan._prev_time = None
        self.pid_tilt._prev_time = None
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