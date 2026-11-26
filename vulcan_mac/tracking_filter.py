# -*- coding: utf-8 -*-
"""
추적 전용 예측 제어 (--control 2dof). 목표: 움직이는 표적을 화면 중심에 계속 붙여두기.

모든 각도는 "서보각 공간"에서 다룬다:
  표적 서보각 = (그 프레임이 찍힌 시점의 서보각) + sign * 화면 각도 오차
  = 서보를 이 각도로 보내면 표적이 화면 중심에 오는 각도.
  (거리·3D 위치·영점 오프셋 불필요)

신호 흐름 (축마다 독립):
  1) 시간 정렬   : 프레임 캡처 시점 = 처리 완료 시각 - L, 그 시점의 서보 명령각을 이력에서 보간
  2) α-β 필터    : 측정 표적각 -> [표적각 θ, 각속도 ω] 추정 (YOLO 박스 떨림도 여기서 거름)
  3) 현재 시점 예측: θ̂(지금) = θ(캡처 시점) + ω·L   (스미스 예측기 개념, 지연 보상)
  4) 2자유도 제어 : 서보 회전속도 = ω̂ (피드포워드, 게인 1) + PID(θ̂ - 서보각)
  5) 놓침 처리   : 측정이 없으면 예측만으로 ω̂ 방향으로 계속 회전, 시간이 지나면 서서히 정지

L(cfg.latency_s)은 analyze_log.py --latency로 실측해서 넣는다. 틀리면 과보상/덜보상이 생긴다.
"""

import bisect
import math
from collections import deque


def soft_deadband(x, db):
    """|x| <= db면 0, 그 밖은 db만큼 빼서 연속으로 (정지 표적에서 속도 추정 노이즈가 서보를 흔들지 않게)."""
    if abs(x) <= db:
        return 0.0
    return x - math.copysign(db, x)


class AlphaBetaFilter:
    """1축 등속 모델 α-β 필터. 상태 = [각도 θ, 각속도 ω], 기준 시각 t."""

    def __init__(self, alpha, beta, gate_deg=None):
        self.alpha = alpha
        self.beta = beta
        self.gate_deg = gate_deg
        self.reset()

    def reset(self):
        self.theta = None
        self.omega = 0.0
        self.t = None

    @property
    def ready(self):
        return self.theta is not None

    def predict_to(self, t):
        """측정 없이 t까지 등속으로 상태를 진행 (놓침 구간)."""
        if self.theta is None or t <= self.t:
            return
        self.theta += self.omega * (t - self.t)
        self.t = t

    def value_at(self, t):
        return self.theta + self.omega * (t - self.t)

    def residual(self, meas, t):
        """측정 - 예측 (상태는 바꾸지 않음). 이상치 판정용."""
        return meas - self.value_at(t)

    def update(self, meas, t):
        """측정 반영. (잔차, 리셋 여부)를 반환한다."""
        if self.theta is None:
            self.theta, self.omega, self.t = meas, 0.0, t
            return 0.0, True
        dt = t - self.t
        if dt <= 1e-4:
            dt = 1e-4
        theta_pred = self.theta + self.omega * dt
        r = meas - theta_pred
        if self.gate_deg is not None and abs(r) > self.gate_deg:
            # 예측과 너무 다름 -> 다른 물체를 잡았거나 큰 기동: 측정값으로 다시 시작
            self.theta, self.omega, self.t = meas, 0.0, t
            return r, True
        self.theta = theta_pred + self.alpha * r
        self.omega = self.omega + (self.beta / dt) * r
        self.t = t
        return r, False


class ServoHistory:
    """서보 명령각 이력 (시각, pan, tilt). 과거 시점의 서보각을 선형 보간으로 꺼낸다."""

    def __init__(self, maxlen=600):
        self._t = deque(maxlen=maxlen)
        self._pan = deque(maxlen=maxlen)
        self._tilt = deque(maxlen=maxlen)

    def add(self, t, pan, tilt):
        if self._t and t <= self._t[-1]:
            self._pan[-1], self._tilt[-1] = pan, tilt  # 같은 시각이면 마지막 값 갱신
            return
        self._t.append(t)
        self._pan.append(pan)
        self._tilt.append(tilt)

    def at(self, t):
        if not self._t:
            return None
        ts = list(self._t)
        if t <= ts[0]:
            return self._pan[0], self._tilt[0]
        if t >= ts[-1]:
            return self._pan[-1], self._tilt[-1]
        i = bisect.bisect_right(ts, t)
        t0, t1 = ts[i - 1], ts[i]
        k = (t - t0) / (t1 - t0)
        return (self._pan[i - 1] + k * (self._pan[i] - self._pan[i - 1]),
                self._tilt[i - 1] + k * (self._tilt[i] - self._tilt[i - 1]))


class PredictiveTracker:
    """지연 보상 + α-β 추정 + 2자유도 제어. TurretControl의 각도를 직접 갱신한다."""

    def __init__(self, cfg, turret):
        self.cfg = cfg
        self.turret = turret
        self.history = ServoHistory()
        # 이상치 판정은 여기서 두 축을 같이 본다 (오검출은 보통 두 축 모두 튐)
        self.f_pan = AlphaBetaFilter(cfg.ab_alpha, cfg.ab_beta)
        self.f_tilt = AlphaBetaFilter(cfg.ab_alpha, cfg.ab_beta)
        self.last_meas_time = None
        self.last_time = None
        self.coast_done = False
        self.outliers = 0
        self.record(0.0)

    def record(self, now):
        self.history.add(now, self.turret.pan_angle, self.turret.tilt_angle)

    def reset(self):
        self.f_pan.reset()
        self.f_tilt.reset()
        self.last_meas_time = None
        self.coast_done = False
        self.outliers = 0

    def _cap_times(self, now):
        """축별 캡처 시점. tilt는 포신 하중 때문에 지연이 다를 수 있어 따로 둔다."""
        lp = self.cfg.latency_s
        lt = self.cfg.latency_tilt_s if self.cfg.latency_tilt_s is not None else lp
        return now - lp, now - lt

    def on_measurement(self, now, err_pan_deg, err_tilt_deg):
        cfg = self.cfg
        tp, tt = self._cap_times(now)
        cap_p = self.history.at(tp)
        cap_t = self.history.at(tt)
        servo_cap_pan = cap_p[0] if cap_p else self.turret.pan_angle
        servo_cap_tilt = cap_t[1] if cap_t else self.turret.tilt_angle

        # 1) 시간 정렬된 측정 표적각 (서보각 공간)
        meas_pan = servo_cap_pan + cfg.pan_sign * err_pan_deg
        meas_tilt = servo_cap_tilt + cfg.tilt_sign * err_tilt_deg

        # 이상치 게이트: 예측과 크게 다르면 한두 프레임은 무시하고 예측대로 계속 돈다.
        # reacquire_frames 프레임 연속이면 진짜로 표적이 바뀐 것으로 보고 필터를 다시 시작.
        # (한 프레임 오검출이나 급가속 한 번에 속도 추정을 0으로 날리지 않기 위함)
        filter_reset = False
        if self.f_pan.ready:
            r_pan = self.f_pan.residual(meas_pan, tp)
            r_tilt = self.f_tilt.residual(meas_tilt, tt)
            if max(abs(r_pan), abs(r_tilt)) > cfg.reacquire_gate_deg:
                self.outliers += 1
                if self.outliers < cfg.reacquire_frames:
                    return self._coast_outlier(now, tp, tt, meas_pan, meas_tilt, r_pan, r_tilt)
                self.f_pan.reset()
                self.f_tilt.reset()
                filter_reset = True
            self.outliers = 0

        # 2) α-β 필터 (필터 시간축 = 캡처 시점)
        r_pan, reset_pan = self.f_pan.update(meas_pan, tp)
        r_tilt, reset_tilt = self.f_tilt.update(meas_tilt, tt)
        filter_reset = filter_reset or (reset_pan and reset_tilt and self.last_meas_time is not None)

        # 3) 현재 시점 예측 (작은 속도 추정값은 노이즈로 보고 0 처리)
        db = cfg.ff_deadband_deg_s
        w_pan = soft_deadband(self.f_pan.omega, db)
        w_tilt = soft_deadband(self.f_tilt.omega, db)
        pred_pan = self.f_pan.theta + w_pan * (now - self.f_pan.t)
        pred_tilt = self.f_tilt.theta + w_tilt * (now - self.f_tilt.t)

        # 4) 2자유도 제어: FF(ω̂) + PID(예측 표적각 - 현재 서보각)
        e_pan = pred_pan - self.turret.pan_angle
        e_tilt = pred_tilt - self.turret.tilt_angle
        self.turret.update_2dof(e_pan, e_tilt, w_pan, w_tilt, now)

        self.last_meas_time = now
        self.last_time = now
        self.coast_done = False
        self.record(now)
        return {
            "meas_pan": meas_pan, "meas_tilt": meas_tilt,
            "est_pan": self.f_pan.theta, "est_tilt": self.f_tilt.theta,
            "omega_pan": w_pan, "omega_tilt": w_tilt,
            "pred_pan": pred_pan, "pred_tilt": pred_tilt,
            "resid_pan": r_pan, "resid_tilt": r_tilt,
            "filter_reset": int(filter_reset), "outlier": 0,
            "ff_scale": 1.0,
        }

    def _coast_outlier(self, now, tp, tt, meas_pan, meas_tilt, r_pan, r_tilt):
        """이상치 프레임: 측정을 버리고 예측 속도로 한 프레임 진행."""
        dt = 0.0 if self.last_time is None else min(max(now - self.last_time, 0.0), 0.2)
        self.last_time = now
        self.f_pan.predict_to(tp)
        self.f_tilt.predict_to(tt)
        self.turret.coast(self.f_pan.omega, self.f_tilt.omega, dt)
        self.record(now)
        return {
            "meas_pan": meas_pan, "meas_tilt": meas_tilt,
            "omega_pan": self.f_pan.omega, "omega_tilt": self.f_tilt.omega,
            "pred_pan": self.f_pan.value_at(now), "pred_tilt": self.f_tilt.value_at(now),
            "resid_pan": r_pan, "resid_tilt": r_tilt,
            "filter_reset": 0, "outlier": 1, "ff_scale": 1.0,
        }

    def on_lost(self, now):
        """측정이 없는 프레임: 예측만으로 계속 회전하다가 시간이 지나면 서서히 멈춘다."""
        cfg = self.cfg
        dt = 0.0 if self.last_time is None else min(max(now - self.last_time, 0.0), 0.2)
        self.last_time = now
        info = {"ff_scale": 0.0}

        if self.last_meas_time is None or not self.f_pan.ready or self.coast_done:
            self.record(now)
            return info

        lost_s = now - self.last_meas_time
        if lost_s >= cfg.coast_max_s:
            self.reset()
            self.coast_done = True
            self.record(now)
            return info

        if lost_s <= cfg.coast_hold_s:
            scale = 1.0
        else:
            scale = 1.0 - (lost_s - cfg.coast_hold_s) / (cfg.coast_max_s - cfg.coast_hold_s)

        tp, tt = self._cap_times(now)
        self.f_pan.predict_to(tp)
        self.f_tilt.predict_to(tt)
        w_pan, w_tilt = self.f_pan.omega * scale, self.f_tilt.omega * scale
        self.turret.coast(w_pan, w_tilt, dt)
        self.record(now)
        info.update({
            "omega_pan": self.f_pan.omega, "omega_tilt": self.f_tilt.omega,
            "pred_pan": self.f_pan.value_at(now), "pred_tilt": self.f_tilt.value_at(now),
            "ff_scale": scale,
        })
        return info
