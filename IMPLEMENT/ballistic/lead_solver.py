"""
lead_solver.py
------------------------------------------------------------
ballistic_model_fit.py 가 구해준 v0, Cd(공기저항 계수)를 불러와서,
실제로 "목표까지 몇 도로 쏴야 하는지 / 비행시간이 얼마인지 / 움직이는
표적에 대한 선도각(Yaw, Pitch)이 얼마인지"를 계산해주는 재사용 모듈.

전체 시스템(카메라 추적 -> 이 모듈 -> 아두이노 서보 명령)에서
매 프레임 아래처럼 호출해서 쓰면 됩니다:

    from lead_solver import BallisticModel

    model = BallisticModel.from_json("ballistic_model.json")

    solution = model.solve_lead_angle(
        turret_pos=(0, 0, 0),          # 터렛 위치 (x=전방, y=높이, z=측면)
        target_pos=(target_x, target_y, target_z),   # 목표물 현재 위치
        target_velocity=(vx, vy, vz),                 # 목표물 현재 속도
    )
    yaw_deg   = solution["yaw_deg"]
    pitch_deg = solution["pitch_deg"]
    t_flight  = solution["time_of_flight"]
    # yaw_deg, pitch_deg를 시리얼로 아두이노에 전송

vision/track_target.py 에서는 --ballistic-json 옵션으로 이 모듈을 불러와
표적의 카메라 기준 거리/각도로부터 (x, y, z), (vx, vy, vz)를 추정하고
solve_lead_angle()을 호출해 서보 목표각을 계산한다.
------------------------------------------------------------
"""

import json

import numpy as np
from scipy.integrate import odeint
from scipy.optimize import brentq


class BallisticModel:
    """v0, Cd(및 질량/지름)로 정의되는 탄도모델.

    이 클래스 하나가 "탄도방정식" 역할을 하며,
    각도->위치, 거리/높이차->필요각도&비행시간, 선도각 계산까지
    전부 이 안에서 제공합니다.

    좌표계: x = 전방(터렛 정면), y = 높이(위로 양수), z = 측면(오른쪽 양수).
    yaw_deg는 x축 기준 z방향으로의 회전각(오른쪽이 양수), pitch_deg는
    수평면 기준 위쪽이 양수인 발사각이다.
    """

    def __init__(self, v0, Cd, mass_kg, diameter_m, g=9.81, rho=1.225):
        self.v0 = v0
        self.Cd = Cd
        self.mass_kg = mass_kg
        self.diameter_m = diameter_m
        self.g = g
        self.rho = rho
        self.area_m2 = np.pi * (diameter_m / 2) ** 2

    # ---------- 저장된 캘리브레이션 결과 불러오기 ----------
    @classmethod
    def from_json(cls, path, g=9.81, rho=1.225):
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return cls(
            v0=data["v0"],
            Cd=data["Cd"],
            mass_kg=data["mass_kg"],
            diameter_m=data["diameter_m"],
            g=g,
            rho=rho,
        )

    # ---------- 핵심: 운동방정식 (공기저항 + 중력) ----------
    def _dynamics(self, state, t):
        x, y, vx, vy = state
        v = np.hypot(vx, vy)
        k = 0.5 * self.rho * self.Cd * self.area_m2
        if v > 1e-9:
            ax = -k * v * vx / self.mass_kg
            ay = -self.g - k * v * vy / self.mass_kg
        else:
            ax, ay = 0.0, -self.g
        return [vx, vy, ax, ay]

    def simulate(self, theta_deg, t_max, n_steps=1200):
        """주어진 각도(theta_deg)로 t=0에 발사했을 때,
        0~t_max 구간의 (t, x, y) 궤적을 계산해서 반환.
        x: 수평 도달거리(m), y: 높이 변화(m, 발사점 기준)
        """
        theta_rad = np.radians(theta_deg)
        t_sim = np.linspace(0, t_max, n_steps)
        state0 = [0.0, 0.0, self.v0 * np.cos(theta_rad), self.v0 * np.sin(theta_rad)]
        sol = odeint(self._dynamics, state0, t_sim)
        return t_sim, sol[:, 0], sol[:, 1]

    # ---------- 특정 수평거리에 도달했을 때의 (시간, 높이) ----------
    def _time_and_height_at_range(self, theta_deg, target_range_m, t_max=6.0, n_steps=1200):
        """theta_deg로 쐈을 때, 수평거리 target_range_m에 도달하는 순간의
        (비행시간, 그때의 높이)를 반환. 그 거리까지 못 가면 None."""
        t_sim, x_sim, y_sim = self.simulate(theta_deg, t_max, n_steps)

        if x_sim.max() < target_range_m:
            return None  # 이 각도/시간 범위 안에서는 그 거리까지 못 감

        idx = int(np.searchsorted(x_sim, target_range_m))
        if idx <= 0 or idx >= len(x_sim):
            return None

        x0, x1 = x_sim[idx - 1], x_sim[idx]
        t0, t1 = t_sim[idx - 1], t_sim[idx]
        y0, y1 = y_sim[idx - 1], y_sim[idx]
        frac = (target_range_m - x0) / (x1 - x0)
        t = t0 + frac * (t1 - t0)
        y = y0 + frac * (y1 - y0)
        return t, y

    # ---------- 역산: 목표(수평거리, 높이차) -> 필요 각도 & 비행시간 ----------
    def solve_angle_for_target(
        self,
        horizontal_dist_m,
        height_diff_m,
        angle_search_range=(0.1, 80.0),
        n_scan=160,
        t_max=6.0,
    ):
        """목표까지의 수평거리와 높이차(목표높이 - 터렛높이)가 주어졌을 때,
        그 지점에 정확히 명중하는 발사각(pitch)과 비행시간을 구한다.

        같은 거리에 대해 각도가 두 개(저각/고각) 존재할 수 있는데,
        여기서는 낮은 각도(직사에 가까운) 해를 우선적으로 찾는다.
        """
        thetas = np.linspace(angle_search_range[0], angle_search_range[1], n_scan)
        errors, valid_thetas = [], []

        for theta in thetas:
            result = self._time_and_height_at_range(theta, horizontal_dist_m, t_max)
            if result is None:
                continue
            _, y = result
            errors.append(y - height_diff_m)
            valid_thetas.append(theta)

        if len(valid_thetas) < 2:
            raise ValueError(
                f"목표(거리={horizontal_dist_m:.2f}m, 높이차={height_diff_m:.2f}m)에 "
                "도달 가능한 각도를 찾지 못했습니다 (사거리 초과 가능성)."
            )

        errors = np.array(errors)
        valid_thetas = np.array(valid_thetas)
        sign_changes = np.where(np.diff(np.sign(errors)) != 0)[0]

        if len(sign_changes) == 0:
            raise ValueError(
                "해당 거리/높이차에 정확히 명중하는 각도를 찾지 못했습니다. "
                "angle_search_range를 넓혀보세요."
            )

        i = sign_changes[0]  # 가장 낮은(첫 번째) 근 = 저각 해
        theta_lo, theta_hi = valid_thetas[i], valid_thetas[i + 1]

        def err_fn(theta):
            result = self._time_and_height_at_range(theta, horizontal_dist_m, t_max)
            if result is None:
                return 1e6
            _, y = result
            return y - height_diff_m

        theta_root = brentq(err_fn, theta_lo, theta_hi, xtol=1e-3)
        t_root, y_root = self._time_and_height_at_range(theta_root, horizontal_dist_m, t_max)

        return theta_root, t_root

    # ---------- 움직이는 표적에 대한 선도각(리드샷) 계산 ----------
    def solve_lead_angle(
        self,
        turret_pos,
        target_pos,
        target_velocity,
        max_iter=10,
        tol=1e-4,
    ):
        """터렛과 표적의 3D 위치/속도로부터 Yaw, Pitch 선도각을 계산한다.

        좌표계: x, z = 수평면, y = 높이(위로 양수)
        turret_pos, target_pos, target_velocity: (x, y, z) 튜플/배열

        반환: {"yaw_deg", "pitch_deg", "time_of_flight", "impact_point"}
        """
        turret_pos = np.array(turret_pos, dtype=float)
        target_pos = np.array(target_pos, dtype=float)
        target_velocity = np.array(target_velocity, dtype=float)

        t_est = 0.0
        future_pos = target_pos.copy()

        for _ in range(max_iter):
            future_pos = target_pos + target_velocity * t_est
            rel = future_pos - turret_pos

            horizontal_dist = np.hypot(rel[0], rel[2])
            height_diff = rel[1]
            yaw_deg = np.degrees(np.arctan2(rel[2], rel[0]))

            pitch_deg, t_flight = self.solve_angle_for_target(horizontal_dist, height_diff)

            if abs(t_flight - t_est) < tol:
                t_est = t_flight
                break
            t_est = t_flight

        return {
            "yaw_deg": yaw_deg,
            "pitch_deg": pitch_deg,
            "time_of_flight": t_est,
            "impact_point": future_pos.tolist(),
        }

    def __repr__(self):
        return (
            f"BallisticModel(v0={self.v0:.3f} m/s, Cd={self.Cd:.3f}, "
            f"mass={self.mass_kg*1000:.3f} g, diameter={self.diameter_m*1000:.2f} mm)"
        )


# ------------------------------------------------------------
# 사용 예시 (직접 실행했을 때만 동작, import 시에는 실행 안 됨)
# ------------------------------------------------------------
if __name__ == "__main__":
    # ballistic_model_fit.py 실행 후 생성된 json을 불러온다고 가정
    model = BallisticModel.from_json("ballistic_model.json")
    print(model)

    # 예시 1: 정지 표적 - 거리 3m, 높이차 0.2m 지점을 맞추려면?
    theta, t_flight = model.solve_angle_for_target(horizontal_dist_m=3.0, height_diff_m=0.2)
    print(f"\n[정지표적] 필요 발사각: {theta:.2f}도, 비행시간: {t_flight:.3f}초")

    # 예시 2: 움직이는 표적 - 선도각(리드샷) 계산
    solution = model.solve_lead_angle(
        turret_pos=(0, 0, 0),
        target_pos=(3.0, 0.5, 1.0),      # 목표물 현재 위치 (x=전방, y=높이, z=측면)
        target_velocity=(0.0, 0.0, 1.5),  # 목표물이 z방향(측면)으로 1.5 m/s 이동 중
    )
    print(f"\n[이동표적] Yaw={solution['yaw_deg']:.2f}도, "
          f"Pitch={solution['pitch_deg']:.2f}도, "
          f"비행시간={solution['time_of_flight']:.3f}초")
    print(f"예상 명중 지점: {solution['impact_point']}")
