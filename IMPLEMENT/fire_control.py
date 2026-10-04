# -*- coding: utf-8 -*-
"""
ballistic/lead_solver.py(BallisticModel) 호출 주기 조절, 선도각->보정값 변환,
조준 완료 판단, "shot" 명령 이후 즉시 사격(미래 위치를 미리 예측하지 않고,
계속 추적하며 선도각 계산+조준이 완료되는 즉시 사격)을 담당한다.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "ballistic"))
from lead_solver import BallisticModel  # noqa: E402 (경로 설정 후 지연 import)


class FireControl:
    def __init__(self, config):
        self.cfg = config
        self.model = BallisticModel.from_json(config.ballistic_json)
        self._last_solve_time = 0.0
        self._cached_solution = None
        self._shot_armed = False
        self._fired_for_current_shot = False
        print(f"리드샷 모드 활성화: {self.model}")

    # ---------- shot 명령 ----------
    def trigger_shot(self):
        self._shot_armed = True
        self._fired_for_current_shot = False
        print("[fire_control] shot 수신: 계속 추적하며 조준 완료되는 즉시 사격")

    def cancel_shot(self):
        self._shot_armed = False
        self._fired_for_current_shot = False

    def is_shot_armed(self):
        return self._shot_armed

    # ---------- 선도각 계산 (주기 조절 + 해 없으면 이전 값 유지) ----------
    def solve(self, now, turret_pos, target_pos, target_velocity):
        if now - self._last_solve_time < self.cfg.lead_solve_interval:
            return self._cached_solution
        self._last_solve_time = now

        try:
            solution = self.model.solve_lead_angle(
                turret_pos=turret_pos,
                target_pos=target_pos,
                target_velocity=target_velocity,
            )
        except ValueError:
            # 사거리를 벗어나는 등 해를 못 찾은 경우, 이전 값을 유지 (급격한 튐 방지)
            return self._cached_solution

        self._cached_solution = solution
        return solution

    # ---------- 조준 완료 판단 ----------
    def aim_complete(self, solution, pan_angle, tilt_angle, pan_center_deg, tilt_center_deg):
        if solution is None:
            return False
        pan_setpoint = pan_center_deg + solution["yaw_deg"]
        tilt_setpoint = tilt_center_deg - solution["pitch_deg"]
        return (
            abs(pan_setpoint - pan_angle) <= self.cfg.aim_tolerance_deg
            and abs(tilt_setpoint - tilt_angle) <= self.cfg.aim_tolerance_deg
        )

    # ---------- shot 대기 중 조준 완료되는 즉시 사격 트리거 (1회성) ----------
    def should_fire(self, aim_locked):
        if not self._shot_armed or self._fired_for_current_shot:
            return False
        if not aim_locked:
            return False
        self._fired_for_current_shot = True
        return True

    def reset(self):
        self._cached_solution = None
        self._last_solve_time = 0.0
        self.cancel_shot()
