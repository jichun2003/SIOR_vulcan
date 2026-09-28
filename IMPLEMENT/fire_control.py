# -*- coding: utf-8 -*-
"""
ballistic/lead_solver.py(BallisticModel) 호출 주기 조절, 선도각->보정값 변환,
조준 완료 판단, "ready" 명령 이후 예측 사격(3초 뒤 위치를 예측해 조준하다가
조준이 완료되면 사격)을 담당한다.

CLAUDE.md 실험계획 2단계: "표적을 따라 포의 헤드 부분이 추적하다가 사용자가
'ready' 프롬프트를 보내면 3초 뒤의 위치를 예측하여 준비 후 사격한다."
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
        self._ready_deadline = None
        self._fired_for_current_ready = False
        print(f"리드샷 모드 활성화: {self.model}")

    # ---------- ready 명령 ----------
    def trigger_ready(self, now):
        self._ready_deadline = now + self.cfg.ready_prepare_seconds
        self._fired_for_current_ready = False
        print(f"[fire_control] ready 수신: {self.cfg.ready_prepare_seconds:.1f}초 뒤 예측 사격 준비")

    def cancel_ready(self):
        self._ready_deadline = None
        self._fired_for_current_ready = False

    def is_ready_pending(self):
        return self._ready_deadline is not None

    def remaining_ready_s(self, now):
        if self._ready_deadline is None:
            return None
        return max(self._ready_deadline - now, 0.0)

    # ---------- 선도각 계산 (주기 조절 + 해 없으면 이전 값 유지) ----------
    def solve(self, now, turret_pos, target_pos, target_velocity):
        if now - self._last_solve_time < self.cfg.lead_solve_interval:
            return self._cached_solution
        self._last_solve_time = now

        aim_target_pos = target_pos
        if self._ready_deadline is not None:
            # ready 이후에는 화면에 보이는 현재 위치가 아니라, 등속 가정으로 외삽한
            # "사격 시점(ready_deadline)의 예측 위치"를 조준 목표로 삼는다.
            lookahead = max(self._ready_deadline - now, 0.0)
            aim_target_pos = tuple(
                p + v * lookahead for p, v in zip(target_pos, target_velocity)
            )

        try:
            solution = self.model.solve_lead_angle(
                turret_pos=turret_pos,
                target_pos=aim_target_pos,
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

    # ---------- ready 마감 도달 + 조준 완료 시 사격 트리거 (1회성) ----------
    def should_fire(self, now, aim_locked):
        if self._ready_deadline is None or self._fired_for_current_ready:
            return False
        if now < self._ready_deadline or not aim_locked:
            return False
        self._fired_for_current_ready = True
        return True

    def reset(self):
        self._cached_solution = None
        self._last_solve_time = 0.0
        self.cancel_ready()
