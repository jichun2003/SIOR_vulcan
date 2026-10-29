# -*- coding: utf-8 -*-
"""
사격 통제: 2단계 신호 상태머신 + 선도각(리드샷) 계산 주기 조절.

상태 흐름 (신호 = 콘솔 "shot" 입력 또는 영상 창에서 스페이스바):
  TRACK   : 표적을 화면 중앙에 맞춰 추적 (선도각 미적용)
     | 신호 1
     v
  LEAD    : 선도각 장입. 목표각이 "표적 + 선도각"으로 바뀌어 포가 그만큼 이동하고,
            선도각이 들어간 상태로 계속 추적 (선도각은 lead_solve_interval마다 갱신)
     | 신호 2
     v
  FIRING  : 추적을 계속하면서 fire_duration_s초 동안 전동건 사격
     | 시간 종료
     v
  LEAD    : 다시 신호를 주면 또 n초 사격

  취소(콘솔 "cancel" 또는 x 키) -> 사격 중지 + TRACK으로 복귀

--ballistic-json이 없으면 선도각을 계산할 수 없으므로 LEAD에서도 중앙 정렬 추적과 같고,
사격 흐름(신호 2번 -> n초 사격)만 동작한다.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "ballistic"))

TRACK = "TRACK"
LEAD = "LEAD"
FIRING = "FIRING"


class FireControl:
    def __init__(self, config):
        self.cfg = config
        self.model = None
        if config.ballistic_json:
            from lead_solver import BallisticModel  # 탄도 모델이 있을 때만 scipy 등 로드
            self.model = BallisticModel.from_json(config.ballistic_json)
            print(f"탄도 모델 로드: {self.model}")
        else:
            print("탄도 모델 없음: 선도각 0으로 동작 (신호 2번 -> 사격 흐름만 사용)")

        self.state = TRACK
        self._fire_end = 0.0
        self._last_solve_time = 0.0
        self._cached_solution = None

    # ---------- 신호 처리 ----------
    def signal(self, now):
        """신호 1회 처리. 'lead'(선도각 장입), 'fire'(사격 시작), None(무시) 중 하나를 반환."""
        if self.state == TRACK:
            self.state = LEAD
            return "lead"
        if self.state == LEAD:
            self.state = FIRING
            self._fire_end = now + self.cfg.fire_duration_s
            return "fire"
        return None  # 사격 중 신호는 무시

    def cancel(self):
        """선도각 해제 + 사격 중지. 사격 중이었는지 반환."""
        was_firing = self.state == FIRING
        self.state = TRACK
        return was_firing

    def update(self, now):
        """사격 시간이 끝났으면 LEAD로 돌아가고 True 반환."""
        if self.state == FIRING and now >= self._fire_end:
            self.state = LEAD
            return True
        return False

    def lead_active(self):
        return self.state in (LEAD, FIRING)

    def is_firing(self):
        return self.state == FIRING

    def fire_remaining(self, now):
        return max(0.0, self._fire_end - now) if self.state == FIRING else 0.0

    # ---------- 선도각 계산 (주기 조절 + 해 없으면 이전 값 유지) ----------
    def solve(self, now, turret_pos, target_pos, target_velocity):
        if self.model is None:
            return None
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

    def reset(self):
        self._cached_solution = None
        self._last_solve_time = 0.0
        self.cancel()
