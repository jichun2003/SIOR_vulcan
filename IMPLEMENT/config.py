# -*- coding: utf-8 -*-
"""
공통으로 쓰이는 가변 파라미터 모음. main.py가 인자를 파싱해 Config를 만들고
그대로 각 모듈(vision/track_target/turret_control/fire_control/serial_link)에
넘긴다 — 파라미터를 여기저기 함수 인자로 흩뿌리지 않기 위한 용도.
"""

from dataclasses import dataclass


@dataclass
class Config:
    # --- 카메라 / 탐지 ---
    model_path: str = "../checkpoint/best.pt"
    camera_index: int = 0
    frame_width: int = 1280
    frame_height: int = 720
    conf_threshold: float = 0.4
    target_class: str = "target"
    max_jump_frac: float = 0.35

    # --- 화각 ---
    hfov_deg: float = 60.0
    vfov_deg: float = 45.0

    # --- 서보 / 포탑 하드웨어 ---
    pan_center_deg: float = 90.0
    tilt_center_deg: float = 90.0
    pan_limits: tuple = (10.0, 170.0)
    tilt_limits: tuple = (20.0, 160.0)
    max_step_deg: float = 8.0
    lost_target_hold_frames: int = 15

    # 포탑 장착 방향에 따른 부호 보정.
    # 2026-09-28 현장 테스트: 표적이 화면에서 오른쪽으로 움직이는데 포탑이 왼쪽으로
    # 도는(반대 방향) 현상 확인 -> pan_sign 기본값을 -1.0으로 교정해서 고침.
    pan_sign: float = -1.0
    tilt_sign: float = 1.0

    # 영점(zero) 보정: 카메라 광축과 포신 기준선이 완전히 일치하지 않을 때의 오프셋
    pan_zero_deg: float = 0.0
    tilt_zero_deg: float = 0.0

    # --- PID ---
    kp: float = 0.35
    ki: float = 0.02
    kd: float = 0.08

    # --- 속도 피드포워드 (표적 각속도 * lead_time을 목표각에 더해 응답 지연 보상) ---
    feedforward_lead_time_s: float = 0.15
    velocity_history_len: int = 8
    velocity_min_samples: int = 3

    # --- 리드샷(선도각) / 사격 ---
    ballistic_json: str = None
    target_width_m: float = None
    lead_solve_interval: float = 0.2
    aim_tolerance_deg: float = 1.5
    flywheel_speed: int = 200
    feeder_push_deg: float = 150.0
    feeder_rest_deg: float = 30.0

    # --- 시리얼 ---
    serial_port: str = None
    serial_baud: int = 9600
    no_serial: bool = False

    # --- 로그 / 표시 ---
    log_csv: str = None
    no_show: bool = False
