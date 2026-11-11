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
    device: str = "auto"   # YOLO 추론 장치: auto / 0(첫 번째 NVIDIA GPU) / cpu
    target_class: str = "target"
    max_jump_frac: float = 0.35

    # --- 화각 ---
    # hfov: 2026-11-04 pan 지연 측정에서 k=1.067 -> 60*1.067 = 64.0
    # vfov: 48로 뒀다가 tilt가 1초 주기로 진동 -> 2dof_v2 로그 역산으로 31(임시)
    #       -> 2026-11-04 tilt 지연 측정(--sweep-axis tilt)에서 k=1.102 -> 31*1.102 = 34.2 확정
    #       (16:9 기하학 값 38.6보다 작은 건 tilt 기구가 명령각보다 덜 도는 영향으로 추정. 서보각 기준 실효값)
    hfov_deg: float = 64.0
    vfov_deg: float = 34.2

    # --- 서보 / 포탑 하드웨어 ---
    pan_center_deg: float = 90.0
    tilt_center_deg: float = 90.0
    pan_limits: tuple = (10.0, 170.0)
    tilt_limits: tuple = (20.0, 160.0)
    lost_target_hold_frames: int = 15

    # 포탑 장착 방향에 따른 부호 보정 ("화면 오차 +" 일 때 서보각을 늘릴지(+1) 줄일지(-1)).
    # 2026-09-28: 좌우 반대 현상으로 -1.0으로 바꿨었으나, 2026-10-29 현재 배선/장착에서
    # 다시 좌우가 반대로 움직여 +1.0으로 되돌림. 표적 반대쪽으로 끝까지 도망가면 부호가 틀린 것.
    pan_sign: float = 1.0
    tilt_sign: float = 1.0

    # 영점(zero) 보정: 카메라 광축과 포신 기준선이 완전히 일치하지 않을 때의 오프셋
    pan_zero_deg: float = 0.0
    tilt_zero_deg: float = 0.0

    # --- 추적 제어 방식 ---
    #   2dof  : 지연 보상 + α-β 속도 추정 + 피드포워드(회전속도에 직접) + PID 보정 (tracking_filter.py)
    #   legacy: 기존 방식 (화면 오차 -> 속도형 PID, 피드포워드는 탄도 모델 사용 시에만)
    control_mode: str = "2dof"

    # --- 추적 PID (속도형: 오차[deg] -> 서보 회전속도[deg/s], turret_control.py 참고) ---
    # 2dof에서는 추종을 피드포워드가 맡고 PID는 남은 오차만 보정하므로 약하게 둔다.
    # (2026-11-04 이전 config 값: kp 4.0, ki 0.2, kd 0.15, deadband 1.0, filter 0.7)
    kp: float = 2.5                  # 오차 1도당 회전속도(deg/s). Kp*L <= 0.75 권장. 진동하면 낮춤
    ki: float = 0.1                  # 피드포워드 추정 오차 몇 deg/s만 메우는 용도. 크게 올리면 진동
    kd: float = 0.0                  # 2dof에서는 진짜 피드포워드가 역할을 대신해 0 (노이즈 증폭만 남음)
    deadband_deg: float = 0.4        # 중심 불감대. 서보가 1도 단위라 0.3 아래는 의미 적음
    max_rate_deg_s: float = 60.0     # 최대 회전 속도 (펌웨어 슬루 66deg/s보다 약 10% 낮게)
    error_filter_alpha: float = 0.9  # 오차 EMA (1=필터 없음). 노이즈는 α-β 필터가 처리하므로 약하게

    # --- 2dof: 지연 보상 / α-β 필터 / 놓침 처리 (tracking_filter.py) ---
    latency_s: float = 0.25          # 전체 지연 L(초, pan 기준). analyze_log.py --latency로 실측
    latency_tilt_s: float = 0.265    # tilt 지연 (None이면 latency_s 사용)
                                     # 2026-11-04 --sweep-axis tilt 실측 (흔들림 5.75 -> 1.40deg)
    # 2026-11-04 실측(10.6fps, 왕복 표적)에서 속도 추정이 0.28s 늦고 77%만 따라가 α/β를 올림
    # (이전 α 0.5 β 0.15). 정지 표적 떨림은 ff_deadband로 억제.
    ab_alpha: float = 0.7            # 측정 위치를 얼마나 믿을지 (크면 빠르지만 떨림 통과)
    ab_beta: float = 0.4             # 속도 갱신 속도 (크면 방향 전환에 빠름, 노이즈에 민감)
    ff_deadband_deg_s: float = 4.0   # 이 값보다 작은 속도 추정은 0으로 (정지 표적 떨림 방지)
    reacquire_gate_deg: float = 10.0 # 예측과 측정 차이가 이보다 크면 이상치 (그 프레임은 무시하고 예측대로)
    reacquire_frames: int = 3        # 이상치가 이만큼 연속이면 표적이 바뀐 것으로 보고 필터 리셋
    coast_hold_s: float = 0.3        # 놓친 뒤 이 시간까지는 예측 속도 그대로 회전
    coast_max_s: float = 0.8         # 이 시간까지 서서히 감속 후 정지, 필터 리셋

    # --- (legacy 전용) 속도 피드포워드: 표적 각속도 * lead_time을 오차에 더함 ---
    feedforward_lead_time_s: float = 0.15
    velocity_history_len: int = 8
    velocity_min_samples: int = 3

    # --- 리드샷(선도각) / 사격 (전동건: DC모터 1개) ---
    ballistic_json: str = None
    target_width_m: float = None
    lead_solve_interval: float = 0.2
    fire_duration_s: float = 1.5   # 신호 2에서 사격할 시간 n초 (펌웨어 상한 5초)
    motor_speed: int = 255         # 전동건 모터 PWM 세기 (0~255)

    # --- 시리얼 ---
    serial_port: str = None
    serial_baud: int = 9600
    no_serial: bool = False

    # --- 로그 / 표시 ---
    log_csv: str = None
    no_show: bool = False