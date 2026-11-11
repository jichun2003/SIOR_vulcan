# -*- coding: utf-8 -*-
"""
실험계획 2단계: YOLO 기반 단일 표적 자율 추적 (2축 pan/tilt PID 제어) + 리드샷(선도각)
진입점. 인자 파싱, 각 모듈(vision/track_target/turret_control/fire_control/serial_link)
생성과 연결, 메인 루프, 키 입력, 화면 오버레이·FPS 표시, CSV 로그를 담당한다.

사격 (전동건, DC모터 1개) - 신호는 콘솔에 "shot" 입력 또는 영상 창에서 스페이스바:
    신호 1: 선도각 장입 -> 포가 선도각만큼 이동, 선도각이 들어간 상태로 계속 추적
    신호 2: 추적을 계속하면서 --fire-duration초 동안 사격 (끝나면 선도각 해제 -> 중앙 추적, 다음 사격도 신호 2번)
    취소  : 콘솔 "cancel" 또는 x 키 -> 사격 중지 + 선도각 해제(중앙 추적으로 복귀)
키: q=종료, r=리셋(중앙 복귀+사격 중지), s=현재 원본 프레임 저장, x=취소, 스페이스=신호

사용 예 (Mac, aaa_code_1111 폴더에서. 아두이노 포트는 ls /dev/cu.usb* 로 확인, 하나뿐이면 --port 생략 가능):
    python3 main.py --camera 0 --no-serial
    python3 main.py --camera 0 --port /dev/cu.usbmodem1101
    python3 main.py --camera 0 --control legacy --ballistic-json ballistic/ballistic_model.json --range-m 1.5 --log-csv logs/lead_test.csv

사용 예 (Windows, aaa_code_1111 폴더에서. 포트 번호는 장치 관리자 > 포트(COM & LPT)에서 확인):
    python main.py --model ..\\checkpoint\\best.pt --camera 0 --port COM3

    # 아두이노 없이 비전/PID 로직만 확인
    python main.py --model ..\\checkpoint\\best.pt --camera 0 --no-serial

    # 거리/크기별 실험 로그 저장 (근거리/원거리 비교 등)
    python main.py --model ..\\checkpoint\\best.pt --no-serial --log-csv logs\\near_small.csv

    # 탄도 모델을 연결해 선도각 계산 + 2단계 사격 (2초 연사, 선도각은 --control legacy에서만 적용)
    python main.py --model ..\\checkpoint\\best.pt --port COM3 --control legacy --ballistic-json ballistic\\ballistic_model.json --target-width-m 0.03 --fire-duration 2

    # 지연 L 실측: 정지 표적을 화면에 두고 pan을 사인파로 흔들며 기록 -> analyze_log.py로 L 계산
    python main.py --camera 0 --port COM3 --latency-sweep --log-csv logs\\latency.csv
    python analyze_log.py logs\\latency.csv --latency

    # 추적 성능 비교 (같은 움직이는 표적으로)
    python main.py --camera 0 --port COM3 --latency 0.18 --log-csv logs\\2dof.csv
    python main.py --camera 0 --port COM3 --control legacy --log-csv logs\\legacy.csv
    python analyze_log.py logs\\2dof.csv logs\\legacy.csv
    python main.py --camera 0 --port COM3 --control legacy --ballistic-json ballistic\ballistic_model.json --range-m 1.5 --log-csv logs\lead_test.csv
"""

import argparse
import csv
import math
import sys
import threading
import time
from pathlib import Path

import cv2

from config import Config
from fire_control import FireControl
from serial_link import find_arduino_port, DummyLink, TurretSerialLink
from track_target import TargetTracker
from tracking_filter import PredictiveTracker
from turret_control import TurretControl
from vision import Camera, Detector, pick_target

# s 키로 저장하는 실시간 원본 프레임 (오버레이 없음) - 실패 장면을 모아 재학습에 쓰기 위함
CAPTURE_DIR = Path(__file__).resolve().parent.parent / "dataset" / "live_capture"

# 프레임별 로그 컬럼. 추적 성능 지표 = err_pan_deg / err_tilt_deg (화면 중심 ~ 표적 중심 각도 오차)
#   servo_*: 그 프레임에 보낸 서보 명령각, meas_*: 시간 정렬된 측정 표적각(서보각 공간),
#   est_*/omega_*: α-β 필터 추정 각도/각속도, pred_*: 현재 시점 예측 표적각 (2dof만)
LOG_FIELDS = [
    "t_mono", "timestamp", "mode", "found", "conf", "bbox_w", "bbox_h",
    "err_pan_deg", "err_tilt_deg", "servo_pan", "servo_tilt",
    "meas_pan", "meas_tilt", "est_pan", "est_tilt", "omega_pan", "omega_tilt",
    "pred_pan", "pred_tilt", "resid_pan", "resid_tilt", "filter_reset", "outlier", "ff_scale",
    "range_m", "lead_yaw_deg", "lead_pitch_deg", "fire_state", "lead_applied",
    "hfov", "vfov",  # 기록 당시 화각 (analyze_log.py --latency가 보정값 계산에 사용)
]


def _fmt(v):
    if isinstance(v, float):
        return f"{v:.4f}"
    return v


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--model", default="best1105_live.pt")
    p.add_argument("--camera", type=int, default=1)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--conf", type=float, default=0.4)
    p.add_argument(
        "--device", default="auto",
        help="YOLO 추론 장치. auto=NVIDIA GPU(0) -> Apple GPU(mps) -> CPU 순 / 0=첫 번째 NVIDIA GPU / mps / cpu",
    )
    p.add_argument(
        "--target-class", default="target",
        help="탐지 클래스 이름으로 필터링. 빈 문자열(\"\")을 넘기면 클래스 무시하고 최고 신뢰도 박스를 사용",
    )
    p.add_argument("--hfov", type=float, default=None,
                   help="카메라 수평 화각(도). 지정 안 하면 config.py 값 (analyze_log.py --latency로 보정)")
    p.add_argument("--vfov", type=float, default=None, help="카메라 수직 화각(도). 지정 안 하면 config.py 값")
    p.add_argument(
        "--pan-sign", type=float, default=1.0, choices=[-1.0, 1.0],
        help="포탑 장착 방향에 따른 pan 부호 보정. 표적과 반대쪽으로 돌면 -1.0으로 "
             "(2026-10-29: 좌우 반대 현상으로 기본값 +1.0으로 변경)",
    )
    p.add_argument("--tilt-sign", type=float, default=1.0, choices=[-1.0, 1.0], help="tilt 부호 보정")
    p.add_argument("--pan-zero-deg", type=float, default=0.0, help="카메라 광축-포신 정렬 오차 보정(pan)")
    p.add_argument("--tilt-zero-deg", type=float, default=0.0, help="카메라 광축-포신 정렬 오차 보정(tilt)")
    p.add_argument("--port", default=None, help="아두이노 시리얼 포트 (Windows: COM3 등 장치 관리자에서 확인 / Mac: /dev/cu.usbmodem..., 생략하면 자동 탐색)")
    p.add_argument("--baud", type=int, default=9600)
    p.add_argument("--no-serial", action="store_true", help="시리얼 연결 없이 비전/PID 로직만 테스트")
    p.add_argument("--log-csv", default=None, help="프레임별 추적 로그 CSV 경로")
    # 추적 튜닝 값: 지정하지 않으면 config.py 값을 그대로 사용 (예전엔 여기 기본값이 config.py를 덮어썼음)
    p.add_argument("--control", choices=["2dof", "legacy"], default=None,
                   help="추적 제어 방식. 2dof=지연 보상+속도 추정+피드포워드(기본), legacy=기존 방식")
    p.add_argument("--kp", type=float, default=None, help="오차 1도당 회전속도(deg/s). 진동하면 낮춤")
    p.add_argument("--ki", type=float, default=None, help="적분 게인")
    p.add_argument("--kd", type=float, default=None, help="미분 게인")
    p.add_argument("--deadband", type=float, default=None, help="중심 불감대(deg). 중심 근처 떨리면 키움")
    p.add_argument("--max-rate", type=float, default=None, help="최대 회전 속도(deg/s)")
    p.add_argument("--err-filter", type=float, default=None,
                   help="오차 필터 계수(0~1, 1=필터 없음). 박스 떨림이 심하면 낮춤")
    p.add_argument("--latency", type=float, default=None,
                   help="2dof: 전체 지연 L(초). analyze_log.py --latency로 실측한 값")
    p.add_argument("--latency-tilt", type=float, default=None, help="2dof: tilt 지연(초). 없으면 config.py 값")
    p.add_argument("--ab-alpha", type=float, default=None, help="2dof: α-β 필터 α (위치 신뢰도)")
    p.add_argument("--ab-beta", type=float, default=None, help="2dof: α-β 필터 β (속도 갱신 속도)")
    p.add_argument("--latency-sweep", action="store_true",
                   help="지연 측정 모드: 추적 없이 pan을 사인파로 흔든다. 정지 표적 + --log-csv와 함께 사용")
    p.add_argument("--sweep-axis", choices=["pan", "tilt"], default="pan", help="지연 측정 모드에서 흔들 축")
    p.add_argument("--sweep-amp", type=float, default=15.0, help="지연 측정 모드 진폭(deg)")
    p.add_argument("--sweep-period", type=float, default=3.0, help="지연 측정 모드 주기(초)")
    p.add_argument("--no-show", action="store_true", help="영상 미리보기 창을 띄우지 않음")

    p.add_argument(
        "--ballistic-json", default=None,
        help="ballistic/ballistic_model_fit.py로 만든 v0/Cd json 경로. 없으면 선도각 0으로 동작",
    )
    p.add_argument("--target-width-m", type=float, default=None,
                   help="표적 실제 폭(m). 박스 크기로 거리 추정. --ballistic-json 사용 시 이것 또는 --range-m 필요")
    p.add_argument("--range-m", type=float, default=None,
                   help="표적 거리를 이 값(m)으로 고정 (거리를 알고 하는 테스트용. 지정하면 --target-width-m보다 우선)")
    p.add_argument("--lead-solve-interval", type=float, default=0.2, help="선도각 재계산 주기(초)")
    p.add_argument("--fire-duration", type=float, default=1.0, help="신호 2에서 사격할 시간 n초 (최대 5)")
    p.add_argument("--motor-speed", type=int, default=255, help="전동건 모터 PWM 세기(0~255)")

    p.add_argument(
        "--max-jump-frac", type=float, default=0.35,
        help="직전 표적 위치에서 프레임 폭 대비 이 비율보다 멀리 떨어진 박스는 무시. 0이면 끔",
    )
    return p.parse_args()


def build_config(args):
    # 명령줄에서 지정한 튜닝 값만 덮어쓰고, 나머지는 config.py 기본값
    tuning = {
        "control_mode": args.control,
        "kp": args.kp, "ki": args.ki, "kd": args.kd,
        "deadband_deg": args.deadband,
        "max_rate_deg_s": args.max_rate,
        "error_filter_alpha": args.err_filter,
        "latency_s": args.latency,
        "latency_tilt_s": args.latency_tilt,
        "hfov_deg": args.hfov,
        "vfov_deg": args.vfov,
        "ab_alpha": args.ab_alpha,
        "ab_beta": args.ab_beta,
    }
    tuning = {k: v for k, v in tuning.items() if v is not None}
    return Config(
        **tuning,
        model_path=args.model,
        camera_index=args.camera,
        frame_width=args.width,
        frame_height=args.height,
        conf_threshold=args.conf,
        device=args.device,
        target_class=args.target_class,
        max_jump_frac=args.max_jump_frac,
        pan_sign=args.pan_sign,
        tilt_sign=args.tilt_sign,
        pan_zero_deg=args.pan_zero_deg,
        tilt_zero_deg=args.tilt_zero_deg,
        ballistic_json=args.ballistic_json,
        target_width_m=args.target_width_m,
        lead_solve_interval=args.lead_solve_interval,
        fire_duration_s=args.fire_duration,
        motor_speed=args.motor_speed,
        serial_port=args.port,
        serial_baud=args.baud,
        no_serial=args.no_serial,
        log_csv=args.log_csv,
        no_show=args.no_show,
    )


class StdinCommandListener:
    """메인 루프를 막지 않고 터미널 입력("shot" = 신호, "cancel" = 취소)을 받는 백그라운드 스레드."""

    def __init__(self):
        self._lock = threading.Lock()
        self._pending = []
        self._thread = threading.Thread(target=self._listen, daemon=True)
        self._thread.start()

    def _listen(self):
        for line in sys.stdin:
            cmd = line.strip().lower()
            if cmd in ("shot", "cancel"):
                with self._lock:
                    self._pending.append(cmd)

    def consume(self):
        with self._lock:
            cmds, self._pending = self._pending, []
        return cmds


def draw_overlay(frame, target, frame_cx, frame_cy, pan_angle, tilt_angle, fps,
                  lead_solution=None, fire_state="TRACK", fire_remaining=0.0):
    h, _ = frame.shape[:2]
    cv2.drawMarker(
        frame, (int(frame_cx), int(frame_cy)), (0, 255, 255),
        markerType=cv2.MARKER_CROSS, markerSize=20, thickness=1,
    )

    if target is not None:
        cv2.rectangle(
            frame, (int(target["x1"]), int(target["y1"])),
            (int(target["x2"]), int(target["y2"])), (0, 255, 0), 2,
        )
        cv2.circle(frame, (int(target["cx"]), int(target["cy"])), 5, (0, 0, 255), -1)
        cv2.putText(
            frame, f"{target['cls_name']} {target['conf']:.2f}",
            (int(target["x1"]), max(int(target["y1"]) - 8, 0)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
        )

    status = f"pan={pan_angle:.1f} tilt={tilt_angle:.1f}  FPS={fps:.1f}"
    cv2.putText(frame, status, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    if lead_solution is not None:
        lead_status = (
            f"range={lead_solution.get('range_m', float('nan')):.2f}m "
            f"lead_yaw={lead_solution['yaw_deg']:.1f} lead_pitch={lead_solution['pitch_deg']:.1f} "
            f"tof={lead_solution['time_of_flight']:.2f}s"
        )
        cv2.putText(frame, lead_status, (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

    if fire_state == "FIRING":
        text, color = f"FIRING {fire_remaining:.1f}s", (0, 0, 255)
    elif fire_state == "LEAD":
        text, color = "LEAD ON (signal -> fire)", (0, 165, 255)
    else:
        text, color = "TRACK (signal -> lead)", (0, 255, 0)
    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)


def main():
    args = parse_args()
    cfg = build_config(args)

    if cfg.ballistic_json and cfg.target_width_m is None and args.range_m is None:
        raise ValueError("--ballistic-json을 쓰려면 --target-width-m(표적 실제 폭, m) 또는 --range-m(고정 거리)도 지정해야 합니다")
    if args.range_m is not None:
        print(f"[range] 표적 거리 {args.range_m:.2f} m로 고정")
    fire_control = FireControl(cfg)
    print('신호("shot" 입력 또는 스페이스바): 1번째 = 선도각 장입, 2번째 = '
          f'{cfg.fire_duration_s:.1f}초 사격 / 취소: "cancel" 또는 x 키')

    camera = Camera(cfg.camera_index, cfg.frame_width, cfg.frame_height)
    detector = Detector(cfg.model_path, cfg.conf_threshold, cfg.target_class, cfg.device)
    tracker = TargetTracker(cfg)
    turret = TurretControl(cfg)

    if cfg.no_serial:
        link = DummyLink()
    else:
        if not cfg.serial_port:
            cfg.serial_port = find_arduino_port()
            if cfg.serial_port:
                print(f"[serial] 아두이노 포트 자동 선택: {cfg.serial_port}")
        if not cfg.serial_port:
            raise ValueError("--no-serial을 쓰지 않으려면 --port로 아두이노 포트를 지정해야 합니다 "
                             "(Windows: COM3 등, Mac: ls /dev/cu.usb* 로 확인)")
        link = TurretSerialLink(cfg.serial_port, cfg.serial_baud)

    cmd_listener = StdinCommandListener()

    def handle_signal(now):
        event = fire_control.signal(now)
        if event == "lead":
            print("[fire_control] 선도각 장입 -> 선도각 적용 상태로 추적")
        elif event == "fire":
            link.send_fire(cfg.fire_duration_s, cfg.motor_speed)
            print(f"[fire_control] 사격 시작 ({cfg.fire_duration_s:.1f}초, 추적 계속)")

    def handle_cancel():
        if fire_control.cancel():
            link.stop_fire()
        print("[fire_control] 취소 -> 선도각 해제, 중앙 추적")

    sweep = args.latency_sweep
    predictor = PredictiveTracker(cfg, turret) if cfg.control_mode == "2dof" else None
    mode_name = "sweep" if sweep else cfg.control_mode
    if sweep:
        print(f"[mode] 지연 측정: {args.sweep_axis}를 ±{args.sweep_amp:.0f}도, 주기 {args.sweep_period:.1f}초로 흔듦 "
              "(추적 안 함). 정지 표적을 화면에 두세요")
        if not cfg.log_csv:
            print("[mode] 경고: --log-csv가 없어 측정 결과가 저장되지 않습니다")
    elif predictor:
        lt = cfg.latency_tilt_s if cfg.latency_tilt_s is not None else cfg.latency_s
        print(f"[mode] 2dof 추적: L pan={cfg.latency_s:.3f}s tilt={lt:.3f}s, 화각 {cfg.hfov_deg}x{cfg.vfov_deg}, Kp={cfg.kp} Ki={cfg.ki} Kd={cfg.kd}, "
              f"불감대 {cfg.deadband_deg}deg, α={cfg.ab_alpha} β={cfg.ab_beta}")
        if cfg.ballistic_json:
            print("[mode] 참고: 2dof는 추적 전용이라 선도각을 적용하지 않습니다 (선도각은 --control legacy)")
    else:
        print(f"[mode] legacy 추적: Kp={cfg.kp} Ki={cfg.ki} Kd={cfg.kd}, 불감대 {cfg.deadband_deg}deg")

    last_center = None
    log_file = None
    log_writer = None
    if cfg.log_csv:
        log_path = Path(cfg.log_csv)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_file = open(log_path, "w", newline="", encoding="utf-8")
        log_writer = csv.DictWriter(log_file, fieldnames=LOG_FIELDS, restval="", extrasaction="ignore")
        log_writer.writeheader()

    prev_time = time.monotonic()
    sweep_t0 = prev_time
    fps = 0.0
    last_status = 0.0
    link.send_angles(turret.pan_angle, turret.tilt_angle)  # 시작 시 중앙으로

    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                break

            h, w = frame.shape[:2]
            frame_cx, frame_cy = w / 2.0, h / 2.0

            result = detector.predict(frame)
            max_jump_px = cfg.max_jump_frac * w if cfg.max_jump_frac > 0 else None
            target = pick_target(result, detector.target_class, last_center, max_jump_px)

            now = time.monotonic()
            timestamp = time.time()

            for line in link.read_responses():
                print(f"[arduino] {line}")

            for cmd in cmd_listener.consume():
                if cmd == "shot":
                    handle_signal(now)
                else:
                    handle_cancel()
            if fire_control.update(now):
                link.stop_fire()  # 펌웨어도 시간 지나면 스스로 끄지만 확실히 한 번 더
                print("[fire_control] 사격 종료 -> 선도각 해제, 중앙 추적 (다시 스페이스 = 선도각 장입)")

            lead_solution = None
            lead_applied = False
            range_m = None
            row = {
                "t_mono": now, "timestamp": f"{timestamp:.3f}", "mode": mode_name,
                "found": int(target is not None), "fire_state": fire_control.state,
                "hfov": cfg.hfov_deg, "vfov": cfg.vfov_deg,
            }

            if target is not None:
                last_center = (target["cx"], target["cy"])
                _, _, err_x_deg, err_y_deg = tracker.pixel_error_deg(target, w, h)
                row.update({"conf": target["conf"], "bbox_w": target["w"], "bbox_h": target["h"],
                            "err_pan_deg": err_x_deg, "err_tilt_deg": err_y_deg})

            if sweep:
                # 지연 측정: 추적하지 않고 한 축을 사인파로 움직인다 (다른 축은 중앙 고정)
                swing = args.sweep_amp * math.sin(2.0 * math.pi * (now - sweep_t0) / args.sweep_period)
                turret.pan_angle = cfg.pan_center_deg + (swing if args.sweep_axis == "pan" else 0.0)
                turret.tilt_angle = cfg.tilt_center_deg + (swing if args.sweep_axis == "tilt" else 0.0)
                link.send_angles(turret.pan_angle, turret.tilt_angle)
            elif predictor is not None:
                if target is not None:
                    row.update(predictor.on_measurement(now, err_x_deg, err_y_deg))
                    link.send_angles(turret.pan_angle, turret.tilt_angle)
                else:
                    info = predictor.on_lost(now)
                    row.update(info)
                    if info["ff_scale"] > 0:
                        link.send_angles(turret.pan_angle, turret.tilt_angle)  # 예측으로 계속 회전
                    if turret.handle_lost():
                        last_center = None  # 오래 놓쳤으면 위치 게이팅 해제
                        if fire_control.is_firing():
                            link.stop_fire()
                            print("[fire_control] 표적 상실 -> 사격 중지")
                        fire_control.reset()
            elif target is not None:
                feedforward = (0.0, 0.0)

                if fire_control.model is not None:
                    yaw_bearing_deg, pitch_bearing_deg = tracker.bearing_deg(
                        turret.pan_angle, turret.tilt_angle, err_x_deg, err_y_deg
                    )
                    range_m = args.range_m if args.range_m is not None else tracker.estimate_range_m(target["w"], w)

                    if range_m is not None:
                        x, y, z = tracker.update_position(now, yaw_bearing_deg, pitch_bearing_deg, range_m)
                        vx, vy, vz = tracker.estimate_velocity()
                        feedforward = tracker.estimate_bearing_rate_deg_s()

                        lead_solution = fire_control.solve(
                            now, turret_pos=(0.0, 0.0, 0.0),
                            target_pos=(x, y, z), target_velocity=(vx, vy, vz),
                        )
                        if lead_solution is not None:
                            lead_solution["range_m"] = range_m

                # 선도각은 항상 계산해 화면에 보여주지만, 목표각에는 신호 1 이후(LEAD/FIRING)에만 넣는다
                lead_applied = fire_control.lead_active() and lead_solution is not None
                pan_error, tilt_error = tracker.pid_errors(
                    turret.pan_angle, turret.tilt_angle, err_x_deg, err_y_deg,
                    lead_solution=lead_solution if lead_applied else None,
                    feedforward_deg_per_s=feedforward,
                )
                pan_angle, tilt_angle = turret.update(pan_error, tilt_error, now)
                link.send_angles(pan_angle, tilt_angle)
                row.update({
                    "range_m": range_m if range_m else "",
                    "lead_yaw_deg": lead_solution["yaw_deg"] if lead_solution else "",
                    "lead_pitch_deg": lead_solution["pitch_deg"] if lead_solution else "",
                    "lead_applied": int(lead_applied),
                })
            else:
                just_reset = turret.handle_lost()
                if just_reset:
                    last_center = None  # 오래 놓쳤으면 위치 게이팅 해제 (재포착 시 전체에서 다시 탐색)
                    tracker.reset()
                    if fire_control.is_firing():
                        link.stop_fire()  # 표적을 오래 놓치면 사격 중지
                        print("[fire_control] 표적 상실 -> 사격 중지, 선도각 해제")
                    fire_control.reset()

            if log_writer:
                row["servo_pan"] = turret.pan_angle
                row["servo_tilt"] = turret.tilt_angle
                log_writer.writerow({k: _fmt(v) for k, v in row.items()})

            # 1초마다 상태 출력 (표적 탐지 여부 / 보내는 각도 확인용)
            if now - last_status >= 1.0:
                last_status = now
                if log_file:
                    log_file.flush()  # 강제 종료돼도 로그가 남도록
                if target is not None:
                    omega = ""
                    if "omega_pan" in row:
                        omega = f" ω=({row['omega_pan']:+.1f},{row['omega_tilt']:+.1f})deg/s"
                    print(f"[status] 표적={target['cls_name']} {target['conf']:.2f} "
                          f"err=({err_x_deg:+.1f},{err_y_deg:+.1f})deg{omega} -> "
                          f"s2 {round(turret.pan_angle)} / s3 {round(turret.tilt_angle)}")
                else:
                    seen = [] if result.boxes is None else sorted(
                        {result.names[int(b.cls[0])] for b in result.boxes})
                    print(f"[status] 표적 없음 (탐지된 클래스 {seen}, 찾는 클래스 '{cfg.target_class}', "
                          f"모델 클래스 {list(result.names.values())}) "
                          f"s2 {round(turret.pan_angle)} / s3 {round(turret.tilt_angle)}")

            dt = now - prev_time
            prev_time = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)

            if not cfg.no_show:
                raw_frame = frame.copy()
                draw_overlay(frame, target, frame_cx, frame_cy, turret.pan_angle, turret.tilt_angle,
                             fps, lead_solution, fire_control.state, fire_control.fire_remaining(now))
                cv2.imshow("SIOR - Stage2 Target Tracking", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("s"):
                    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
                    tag = "det" if target is not None else "miss"
                    out = CAPTURE_DIR / f"{time.strftime('%Y%m%d_%H%M%S')}_{int(time.time() * 1000) % 1000:03d}_{tag}.jpg"
                    cv2.imwrite(str(out), raw_frame)
                    print(f"프레임 저장: {out}")
                if key == ord(" "):
                    handle_signal(now)
                if key == ord("x"):
                    handle_cancel()
                if key == ord("r"):
                    turret.reset()
                    last_center = None
                    tracker.reset()
                    if predictor is not None:
                        predictor.reset()
                        predictor.record(now)
                    fire_control.reset()
                    link.stop_fire()
                    link.send_angles(turret.pan_angle, turret.tilt_angle)
    finally:
        link.stop_fire()
        camera.release()
        cv2.destroyAllWindows()
        link.close()
        if log_file:
            log_file.close()


if __name__ == "__main__":
    main()