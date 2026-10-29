# -*- coding: utf-8 -*-
"""
실험계획 2단계: YOLO 기반 단일 표적 자율 추적 (2축 pan/tilt PID 제어) + 리드샷(선도각)
진입점. 인자 파싱, 각 모듈(vision/track_target/turret_control/fire_control/serial_link)
생성과 연결, 메인 루프, 키 입력, 화면 오버레이·FPS 표시, CSV 로그를 담당한다.

사격 (전동건, DC모터 1개) - 신호는 콘솔에 "shot" 입력 또는 영상 창에서 스페이스바:
    신호 1: 선도각 장입 -> 포가 선도각만큼 이동, 선도각이 들어간 상태로 계속 추적
    신호 2: 추적을 계속하면서 --fire-duration초 동안 사격 (끝나면 선도각 상태 유지)
    취소  : 콘솔 "cancel" 또는 x 키 -> 사격 중지 + 선도각 해제(중앙 추적으로 복귀)
키: q=종료, r=리셋(중앙 복귀+사격 중지), s=현재 원본 프레임 저장, x=취소, 스페이스=신호

사용 예 (Windows, Imp_Window 폴더에서. 포트 번호는 장치 관리자 > 포트(COM & LPT)에서 확인):
    python main.py --model ..\\checkpoint\\best.pt --camera 0 --port COM3

    # 아두이노 없이 비전/PID 로직만 확인
    python main.py --model ..\\checkpoint\\best.pt --camera 0 --no-serial

    # 거리/크기별 실험 로그 저장 (근거리/원거리 비교 등)
    python main.py --model ..\\checkpoint\\best.pt --no-serial --log-csv logs\\near_small.csv

    # 탄도 모델을 연결해 선도각 계산 + 2단계 사격 (2초 연사)
    python main.py --model ..\\checkpoint\\best.pt --port COM3 --ballistic-json ballistic\\ballistic_model.json --target-width-m 0.03 --fire-duration 2
"""

import argparse
import csv
import sys
import threading
import time
from pathlib import Path

import cv2

from config import Config
from fire_control import FireControl
from serial_link import DummyLink, TurretSerialLink
from track_target import TargetTracker
from turret_control import TurretControl
from vision import Camera, Detector, pick_target

# s 키로 저장하는 실시간 원본 프레임 (오버레이 없음) - 실패 장면을 모아 재학습에 쓰기 위함
CAPTURE_DIR = Path(__file__).resolve().parent.parent / "dataset" / "live_capture"


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--model", default="../checkpoint/best.pt")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--conf", type=float, default=0.4)
    p.add_argument(
        "--target-class", default="target",
        help="탐지 클래스 이름으로 필터링. 빈 문자열(\"\")을 넘기면 클래스 무시하고 최고 신뢰도 박스를 사용",
    )
    p.add_argument("--hfov", type=float, default=60.0, help="카메라 수평 화각(도)")
    p.add_argument("--vfov", type=float, default=45.0, help="카메라 수직 화각(도)")
    p.add_argument(
        "--pan-sign", type=float, default=-1.0, choices=[-1.0, 1.0],
        help="포탑 장착 방향에 따른 pan 부호 보정 "
             "(2026-09-28 실측: 표적이 오른쪽으로 갈 때 포탑이 왼쪽으로 돌던 반전 현상을 -1.0 기본값으로 교정)",
    )
    p.add_argument("--tilt-sign", type=float, default=1.0, choices=[-1.0, 1.0], help="tilt 부호 보정")
    p.add_argument("--pan-zero-deg", type=float, default=0.0, help="카메라 광축-포신 정렬 오차 보정(pan)")
    p.add_argument("--tilt-zero-deg", type=float, default=0.0, help="카메라 광축-포신 정렬 오차 보정(tilt)")
    p.add_argument("--port", default=None, help="아두이노 시리얼 포트 (예: COM3, 장치 관리자에서 확인)")
    p.add_argument("--baud", type=int, default=9600)
    p.add_argument("--no-serial", action="store_true", help="시리얼 연결 없이 비전/PID 로직만 테스트")
    p.add_argument("--log-csv", default=None, help="프레임별 추적 로그 CSV 경로")
    p.add_argument("--kp", type=float, default=0.35)
    p.add_argument("--ki", type=float, default=0.02)
    p.add_argument("--kd", type=float, default=0.08)
    p.add_argument("--no-show", action="store_true", help="영상 미리보기 창을 띄우지 않음")

    p.add_argument(
        "--ballistic-json", default=None,
        help="ballistic/ballistic_model_fit.py로 만든 v0/Cd json 경로. 없으면 선도각 0으로 동작",
    )
    p.add_argument("--target-width-m", type=float, default=None, help="--ballistic-json 사용 시 필수")
    p.add_argument("--lead-solve-interval", type=float, default=0.2, help="선도각 재계산 주기(초)")
    p.add_argument("--fire-duration", type=float, default=1.0, help="신호 2에서 사격할 시간 n초 (최대 5)")
    p.add_argument("--motor-speed", type=int, default=255, help="전동건 모터 PWM 세기(0~255)")

    p.add_argument(
        "--max-jump-frac", type=float, default=0.35,
        help="직전 표적 위치에서 프레임 폭 대비 이 비율보다 멀리 떨어진 박스는 무시. 0이면 끔",
    )
    return p.parse_args()


def build_config(args):
    return Config(
        model_path=args.model,
        camera_index=args.camera,
        frame_width=args.width,
        frame_height=args.height,
        conf_threshold=args.conf,
        target_class=args.target_class,
        max_jump_frac=args.max_jump_frac,
        hfov_deg=args.hfov,
        vfov_deg=args.vfov,
        pan_sign=args.pan_sign,
        tilt_sign=args.tilt_sign,
        pan_zero_deg=args.pan_zero_deg,
        tilt_zero_deg=args.tilt_zero_deg,
        kp=args.kp, ki=args.ki, kd=args.kd,
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

    if cfg.ballistic_json and cfg.target_width_m is None:
        raise ValueError("--ballistic-json을 쓰려면 --target-width-m(표적 실제 폭, m)도 지정해야 합니다")
    fire_control = FireControl(cfg)
    print('신호("shot" 입력 또는 스페이스바): 1번째 = 선도각 장입, 2번째 = '
          f'{cfg.fire_duration_s:.1f}초 사격 / 취소: "cancel" 또는 x 키')

    camera = Camera(cfg.camera_index, cfg.frame_width, cfg.frame_height)
    detector = Detector(cfg.model_path, cfg.conf_threshold, cfg.target_class)
    tracker = TargetTracker(cfg)
    turret = TurretControl(cfg)

    if cfg.no_serial:
        link = DummyLink()
    else:
        if not cfg.serial_port:
            raise ValueError("--no-serial을 쓰지 않으려면 --port로 아두이노 포트를 지정해야 합니다")
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

    last_center = None
    log_file = None
    log_writer = None
    if cfg.log_csv:
        log_path = Path(cfg.log_csv)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_file = open(log_path, "w", newline="", encoding="utf-8")
        log_writer = csv.writer(log_file)
        log_writer.writerow([
            "timestamp", "found", "conf", "cls_name", "bbox_w", "bbox_h", "bbox_area",
            "err_x_px", "err_y_px", "pan_angle", "tilt_angle",
            "range_m", "lead_yaw_deg", "lead_pitch_deg", "fire_state", "lead_applied",
        ])

    prev_time = time.monotonic()
    fps = 0.0

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

            for cmd in cmd_listener.consume():
                if cmd == "shot":
                    handle_signal(now)
                else:
                    handle_cancel()
            if fire_control.update(now):
                link.stop_fire()  # 펌웨어도 시간 지나면 스스로 끄지만 확실히 한 번 더
                print("[fire_control] 사격 종료 -> 선도각 유지 상태로 추적")

            lead_solution = None
            lead_applied = False
            range_m = None

            if target is not None:
                last_center = (target["cx"], target["cy"])
                err_x_px, err_y_px, err_x_deg, err_y_deg = tracker.pixel_error_deg(target, w, h)

                feedforward = (0.0, 0.0)

                if fire_control.model is not None:
                    yaw_bearing_deg, pitch_bearing_deg = tracker.bearing_deg(
                        turret.pan_angle, turret.tilt_angle, err_x_deg, err_y_deg
                    )
                    range_m = tracker.estimate_range_m(target["w"], w)

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

                if log_writer:
                    log_writer.writerow([
                        timestamp, 1, f"{target['conf']:.3f}", target["cls_name"],
                        f"{target['w']:.1f}", f"{target['h']:.1f}", f"{target['w'] * target['h']:.1f}",
                        f"{err_x_px:.1f}", f"{err_y_px:.1f}", f"{pan_angle:.1f}", f"{tilt_angle:.1f}",
                        f"{range_m:.3f}" if range_m else "",
                        f"{lead_solution['yaw_deg']:.2f}" if lead_solution else "",
                        f"{lead_solution['pitch_deg']:.2f}" if lead_solution else "",
                        fire_control.state, int(lead_applied),
                    ])
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
                    log_writer.writerow([
                        timestamp, 0, "", "", "", "", "", "", "",
                        f"{turret.pan_angle:.1f}", f"{turret.tilt_angle:.1f}", "", "", "",
                        fire_control.state, 0,
                    ])

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
