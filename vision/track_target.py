# -*- coding: utf-8 -*-
"""
실험계획 2단계: YOLO 기반 단일 표적 자율 추적 (2축 pan/tilt PID 제어)

카메라 영상에서 YOLO로 표적을 탐지하고, 프레임 중심과 표적 중심의 픽셀 오차를
화각(FOV) 기준 각도 오차로 변환한 뒤 2축(pan/tilt) PID로 보정해 서보 각도
명령을 아두이노로 전송한다. IP나 가변저항 피드백 없이 오픈루프로 동작한다.

사용 예:
    python track_target.py --model ../checkpoint/best.pt --camera 0 \
        --port /dev/tty.usbmodem14101

    # 아두이노 없이 비전/PID 로직만 확인
    python track_target.py --model ../checkpoint/best.pt --camera 0 --no-serial

    # 거리/크기별 실험 로그 저장 (근거리/원거리 비교 등)
    python track_target.py --model ../checkpoint/best.pt --no-serial \
        --log-csv logs/near_small.csv

    # 탄도 모델(ballistic/lead_solver.py)을 연결해 리드샷(선도각) 계산 모드로 실행
    # (거리 센서가 없어 bbox 폭 기반 단안 거리 추정을 쓰므로 정밀도는 낮음)
    python track_target.py --model ../checkpoint/best.pt --no-serial \
        --ballistic-json ../ballistic/ballistic_model.json --target-width-m 0.03
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import cv2
from ultralytics import YOLO

from lead_estimator import LeadEstimator
from pid import PID
from serial_link import DummyLink, TurretSerialLink

PAN_CENTER = 90.0
TILT_CENTER = 90.0
PAN_LIMITS = (10.0, 170.0)
TILT_LIMITS = (20.0, 160.0)
MAX_STEP_DEG = 8.0            # 프레임당 최대 서보 이동각 (오버슈트/진동 억제)
LOST_TARGET_HOLD_FRAMES = 15  # 이 프레임 수만큼 표적을 놓치면 PID를 리셋하고 위치 유지


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
        "--target-class",
        default="target",
        help="탐지 클래스 이름으로 필터링 (체크포인트의 단일 클래스 이름이 'target'). "
             "빈 문자열(\"\")을 넘기면 클래스 무시하고 최고 신뢰도 박스를 사용",
    )
    p.add_argument("--hfov", type=float, default=60.0, help="카메라 수평 화각(도), 사용 카메라에 맞게 보정 필요")
    p.add_argument("--vfov", type=float, default=45.0, help="카메라 수직 화각(도), 사용 카메라에 맞게 보정 필요")
    p.add_argument("--pan-sign", type=float, default=1.0, choices=[-1.0, 1.0], help="포탑 장착 방향에 따른 부호 보정")
    p.add_argument("--tilt-sign", type=float, default=1.0, choices=[-1.0, 1.0], help="포신 장착 방향에 따른 부호 보정")
    p.add_argument("--port", default=None, help="아두이노 시리얼 포트 (예: /dev/tty.usbmodem14101)")
    p.add_argument("--baud", type=int, default=9600, help="ardoino/ 테스트 스케치와 동일한 9600 baud")
    p.add_argument("--no-serial", action="store_true", help="시리얼 연결 없이 비전/PID 로직만 테스트")
    p.add_argument("--log-csv", default=None, help="프레임별 추적 로그 CSV 경로 (거리/크기별 실험 비교용)")
    p.add_argument("--kp", type=float, default=0.35)
    p.add_argument("--ki", type=float, default=0.02)
    p.add_argument("--kd", type=float, default=0.08)
    p.add_argument("--no-show", action="store_true", help="영상 미리보기 창을 띄우지 않음")

    p.add_argument(
        "--ballistic-json", default=None,
        help="ballistic/ballistic_model_fit.py로 만든 v0/Cd json 경로. 지정하면 단순 "
             "화면 중앙 정렬 대신 ballistic/lead_solver.py로 계산한 리드샷(선도각)을 목표로 서보를 움직임",
    )
    p.add_argument(
        "--target-width-m", type=float, default=None,
        help="--ballistic-json 사용 시 필수. 표적의 실제 폭(m) - bbox 폭 기반 단안 거리 추정에 사용",
    )
    p.add_argument("--lead-solve-interval", type=float, default=0.2, help="리드샷 재계산 주기(초)")

    p.add_argument(
        "--max-jump-frac", type=float, default=0.35,
        help="직전 표적 위치에서 프레임 폭 대비 이 비율보다 멀리 떨어진 박스는 (다른 후보가 "
             "있으면) 무시 - 엉뚱한 물체로 튀는 것 방지. 0이면 끔",
    )
    return p.parse_args()


def pick_target(result, target_class, last_center=None, max_jump_px=None):
    """프레임에서 표적 하나를 선택한다 (단일 표적 실험이므로 기본은 최고 신뢰도 박스).

    last_center/max_jump_px가 주어지면, 직전에 표적이 있던 위치에서 max_jump_px보다
    멀리 떨어진 후보는 (가까운 후보가 하나라도 있으면) 제외한다. 학습 환경과 다른
    곳에서 엉뚱한 물체가 순간적으로 더 높은 신뢰도로 잡혀 그쪽으로 튀는 것을 막기 위함.
    """
    boxes = result.boxes
    if boxes is None or len(boxes) == 0:
        return None

    candidates = []
    for box in boxes:
        cls_id = int(box.cls[0])
        cls_name = result.names[cls_id]
        if target_class is not None and cls_name != target_class:
            continue
        conf = float(box.conf[0])
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        candidates.append({
            "conf": conf,
            "cls_name": cls_name,
            "x1": x1, "y1": y1, "x2": x2, "y2": y2,
            "cx": (x1 + x2) / 2.0,
            "cy": (y1 + y2) / 2.0,
            "w": x2 - x1,
            "h": y2 - y1,
        })

    if not candidates:
        return None

    if last_center is not None and max_jump_px:
        near = [
            c for c in candidates
            if math.hypot(c["cx"] - last_center[0], c["cy"] - last_center[1]) <= max_jump_px
        ]
        if near:
            candidates = near  # 가까운 후보가 있으면 그중에서만 고름, 없으면 전체에서 고름(재포착)

    return max(candidates, key=lambda c: c["conf"])


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def draw_overlay(frame, target, frame_cx, frame_cy, pan_angle, tilt_angle, fps, lead_solution=None):
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


def main():
    args = parse_args()

    target_class = args.target_class or None  # 빈 문자열이면 클래스 필터링 없이 최고 신뢰도 박스 사용

    lead_estimator = None
    if args.ballistic_json:
        if args.target_width_m is None:
            raise ValueError("--ballistic-json을 쓰려면 --target-width-m(표적 실제 폭, m)도 지정해야 합니다")
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "ballistic"))
        from lead_solver import BallisticModel  # noqa: E402 (조건부 로딩이라 지연 import)

        ballistic_model = BallisticModel.from_json(args.ballistic_json)
        lead_estimator = LeadEstimator(
            ballistic_model, args.target_width_m, solve_interval=args.lead_solve_interval
        )
        print(f"리드샷 모드 활성화: {ballistic_model}")

    model = YOLO(args.model)

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise RuntimeError(f"카메라를 열 수 없습니다: index={args.camera}")

    if args.no_serial:
        link = DummyLink()
    else:
        if not args.port:
            raise ValueError("--no-serial을 쓰지 않으려면 --port로 아두이노 포트를 지정해야 합니다")
        link = TurretSerialLink(args.port, args.baud)

    pid_pan = PID(args.kp, args.ki, args.kd, output_limits=(-MAX_STEP_DEG, MAX_STEP_DEG))
    pid_tilt = PID(args.kp, args.ki, args.kd, output_limits=(-MAX_STEP_DEG, MAX_STEP_DEG))

    pan_angle, tilt_angle = PAN_CENTER, TILT_CENTER
    lost_frames = 0
    last_center = None

    log_file = None
    log_writer = None
    if args.log_csv:
        log_path = Path(args.log_csv)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_file = open(log_path, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow([
            "timestamp", "found", "conf", "cls_name", "bbox_w", "bbox_h", "bbox_area",
            "err_x_px", "err_y_px", "pan_angle", "tilt_angle",
            "range_m", "lead_yaw_deg", "lead_pitch_deg",
        ])

    prev_time = time.monotonic()
    fps = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            h, w = frame.shape[:2]
            frame_cx, frame_cy = w / 2.0, h / 2.0

            results = model.predict(frame, conf=args.conf, verbose=False)
            max_jump_px = args.max_jump_frac * w if args.max_jump_frac > 0 else None
            target = pick_target(results[0], target_class, last_center, max_jump_px)

            now = time.monotonic()
            timestamp = time.time()

            lead_solution = None

            if target is not None:
                lost_frames = 0
                last_center = (target["cx"], target["cy"])
                err_x_px = target["cx"] - frame_cx
                err_y_px = target["cy"] - frame_cy
                err_x_deg = (err_x_px / (w / 2.0)) * (args.hfov / 2.0)
                err_y_deg = (err_y_px / (h / 2.0)) * (args.vfov / 2.0)

                pan_pid_error = err_x_deg
                tilt_pid_error = err_y_deg

                if lead_estimator is not None:
                    # 화면 중앙 정렬 대신, 현재 포탑 각도 + 남은 픽셀 오차로 표적의
                    # 실제 베어링을 구하고 그 위치/속도로 리드샷(선도각)을 계산한다.
                    yaw_bearing_deg = (pan_angle - PAN_CENTER) + args.pan_sign * err_x_deg
                    pitch_bearing_deg = -((tilt_angle - TILT_CENTER) + args.tilt_sign * err_y_deg)
                    range_m = lead_estimator.estimate_range_m(target["w"], w, args.hfov)

                    if range_m is not None:
                        lead_solution = lead_estimator.update(now, yaw_bearing_deg, pitch_bearing_deg, range_m)

                    if lead_solution is not None:
                        pan_setpoint = PAN_CENTER + lead_solution["yaw_deg"]
                        tilt_setpoint = TILT_CENTER - lead_solution["pitch_deg"]
                        # 목표각-현재각 차이를 PID에 넣는 것이므로, 리드샷이 필요 없는
                        # 정지 표적일 때는 그대로 err_x_deg/err_y_deg로 수렴한다.
                        pan_pid_error = args.pan_sign * (pan_setpoint - pan_angle)
                        tilt_pid_error = args.tilt_sign * (tilt_setpoint - tilt_angle)

                pan_angle += args.pan_sign * pid_pan.update(pan_pid_error, now)
                tilt_angle += args.tilt_sign * pid_tilt.update(tilt_pid_error, now)
                pan_angle = clamp(pan_angle, *PAN_LIMITS)
                tilt_angle = clamp(tilt_angle, *TILT_LIMITS)

                link.send_angles(pan_angle, tilt_angle)

                if log_writer:
                    range_str = f"{lead_solution['range_m']:.3f}" if lead_solution else ""
                    lead_yaw_str = f"{lead_solution['yaw_deg']:.2f}" if lead_solution else ""
                    lead_pitch_str = f"{lead_solution['pitch_deg']:.2f}" if lead_solution else ""
                    log_writer.writerow([
                        timestamp, 1, f"{target['conf']:.3f}", target["cls_name"],
                        f"{target['w']:.1f}", f"{target['h']:.1f}", f"{target['w'] * target['h']:.1f}",
                        f"{err_x_px:.1f}", f"{err_y_px:.1f}", f"{pan_angle:.1f}", f"{tilt_angle:.1f}",
                        range_str, lead_yaw_str, lead_pitch_str,
                    ])
            else:
                lost_frames += 1
                if lost_frames == LOST_TARGET_HOLD_FRAMES:
                    pid_pan.reset()
                    pid_tilt.reset()
                    last_center = None  # 오래 놓쳤으면 위치 게이팅 해제 (재포착 시 전체에서 다시 탐색)
                    if lead_estimator is not None:
                        lead_estimator.reset()
                if log_writer:
                    log_writer.writerow(
                        [timestamp, 0, "", "", "", "", "", "", "", f"{pan_angle:.1f}", f"{tilt_angle:.1f}",
                         "", "", ""]
                    )

            dt = now - prev_time
            prev_time = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)

            if not args.no_show:
                draw_overlay(frame, target, frame_cx, frame_cy, pan_angle, tilt_angle, fps, lead_solution)
                cv2.imshow("SIOR - Stage2 Target Tracking", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("r"):
                    pan_angle, tilt_angle = PAN_CENTER, TILT_CENTER
                    pid_pan.reset()
                    pid_tilt.reset()
                    last_center = None
                    if lead_estimator is not None:
                        lead_estimator.reset()
                    link.send_angles(pan_angle, tilt_angle)
    finally:
        cap.release()
        cv2.destroyAllWindows()
        link.close()
        if log_file:
            log_file.close()


if __name__ == "__main__":
    main()
