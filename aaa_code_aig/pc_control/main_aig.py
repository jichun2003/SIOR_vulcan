# -*- coding: utf-8 -*-
"""
PC 제어형 진입점: AI-G 보드는 NPU 검출만 하고, 추적/PID/사격 통제는 PC에서 한다.

  [AI-G 보드] 카메라 -> NPU -> 박스  --TCP JSON-->  [PC] 이 파일
                                                      -> aaa_code_1111의 추적/PID 그대로
                                                      -> pyserial로 아두이노

즉 검증이 끝난 aaa_code_1111의 제어 코드(tracking_filter / turret_control / track_target /
fire_control / serial_link)를 **그대로 import해서 쓴다**. 복사본을 만들지 않으므로
1111을 고치면 여기도 같이 바뀐다.

보드 쪽 실행:
    ./aig_turret --mode detect-only --network /home/root/yolov8s_target_quantized --json-port 5000

PC 쪽 실행 (보드 IP는 board_check.sh의 `ip a` 결과):
    python main_aig.py --board 192.168.0.100 --port COM3
    python main_aig.py --board 192.168.0.100 --no-serial          # 아두이노 없이 로직만
    python main_aig.py --board 192.168.0.100 --no-serial --log-csv logs/aig.csv

신호 (터미널에 입력): shot = 1번째 선도각 장입 / 2번째 사격, cancel = 취소, r = 리셋, q = 종료
"""

import argparse
import csv
import sys
import threading
import time
from pathlib import Path

import aig_link

# aaa_code_1111의 모듈을 그대로 쓴다 (복사하지 않음)
DEFAULT_CODE_DIR = Path(__file__).resolve().parent.parent.parent / "aaa_code_1111"

LOG_FIELDS = [
    "t_mono", "timestamp", "mode", "found", "conf", "bbox_w", "bbox_h",
    "err_pan_deg", "err_tilt_deg", "servo_pan", "servo_tilt",
    "meas_pan", "meas_tilt", "est_pan", "est_tilt", "omega_pan", "omega_tilt",
    "pred_pan", "pred_tilt", "resid_pan", "resid_tilt", "filter_reset", "outlier", "ff_scale",
    "infer_ms", "fire_state", "hfov", "vfov",
]


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--board", required=True, help="AI-G 보드 IP")
    p.add_argument("--board-port", type=int, default=5000, help="보드 json-port (기본 5000)")
    p.add_argument("--code-dir", default=str(DEFAULT_CODE_DIR),
                   help="제어 모듈을 가져올 폴더 (기본: ../../aaa_code_1111)")
    p.add_argument("--port", default=None, help="아두이노 시리얼 포트 (예: COM3, /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=9600)
    p.add_argument("--no-serial", action="store_true", help="시리얼 없이 추적 로직만")
    p.add_argument("--target-class", default="target", help="빈 문자열이면 클래스 무시")
    p.add_argument("--max-jump-frac", type=float, default=None)
    p.add_argument("--hfov", type=float, default=None)
    p.add_argument("--vfov", type=float, default=None)
    p.add_argument("--kp", type=float, default=None)
    p.add_argument("--ki", type=float, default=None)
    p.add_argument("--kd", type=float, default=None)
    p.add_argument("--deadband", type=float, default=None)
    p.add_argument("--max-rate", type=float, default=None)
    p.add_argument("--err-filter", type=float, default=None)
    p.add_argument("--latency", type=float, default=None,
                   help="지연 L(초). 보드+NPU 기준으로 재측정한 값을 넣어야 한다")
    p.add_argument("--latency-tilt", type=float, default=None)
    p.add_argument("--ab-alpha", type=float, default=None)
    p.add_argument("--ab-beta", type=float, default=None)
    p.add_argument("--pan-sign", type=float, default=None, choices=[-1.0, 1.0])
    p.add_argument("--tilt-sign", type=float, default=None, choices=[-1.0, 1.0])
    p.add_argument("--fire-duration", type=float, default=None)
    p.add_argument("--motor-speed", type=int, default=None)
    p.add_argument("--log-csv", default=None)
    return p.parse_args()


class StdinCommandListener:
    """메인 루프를 막지 않고 터미널 입력을 받는다 (1111 main.py와 같은 방식)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._pending = []
        threading.Thread(target=self._listen, daemon=True).start()

    def _listen(self):
        for line in sys.stdin:
            cmd = line.strip().lower()
            if cmd in ("shot", "cancel", "r", "q"):
                with self._lock:
                    self._pending.append(cmd)

    def consume(self):
        with self._lock:
            cmds, self._pending = self._pending, []
        return cmds


def build_config(args, Config):
    tuning = {
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
        "pan_sign": args.pan_sign,
        "tilt_sign": args.tilt_sign,
        "max_jump_frac": args.max_jump_frac,
        "fire_duration_s": args.fire_duration,
        "motor_speed": args.motor_speed,
    }
    tuning = {k: v for k, v in tuning.items() if v is not None}
    return Config(
        control_mode="2dof",
        target_class=args.target_class,
        serial_port=args.port,
        serial_baud=args.baud,
        no_serial=args.no_serial,
        log_csv=args.log_csv,
        **tuning,
    )


def _fmt(v):
    return f"{v:.4f}" if isinstance(v, float) else v


def main():
    args = parse_args()

    code_dir = Path(args.code_dir).resolve()
    if not (code_dir / "tracking_filter.py").is_file():
        raise SystemExit(
            f"제어 모듈을 찾을 수 없습니다: {code_dir}\n"
            "--code-dir로 aaa_code_1111 폴더를 지정하세요"
        )
    sys.path.insert(0, str(code_dir))

    from config import Config
    from fire_control import FireControl
    from track_target import TargetTracker
    from tracking_filter import PredictiveTracker
    from turret_control import TurretControl

    cfg = build_config(args, Config)
    print(f"[aig] 제어 모듈: {code_dir}")
    lt = cfg.latency_tilt_s if cfg.latency_tilt_s is not None else cfg.latency_s
    print(f"[aig] 2dof: L pan={cfg.latency_s:.3f}s tilt={lt:.3f}s, 화각 {cfg.hfov_deg}x{cfg.vfov_deg}, "
          f"Kp={cfg.kp} Ki={cfg.ki} Kd={cfg.kd}, 불감대 {cfg.deadband_deg}deg, "
          f"α={cfg.ab_alpha} β={cfg.ab_beta}")
    if args.latency is not None and args.latency_tilt is None:
        print(f"[aig] 주의: --latency만 주면 tilt 지연은 config.py 값({cfg.latency_tilt_s})이 그대로 쓰입니다. "
              "두 축을 같이 쓰려면 --latency-tilt도 주세요")
    if cfg.latency_s >= 0.2:
        print("[aig] 주의: L이 0.2s 이상입니다. 보드 NPU는 추론이 ~15ms라 L이 훨씬 작을 것이니 "
              "--latency로 실측값을 넣으세요 (틀리면 과보상으로 오차가 몇 배 커집니다)")

    link = aig_link.AigDetectionLink(args.board, args.board_port,
                                     target_class=0, class_names=(args.target_class or "target",))
    tracker = TargetTracker(cfg)
    turret = TurretControl(cfg)
    predictor = PredictiveTracker(cfg, turret)
    fire_control = FireControl(cfg)
    cmd_listener = StdinCommandListener()

    if cfg.no_serial:
        from serial_link import DummyLink
        serial_out = DummyLink()
    else:
        if not cfg.serial_port:
            raise SystemExit("--no-serial을 쓰지 않으려면 --port로 아두이노 포트를 지정하세요")
        from serial_link import TurretSerialLink
        serial_out = TurretSerialLink(cfg.serial_port, cfg.serial_baud)

    log_file = None
    log_writer = None
    if cfg.log_csv:
        path = Path(cfg.log_csv)
        path.parent.mkdir(parents=True, exist_ok=True)
        log_file = open(path, "w", newline="", encoding="utf-8")
        log_writer = csv.DictWriter(log_file, fieldnames=LOG_FIELDS, restval="",
                                    extrasaction="ignore")
        log_writer.writeheader()

    last_center = None
    last_status = 0.0
    frames = 0
    serial_out.send_angles(turret.pan_angle, turret.tilt_angle)

    try:
        while True:
            dets, meta = link.read_latest()
            now = time.monotonic()

            for line in serial_out.read_responses():
                print(f"[arduino] {line}")

            for cmd in cmd_listener.consume():
                if cmd == "shot":
                    event = fire_control.signal(now)
                    if event == "lead":
                        print("[fire_control] 선도각 장입")
                    elif event == "fire":
                        serial_out.send_fire(cfg.fire_duration_s, cfg.motor_speed)
                        print(f"[fire_control] 사격 시작 ({cfg.fire_duration_s:.1f}초)")
                elif cmd == "cancel":
                    if fire_control.cancel():
                        serial_out.stop_fire()
                    print("[fire_control] 취소 -> 중앙 추적")
                elif cmd == "r":
                    turret.reset()
                    tracker.reset()
                    predictor.reset()
                    predictor.record(now)
                    fire_control.reset()
                    last_center = None
                    serial_out.stop_fire()
                    serial_out.send_angles(turret.pan_angle, turret.tilt_angle)
                    print("[reset] 중앙 복귀")
                elif cmd == "q":
                    return
            if fire_control.update(now):
                serial_out.stop_fire()
                print("[fire_control] 사격 종료 -> 중앙 추적")

            if dets is None:
                # 아직 프레임이 안 왔거나 통신이 밀림. 제어는 하지 않고 잠깐 쉰다
                # (보드가 끊긴 걸 "표적 상실"로 처리하면 포가 멋대로 coast 하므로 구분한다)
                time.sleep(0.002)
                continue

            frames += 1
            frame_w, frame_h = meta["frame_w"], meta["frame_h"]
            max_jump_px = cfg.max_jump_frac * frame_w if cfg.max_jump_frac > 0 else None
            target = aig_link.pick_target(
                dets, cfg.target_class or None, last_center, max_jump_px)

            row = {
                "t_mono": now, "timestamp": f"{time.time():.3f}", "mode": "2dof-aig",
                "found": int(target is not None), "fire_state": fire_control.state,
                "infer_ms": meta.get("infer_ms", 0.0),
                "hfov": cfg.hfov_deg, "vfov": cfg.vfov_deg,
            }

            if target is not None:
                last_center = (target["cx"], target["cy"])
                _, _, err_x_deg, err_y_deg = tracker.pixel_error_deg(target, frame_w, frame_h)
                row.update({"conf": target["conf"], "bbox_w": target["w"], "bbox_h": target["h"],
                            "err_pan_deg": err_x_deg, "err_tilt_deg": err_y_deg})
                row.update(predictor.on_measurement(now, err_x_deg, err_y_deg))
                serial_out.send_angles(turret.pan_angle, turret.tilt_angle)
            else:
                info = predictor.on_lost(now)
                row.update(info)
                if info["ff_scale"] > 0:
                    serial_out.send_angles(turret.pan_angle, turret.tilt_angle)
                if turret.handle_lost():
                    last_center = None
                    if fire_control.is_firing():
                        serial_out.stop_fire()
                        print("[fire_control] 표적 상실 -> 사격 중지")
                    fire_control.reset()

            if log_writer:
                row["servo_pan"] = turret.pan_angle
                row["servo_tilt"] = turret.tilt_angle
                log_writer.writerow({k: _fmt(v) for k, v in row.items()})

            if now - last_status >= 1.0:
                last_status = now
                if log_file:
                    log_file.flush()
                if target is not None:
                    print(f"[status] 표적 {target['conf']:.2f} "
                          f"err=({row['err_pan_deg']:+.1f},{row['err_tilt_deg']:+.1f})deg "
                          f"ω=({row.get('omega_pan', 0.0):+.1f},{row.get('omega_tilt', 0.0):+.1f})deg/s "
                          f"-> s2 {round(turret.pan_angle)} / s3 {round(turret.tilt_angle)} "
                          f"(보드 추론 {meta.get('infer_ms', 0.0):.1f}ms, {frames}프레임)")
                else:
                    print(f"[status] 표적 없음 (탐지 {len(dets)}개) "
                          f"s2 {round(turret.pan_angle)} / s3 {round(turret.tilt_angle)}")
    except KeyboardInterrupt:
        pass
    finally:
        serial_out.stop_fire()
        link.close()
        serial_out.close()
        if log_file:
            log_file.close()


if __name__ == "__main__":
    main()
