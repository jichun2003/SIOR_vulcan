"""
ballistic_model_fit.py
------------------------------------------------------------
발사 영상 하나를 넣으면:
  1) 배경차분(모션 검출)으로 화면에서 "가장 빠르게 움직인 물체"를
     자동으로 찾아 프레임별 궤적(픽셀 좌표)을 뽑고
  2) 그 궤적을 공기저항(항력) 포함 탄도 운동방정식에 피팅해서
     초기속도(v0)와 항력계수(Cd)를 역산합니다.

사전 라벨링/학습된 모델이 전혀 필요 없습니다 (모션 기반 검출).

카메라는 총구 옆(발사 순간을 못 볼 수도 있는 위치)에서 찍은 영상이어도
됩니다 — "발사 후 t_offset초 뒤부터 관측을 시작했다"는 것까지 같이
역산하도록 설계되어 있어서, 절대 위치가 아니라 궤적의 "모양"만 맞으면
됩니다.

필요 패키지: opencv-python, numpy, scipy, matplotlib (vision/requirements.txt에 포함)

사용 예:
    python ballistic_model_fit.py --video gel_shot.mp4 \
        --mass-kg 0.00025 --diameter-m 0.0075 \
        --launch-angle-deg 20 --meters-per-pixel 0.005
------------------------------------------------------------
"""

import argparse
import json
import os

import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import odeint
from scipy.optimize import curve_fit

GRAVITY = 9.81         # 중력가속도 (m/s^2)
AIR_DENSITY = 1.225    # 공기밀도 (kg/m^3), 상온 기준


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--video", required=True, help="발사체가 찍힌 동영상 경로")

    p.add_argument("--mass-kg", type=float, required=True, help="발사체 질량 (kg). 예: 0.25g 젤볼이면 0.00025")
    p.add_argument("--diameter-m", type=float, required=True, help="발사체 지름 (m). 예: 7.5mm 젤볼이면 0.0075")
    p.add_argument("--launch-angle-deg", type=float, required=True, help="수평 기준 발사각(도). 위로 쏠수록 양수")
    p.add_argument(
        "--meters-per-pixel", type=float, required=True,
        help="화면 속 알려진 길이(기준물)로 구한 '픽셀당 실제 거리(m)'. "
             "예: 화면에서 1m 길이 자가 200픽셀로 찍혔다면 1/200=0.005",
    )

    p.add_argument("--fps-override", type=float, default=None, help="영상 메타데이터 FPS를 못 읽을 때 강제 지정")
    p.add_argument(
        "--ball-moves-left", action="store_true",
        help="발사체가 화면에서 오른쪽->왼쪽으로 날아가면 지정 (기본은 왼쪽->오른쪽)",
    )

    p.add_argument("--min-blob-area-px", type=float, default=15, help="이보다 작은 움직임 덩어리는 노이즈로 무시")
    p.add_argument("--max-match-dist-px", type=float, default=80, help="프레임 간 같은 물체로 인정할 최대 이동거리(픽셀)")
    p.add_argument("--min-track-len", type=int, default=5, help="이 프레임 수 이상 이어진 궤적만 후보로 인정")

    p.add_argument("--out-json", default="ballistic_model.json", help="결과 파라미터 저장 경로")
    p.add_argument("--out-plot", default="trajectory_fit.png", help="궤적 비교 그래프 저장 경로")

    return p.parse_args()


def extract_trajectory(video_path, fps_override, min_blob_area_px, max_match_dist_px, min_track_len):
    """영상에서 배경차분으로 움직이는 모든 후보를 추적하고,
    그중 평균 이동속도가 가장 빠른 트랙(=발사체)을 골라 반환한다.

    반환: (t_array, x_px_array, y_px_array, fps)
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"영상을 열 수 없습니다: {video_path}")

    fps = fps_override if fps_override else cap.get(cv2.CAP_PROP_FPS)
    if not fps or fps <= 0:
        fps = 30.0
        print("경고: FPS를 읽지 못해 기본값 30으로 가정합니다.")

    back_sub = cv2.createBackgroundSubtractorMOG2(
        history=200, varThreshold=30, detectShadows=False
    )

    # tracks: 각 트랙 = {"points": [(frame_idx, cx, cy), ...], "last_seen": frame_idx}
    tracks = []
    frame_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        fg_mask = back_sub.apply(frame)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_DILATE, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_blob_area_px:
                continue
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = M["m10"] / M["m00"]
            cy = M["m01"] / M["m00"]
            centroids.append((cx, cy))

        used = set()
        for track in tracks:
            if not centroids:
                break
            last_x, last_y = track["points"][-1][1], track["points"][-1][2]
            best_idx, best_dist = None, max_match_dist_px
            for i, (cx, cy) in enumerate(centroids):
                if i in used:
                    continue
                dist = np.hypot(cx - last_x, cy - last_y)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            if best_idx is not None:
                cx, cy = centroids[best_idx]
                track["points"].append((frame_idx, cx, cy))
                track["last_seen"] = frame_idx
                used.add(best_idx)

        for i, (cx, cy) in enumerate(centroids):
            if i not in used:
                tracks.append({"points": [(frame_idx, cx, cy)], "last_seen": frame_idx})

        frame_idx += 1

    cap.release()

    tracks = [t for t in tracks if len(t["points"]) >= min_track_len]
    if not tracks:
        raise RuntimeError(
            "움직이는 물체를 찾지 못했습니다. --min-blob-area-px / "
            "--max-match-dist-px 값을 조절해보세요."
        )

    def avg_speed(track):
        pts = track["points"]
        total_dist = 0.0
        for i in range(1, len(pts)):
            _, x0, y0 = pts[i - 1]
            _, x1, y1 = pts[i]
            total_dist += np.hypot(x1 - x0, y1 - y0)
        return total_dist / len(pts)

    fastest_track = max(tracks, key=avg_speed)

    pts = fastest_track["points"]
    frame_indices = np.array([p[0] for p in pts], dtype=float)
    x_px = np.array([p[1] for p in pts], dtype=float)
    y_px = np.array([p[2] for p in pts], dtype=float)

    t_array = (frame_indices - frame_indices[0]) / fps

    print(f"검출된 궤적: {len(pts)}개 지점, 총 {t_array[-1]:.3f}초 구간")
    return t_array, x_px, y_px, fps


def pixels_to_meters(t_array, x_px, y_px, meters_per_pixel, ball_moves_right):
    """픽셀 좌표(상대) -> 실제 물리 좌표(상대, m) 변환.
    첫 관측 지점을 (0,0)으로 두고 상대 변위만 계산한다.
    (절대 발사 위치를 몰라도 되는 이유가 이 부분)
    """
    dx_px = x_px - x_px[0]
    dy_px = y_px - y_px[0]

    x_rel_m = dx_px * meters_per_pixel
    if not ball_moves_right:
        x_rel_m = -x_rel_m

    # 이미지 y축은 아래로 갈수록 증가 -> 물리 좌표는 위로 갈수록 증가이므로 부호 반전
    y_rel_m = -dy_px * meters_per_pixel

    return t_array, x_rel_m, y_rel_m


def drag_dynamics(state, t, Cd, area, mass, rho, g):
    """공기저항(항력) + 중력을 포함한 운동방정식 (odeint용)."""
    x, y, vx, vy = state
    v = np.hypot(vx, vy)
    drag_coeff = 0.5 * rho * Cd * area  # F_drag = drag_coeff * v^2, 방향은 -v_hat
    if v > 1e-9:
        ax = -drag_coeff * v * vx / mass
        ay = -g - drag_coeff * v * vy / mass
    else:
        ax = 0.0
        ay = -g
    return [vx, vy, ax, ay]


def simulate_full_trajectory(v0, Cd, t_max, theta_rad, area, mass, rho, g, n_steps=2000):
    """t=0(발사 순간)부터 t_max까지 촘촘하게 적분해서 (t, x, y) 배열 반환."""
    t_sim = np.linspace(0, t_max, n_steps)
    state0 = [0.0, 0.0, v0 * np.cos(theta_rad), v0 * np.sin(theta_rad)]
    sol = odeint(drag_dynamics, state0, t_sim, args=(Cd, area, mass, rho, g))
    return t_sim, sol[:, 0], sol[:, 1]


def make_model_function(theta_rad, area, mass, rho, g):
    """curve_fit에 넘길 모델 함수를 만들어 반환.
    미지수: v0, Cd, t_offset(발사 후 첫 관측까지 걸린 시간)
    """

    def model(t_rel_array, v0, Cd, t_offset):
        t_max = t_offset + float(np.max(t_rel_array)) + 0.05
        t_max = max(t_max, 0.05)
        t_sim, x_sim, y_sim = simulate_full_trajectory(
            v0, Cd, t_max, theta_rad, area, mass, rho, g
        )

        query_t = t_offset + t_rel_array
        x_at = np.interp(query_t, t_sim, x_sim)
        y_at = np.interp(query_t, t_sim, y_sim)

        x0 = np.interp(t_offset, t_sim, x_sim)
        y0 = np.interp(t_offset, t_sim, y_sim)

        return np.concatenate([x_at - x0, y_at - y0])

    return model


def fit_ballistic_model(t_rel, x_rel_m, y_rel_m, theta_deg, mass_kg, diameter_m):
    theta_rad = np.radians(theta_deg)
    area = np.pi * (diameter_m / 2) ** 2

    model_fn = make_model_function(theta_rad, area, mass_kg, AIR_DENSITY, GRAVITY)
    observed = np.concatenate([x_rel_m, y_rel_m])

    if len(t_rel) > 1 and t_rel[1] > t_rel[0]:
        v0_guess = np.hypot(x_rel_m[1] - x_rel_m[0], y_rel_m[1] - y_rel_m[0]) / (t_rel[1] - t_rel[0])
        v0_guess = max(v0_guess, 1.0)
    else:
        v0_guess = 10.0

    p0 = [v0_guess, 0.47, 0.01]
    bounds = ([0.1, 0.05, 0.0], [200.0, 2.0, 5.0])

    popt, pcov = curve_fit(model_fn, t_rel, observed, p0=p0, bounds=bounds, maxfev=20000)
    v0_fit, Cd_fit, t_offset_fit = popt
    perr = np.sqrt(np.diag(pcov))

    return {
        "v0": v0_fit,
        "Cd": Cd_fit,
        "t_offset": t_offset_fit,
        "v0_stderr": perr[0],
        "Cd_stderr": perr[1],
        "t_offset_stderr": perr[2],
        "theta_deg": theta_deg,
        "mass_kg": mass_kg,
        "diameter_m": diameter_m,
        "area_m2": area,
    }, model_fn


def plot_fit(t_rel, x_rel_m, y_rel_m, model_fn, params, save_path):
    predicted = model_fn(t_rel, params["v0"], params["Cd"], params["t_offset"])
    n = len(t_rel)
    x_pred, y_pred = predicted[:n], predicted[n:]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    axes[0].plot(x_rel_m, y_rel_m, "o", label="관측된 궤적", color="tab:blue")
    axes[0].plot(x_pred, y_pred, "-", label="피팅된 모델", color="tab:red")
    axes[0].set_xlabel("수평 변위 (m)")
    axes[0].set_ylabel("수직 변위 (m)")
    axes[0].set_title("궤적 (상대 위치)")
    axes[0].legend()
    axes[0].axis("equal")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_rel, x_rel_m, "o", label="관측 x", color="tab:blue")
    axes[1].plot(t_rel, x_pred, "-", label="모델 x", color="tab:blue", alpha=0.5)
    axes[1].plot(t_rel, y_rel_m, "s", label="관측 y", color="tab:orange")
    axes[1].plot(t_rel, y_pred, "-", label="모델 y", color="tab:orange", alpha=0.5)
    axes[1].set_xlabel("시간 (s, 첫 관측 프레임 기준)")
    axes[1].set_ylabel("변위 (m)")
    axes[1].set_title("시간에 따른 변위")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    out_dir = os.path.dirname(save_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    plt.savefig(save_path, dpi=150)
    print(f"궤적 비교 그래프 저장: {save_path}")


def main():
    args = parse_args()

    print("1) 영상에서 궤적 추출 중...")
    t_array, x_px, y_px, fps = extract_trajectory(
        args.video, args.fps_override, args.min_blob_area_px,
        args.max_match_dist_px, args.min_track_len,
    )

    print("2) 픽셀 -> 실제 좌표(m) 변환 중...")
    t_rel, x_rel_m, y_rel_m = pixels_to_meters(
        t_array, x_px, y_px, args.meters_per_pixel, not args.ball_moves_left
    )

    print("3) 탄도모델(v0, Cd) 피팅 중...")
    params, model_fn = fit_ballistic_model(
        t_rel, x_rel_m, y_rel_m, args.launch_angle_deg, args.mass_kg, args.diameter_m
    )

    print("\n===== 결과 =====")
    print(f"초기속도 v0      = {params['v0']:.3f} ± {params['v0_stderr']:.3f} m/s")
    print(f"항력계수 Cd      = {params['Cd']:.3f} ± {params['Cd_stderr']:.3f}")
    print(f"발사~첫관측 시간  = {params['t_offset']:.3f} ± {params['t_offset_stderr']:.3f} s")
    print(f"(참고) 발사각     = {params['theta_deg']:.1f} deg")
    print(f"(참고) 질량       = {params['mass_kg']*1000:.3f} g")
    print(f"(참고) 지름       = {params['diameter_m']*1000:.3f} mm")

    print("\n4) 결과 저장 및 그래프 생성...")
    out_json_dir = os.path.dirname(args.out_json)
    if out_json_dir:
        os.makedirs(out_json_dir, exist_ok=True)
    with open(args.out_json, "w", encoding="utf-8") as f:
        json.dump(params, f, ensure_ascii=False, indent=2)
    print(f"탄도모델 파라미터 저장: {args.out_json}")

    plot_fit(t_rel, x_rel_m, y_rel_m, model_fn, params, args.out_plot)

    print("\n이 v0, Cd 값은 발사각과 무관한 물리 상수이므로,")
    print("다른 각도로 쏠 때도 그대로 재사용해서 탄도(비행시간, 도달거리)를 계산할 수 있습니다.")


if __name__ == "__main__":
    main()
