# -*- coding: utf-8 -*-
"""
main.py --log-csv 로그 분석.

1) 추적 성능 비교 (기본): 화면 중심 ~ 표적 중심 각도 오차의 평균/RMS/95%/최대, 탐지율
     python analyze_log.py logs\\2dof.csv logs\\legacy.csv

2) 지연 L 실측 (--latency): main.py --latency-sweep 로그 사용 (정지 표적, pan 사인파)
     python analyze_log.py logs\\latency.csv --latency
   원리: 표적이 정지해 있으면 "L초 전 서보 명령각 + 화면 오차"가 항상 같아야 한다.
         L을 0~0.6초로 바꿔가며 이 값이 가장 덜 흔들리는 L을 찾는다.
         같이 구한 기울기 k는 화각 보정 계수 (k≈1이면 --hfov 값이 맞음).

--plot을 붙이면 그래프 PNG도 저장 (matplotlib 필요).
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

from config import Config


def load(path):
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows or "t_mono" not in rows[0]:
        sys.exit(f"{path}: 새 형식 로그가 아닙니다 (t_mono 컬럼 없음). 최신 main.py로 다시 기록하세요")

    def col(name):
        return np.array([float(r[name]) if r.get(name, "") != "" else np.nan for r in rows])

    return {k: col(k) for k in ("t_mono", "found", "err_pan_deg", "err_tilt_deg",
                                "servo_pan", "servo_tilt", "omega_pan", "omega_tilt")}, rows


def _korean_font():
    """그래프 한글 깨짐 방지 (Windows: 맑은 고딕)."""
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    names = {f.name for f in font_manager.fontManager.ttflist}
    for name in ("Malgun Gothic", "AppleGothic", "NanumGothic"):
        if name in names:
            plt.rcParams["font.family"] = name
            break
    plt.rcParams["axes.unicode_minus"] = False


def tracking_stats(paths, plot):
    print(f"{'파일':<28}{'프레임':>7}{'탐지율':>8} | {'pan 평균':>8}{'RMS':>7}{'95%':>7}{'최대':>7}"
          f" | {'tilt 평균':>9}{'RMS':>7}{'95%':>7}{'최대':>7}")
    for p in paths:
        d, rows = load(p)
        found = d["found"] == 1
        mode = rows[0].get("mode", "")
        out = f"{Path(p).name[:20] + ' (' + mode + ')':<28}{len(rows):>7}{found.mean() * 100:>7.1f}% |"
        for axis in ("err_pan_deg", "err_tilt_deg"):
            e = np.abs(d[axis][found])
            if len(e) == 0:
                out += f"{'-':>8}{'-':>7}{'-':>7}{'-':>7} |"
                continue
            out += (f"{e.mean():>8.2f}{np.sqrt((e ** 2).mean()):>7.2f}"
                    f"{np.percentile(e, 95):>7.2f}{e.max():>7.2f} |")
        print(out.rstrip(" |"))
    print("(단위: deg, 탐지된 프레임 기준. 작을수록 표적이 화면 중심에 잘 붙어 있음)")

    if plot:
        import matplotlib.pyplot as plt
        _korean_font()
        fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        for p in paths:
            d, _ = load(p)
            t = d["t_mono"] - d["t_mono"][0]
            for ax, axis in zip(axes, ("err_pan_deg", "err_tilt_deg")):
                ax.plot(t, d[axis], label=Path(p).stem, lw=1)
        for ax, name in zip(axes, ("pan 오차 (deg)", "tilt 오차 (deg)")):
            ax.set_ylabel(name)
            ax.axhline(0, color="gray", lw=0.5)
            ax.legend()
        axes[-1].set_xlabel("시간 (s)")
        out = Path(paths[0]).with_suffix(".tracking.png")
        fig.tight_layout()
        fig.savefig(out, dpi=120)
        print(f"그래프 저장: {out}")


def estimate_latency(path, sign, plot, axis="pan", l_max=0.6, step=0.005):
    d, rows = load(path)
    t, servo, err = d["t_mono"], d[f"servo_{axis}"], d[f"err_{axis}_deg"]
    ok = (d["found"] == 1) & ~np.isnan(err)
    if ok.sum() < 30:
        sys.exit(f"탐지된 프레임이 {ok.sum()}개뿐입니다. 정지 표적이 화면에 계속 보이게 하고 다시 기록하세요")
    if np.nanstd(servo) < 2.0:
        print(f"경고: {axis} 서보가 거의 움직이지 않았습니다. "
              f"--latency-sweep --sweep-axis {axis}로 기록한 로그인지 확인하세요")

    x = sign * err[ok]  # 화면 오차 (서보각 공간)
    tq = t[ok]
    Ls = np.arange(0.0, l_max + 1e-9, step)
    stds, ks = [], []
    for L in Ls:
        s = np.interp(tq - L, t, servo)  # L초 전 서보 명령각
        # 정지 표적: s + k*x = 상수  ->  s를 x에 대해 선형회귀, 잔차 표준편차 최소인 L을 찾음
        A = np.vstack([np.ones_like(x), -x]).T
        coef, *_ = np.linalg.lstsq(A, s, rcond=None)
        stds.append(np.std(s - A @ coef))
        ks.append(coef[1])
    stds, ks = np.array(stds), np.array(ks)
    i = int(np.argmin(stds))
    L_best, k_best = Ls[i], ks[i]

    fov_name = "hfov" if axis == "pan" else "vfov"
    logged = rows[0].get(fov_name, "")
    if logged != "":
        fov_now, fov_src = float(logged), "로그에 기록된 값"
    else:
        fov_now = Config().hfov_deg if axis == "pan" else Config().vfov_deg
        fov_src = "지금 config.py 값 (예전 로그라 기록 당시 화각이 없음. 다르면 직접 계산하세요)"
    print(f"[{axis}] 탐지 프레임 {ok.sum()}개, 서보 진폭 약 ±{(np.nanmax(servo) - np.nanmin(servo)) / 2:.1f}deg")
    print(f"  L=0 일 때 베어링 흔들림  : {stds[0]:.2f} deg (시간 정렬 안 했을 때)")
    print(f"  최적 L = {L_best:.3f} s  -> 흔들림 {stds[i]:.2f} deg")
    print(f"  화각 보정 계수 k = {k_best:.3f}  (실효 {fov_name} ≈ 기록 당시 {fov_name} × {k_best:.3f})")
    if i == len(Ls) - 1:
        print("  경고: 최적 L이 탐색 범위 끝입니다. 지연이 더 크거나 로그가 이상할 수 있습니다")
    if not 0.5 < k_best < 1.5:
        print(f"  경고: k가 1에서 많이 벗어남 -> {axis}_sign이 틀렸거나 표적이 움직였을 수 있습니다")
    lat_opt = "--latency" if axis == "pan" else "--latency-tilt"
    print(f"\n적용 (config.py 또는 실행 옵션): {lat_opt} {L_best:.3f}"
          + (f" --{fov_name} {fov_now * k_best:.1f}" if abs(k_best - 1) > 0.05 else ""))
    print(f"  ※ --{fov_name} = 기록 당시 {fov_name} {fov_now} × k  (기준: {fov_src})")

    if plot:
        import matplotlib.pyplot as plt
        _korean_font()
        fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6))
        a1.plot(Ls, stds)
        a1.axvline(L_best, color="r", ls="--")
        a1.set_xlabel("L (s)")
        a1.set_ylabel("베어링 흔들림 (deg)")
        s_best = np.interp(tq - L_best, t, servo)
        a2.plot(tq - tq[0], np.interp(tq, t, servo) + k_best * x, label="L=0", lw=1)
        a2.plot(tq - tq[0], s_best + k_best * x, label=f"L={L_best:.3f}", lw=1)
        a2.set_xlabel("시간 (s)")
        a2.set_ylabel("정지 표적 베어링 (deg)")
        a2.legend()
        out = Path(path).with_suffix(f".latency_{axis}.png")
        fig.tight_layout()
        fig.savefig(out, dpi=120)
        print(f"그래프 저장: {out}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("logs", nargs="+")
    p.add_argument("--latency", action="store_true", help="지연 L 추정 (--latency-sweep 로그)")
    p.add_argument("--axis", choices=["pan", "tilt"], default=None,
                   help="--latency에서 분석할 축 (기본: 로그에서 더 많이 움직인 축)")
    p.add_argument("--pan-sign", type=float, default=Config().pan_sign)
    p.add_argument("--tilt-sign", type=float, default=Config().tilt_sign)
    p.add_argument("--plot", action="store_true")
    a = p.parse_args()
    if a.latency:
        for path in a.logs:
            print(f"== {path}")
            axis = a.axis
            if axis is None:
                d, _ = load(path)
                axis = "tilt" if np.nanstd(d["servo_tilt"]) > np.nanstd(d["servo_pan"]) else "pan"
            sign = a.pan_sign if axis == "pan" else a.tilt_sign
            estimate_latency(path, sign, a.plot, axis=axis)
    else:
        tracking_stats(a.logs, a.plot)


if __name__ == "__main__":
    main()
