# -*- coding: utf-8 -*-
"""
실험계획 2단계 로그 분석: 거리별/크기별로 저장한 CSV 로그를 비교한다.

각 CSV는 track_target.py --log-csv 옵션으로 생성된 파일이다.

사용 예:
    python analyze_log.py --log far=logs/far.csv --log near=logs/near.csv
"""

import argparse
import csv
import math

import matplotlib.pyplot as plt


def load_log(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def summarize(label, rows):
    found = [r for r in rows if r["found"] == "1"]
    total = len(rows)
    found_ratio = len(found) / total if total else 0.0

    errors = [math.hypot(float(r["err_x_px"]), float(r["err_y_px"])) for r in found]
    mean_err = sum(errors) / len(errors) if errors else float("nan")

    print(f"[{label}] frames={total} found_ratio={found_ratio:.1%} mean_center_error_px={mean_err:.1f}")
    return errors


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--log", action="append", required=True,
        help="label=path.csv 형식으로 여러 번 지정 (예: --log near=logs/near.csv)",
    )
    args = parser.parse_args()

    plt.figure(figsize=(9, 5))
    for entry in args.log:
        label, path = entry.split("=", 1)
        rows = load_log(path)
        errors = summarize(label, rows)
        plt.plot(errors, label=label, alpha=0.8)

    plt.xlabel("탐지된 프레임 순번")
    plt.ylabel("중심 오차 (px)")
    plt.title("Stage 2: 거리/크기별 추적 오차 비교")
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
