# -*- coding: utf-8 -*-
r"""
추적 로그 CSV에서 서보 명령각이 갑자기 튄 순간을 찾는다 (aaa_code_1111 / AI-G 로그 공통).

사용: python find_spikes.py logs\lead_test.csv            (기본: 한 프레임에 5도 넘게 변하면 표시)
      python find_spikes.py logs\lead_test.csv 3          (3도 기준)

출력:
  1) 로그에 있는 열 이름
  2) 튄 프레임마다 앞뒤 3프레임의 주요 값 (found, conf, 오차, 서보각, lead/ff 관련 열)
  3) 요약: 튄 횟수, 그중 표적이 '없던' 프레임 수, 탄도/선도 관련 열이 같이 튄 횟수
"""
import csv
import sys

path = sys.argv[1]
thr = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

with open(path, newline="", encoding="utf-8-sig") as f:
    rows = list(csv.DictReader(f))
if not rows:
    sys.exit("로그가 비어 있습니다")
cols = list(rows[0].keys())
print("열:", ", ".join(cols))


def find(*keys):
    """열 이름에 keys가 모두 들어간 첫 열"""
    for c in cols:
        lc = c.lower()
        if all(k in lc for k in keys):
            return c
    return None


c_pan = find("servo", "pan") or find("pan")
c_tilt = find("servo", "tilt") or find("tilt")
c_found = find("found")
c_conf = find("conf")
c_ex = find("err", "pan") or find("err", "x")
c_ey = find("err", "tilt") or find("err", "y")
c_t = cols[0]
extra = [c for c in cols if any(k in c.lower() for k in ("lead", "ff", "omega", "vel", "ballistic", "elev", "range"))]
show = [c for c in [c_t, c_found, c_conf, c_ex, c_ey, c_pan, c_tilt] if c] + [c for c in extra if c not in (c_pan, c_tilt)]
print("서보 열:", c_pan, "/", c_tilt)
print("함께 보는 열:", ", ".join(show))
print()


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


spikes = []
for i in range(1, len(rows)):
    for c in (c_pan, c_tilt):
        if c is None:
            continue
        a, b = num(rows[i - 1][c]), num(rows[i][c])
        if a is not None and b is not None and abs(b - a) > thr:
            spikes.append((i, c, b - a))

no_target = 0
for i, c, d in spikes:
    print(f"=== 프레임 {i}: {c} {d:+.1f}도 ===")
    for j in range(max(0, i - 3), min(len(rows), i + 4)):
        mark = ">>" if j == i else "  "
        print(mark, "  ".join(f"{k}={rows[j][k]}" for k in show))
    if c_found and str(rows[i][c_found]).strip() in ("0", "False", "false", ""):
        no_target += 1
    print()

print(f"요약: {thr}도 넘게 튄 프레임 {len(spikes)}개 (전체 {len(rows)}프레임)")
if c_found:
    print(f"  그중 표적이 없던 프레임: {no_target}개  (많으면 놓침/예측 구간에서 튐)")
if not spikes:
    print("  명령각은 부드러움 -> 모터가 튀었다면 서보/전원/배선 쪽 문제")
