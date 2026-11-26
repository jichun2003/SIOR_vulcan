# [PC] 보드 미리보기 스트림에서 JPEG 원본만 저장 (창 없음, 발열 적게)
# 보드: bash run.sh --preview-scale 1 --preview-quality 95 --preview-fps 2
# PC:   python3 grab_frames.py <보드IP> <장수> <저장폴더>
import socket, struct, json, sys, os
host, n, out = sys.argv[1], int(sys.argv[2]), sys.argv[3]
os.makedirs(out, exist_ok=True)
H = struct.Struct("<4sIIB3x")
def rx(s, k):
    b = bytearray()
    while len(b) < k:
        c = s.recv(k - len(b))
        if not c: raise ConnectionError("끊김 (보드 앱이 종료됨)")
        b += c
    return bytes(b)
s = socket.create_connection((host, 5001), timeout=10)
saved = 0
while saved < n:
    magic, ml, il, fmt = H.unpack(rx(s, H.size)); meta = json.loads(rx(s, ml)); img = rx(s, il)
    if fmt == 1:
        p = os.path.join(out, f"{saved:03d}.jpg"); open(p, "wb").write(img); saved += 1
        print(p, meta.get("img_w"), "x", meta.get("img_h"), " 보드 int8 conf:", [round(x["conf"], 2) for x in meta.get("dets", [])])
