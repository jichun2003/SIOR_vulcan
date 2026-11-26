# [PC, 도커] 보드에서 받은 CSI 프레임으로 ENLIGHT fp / int8 시뮬레이션 비교 (보드와 같은 960x960 stretch)
# 사용: tc-nn-toolkit 폴더에 이 파일과 eval_csi/ 폴더를 두고
#   [도커] cd /sdk && /sdk/venv_onnx/bin/python eval_csi_int8.py
import sys, glob, torch
sys.path.append("./EnlightSDK")
from custom_evaluation_enlight_object_detection import prepare_enlight_model, get_input_transforms
from PIL import Image

paths = sorted(glob.glob("eval_csi/*.jpg"))
tf = get_input_transforms((0, 0, 0), (1, 1, 1))


def run(model_path, resample):
    net = prepare_enlight_model(model_path)
    net.set_post_process_threshold(0.05, 0.5)
    h, w = net.get_input_size()[2:]
    best = []
    with torch.no_grad():
        for p in paths:
            x = tf(Image.open(p).convert("RGB").resize((w, h), resample)).unsqueeze(0)
            if net.is_quantized():
                x = torch.clamp(torch.round(x * 128.0), -128, 127)
            out = net(x)
            confs = [o[4] for b in out for cls in b if cls for o in cls]
            best.append(float(max(confs, default=0.0)))
    return best


fp = run("output_networks/vulcan.enlight", Image.BICUBIC)
q_bc = run("output_networks/vulcan_s16_quantized.enlight", Image.BICUBIC)
q_nn = run("output_networks/vulcan_s16_quantized.enlight", Image.NEAREST)

print(f"images: {len(paths)}  (모두 960x960 stretch)")
print(f"{'file':10s} {'fp':>6s} {'int8':>6s} {'int8 nearest':>13s}")
for p, a, b, c in zip(paths, fp, q_bc, q_nn):
    print(f"{p.split('/')[-1]:10s} {a:6.2f} {b:6.2f} {c:13.2f}")
avg = lambda v: sum(v) / max(len(v), 1)
print(f"{'평균':10s} {avg(fp):6.2f} {avg(q_bc):6.2f} {avg(q_nn):13.2f}")
