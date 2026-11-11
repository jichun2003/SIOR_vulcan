# -*- coding: utf-8 -*-
"""
학습된 yolov8s 가중치(.pt)를 TOPST AI-G NPU 툴킷(EnlightSDK)에 넣을 수 있는
ONNX로 내보낸다.

왜 "그냥 ultralytics export"가 아닌가
------------------------------------
ultralytics가 기본으로 내보내는 ONNX는 Detect 헤드의 DFL 디코드(Softmax/Sub/Add/
Div/Concat)와 좌표 변환까지 그래프 안에 들어있다. AI-G NPU가 하드웨어로 지원하는
연산은 conv / depthwise conv / convtranspose / pool / 활성함수 / elementwise
add·mul / BN / concat / slice / split 정도이고, 지원하지 않는 연산은 "마지막
레이어일 때만" CPU로 떨어뜨릴 수 있다 (NPU toolkit 문서 5.1절).

그래서 Telechips 모델 주에 들어있는 yolo 계열이 `*_extracted.onnx`인 것과 같은
방식으로, **헤드의 conv 출력까지만** 그래프에 남기고 DFL 디코드 + sigmoid + NMS는
보드 쪽 CPU 후처리(post_process.c)로 넘긴다.

내보내는 출력 (640x640 입력 기준, 전부 conv 출력 = NPU 지원 연산):
    box_s8   (1, 64, 80, 80)    cls_s8   (1, nc, 80, 80)
    box_s16  (1, 64, 40, 40)    cls_s16  (1, nc, 40, 40)
    box_s32  (1, 64, 20, 20)    cls_s32  (1, nc, 20, 20)
  box 채널 64 = 4변(l,t,r,b) x reg_max(16)  -> DFL 기댓값으로 디코드
  cls 채널 nc = 클래스 수 (이 프로젝트는 1: target), sigmoid 전의 raw logit

--verify를 주면 (onnxruntime 필요):
  1) torch 원본 모델과 ONNX의 raw 출력이 같은지
  2) 이 raw 출력을 numpy로 DFL 디코드 + NMS 한 결과가 ultralytics 정식 출력과 같은지
  를 실제 이미지로 비교한다. 2)의 numpy 디코드가 보드 post_process.c의 기준 구현이다.

사용 예:
    python tools/export_onnx_npu.py --weights ../checkpoint/best1105_live.pt \
        --imgsz 640 --out yolov8s_target_extracted.onnx \
        --verify --verify-image ../dataset/target/img_0000.jpg

필요 패키지: torch, ultralytics, onnx (+ --verify 시 onnxruntime)
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import torch

# ---- Detect 헤드를 "conv 출력까지만" 내보내도록 바꾼 forward ----------------


def _detect_forward_raw(self, x):
    """ultralytics Detect.forward 대체. DFL 디코드 전 conv 출력을 그대로 반환.

    스케일마다 (box conv, cls conv) 두 개씩, 작은 stride(8) -> 큰 stride(32) 순서.
    """
    outs = []
    for i in range(self.nl):
        outs.append(self.cv2[i](x[i]))  # box: (1, 4*reg_max, h, w)
        outs.append(self.cv3[i](x[i]))  # cls: (1, nc, h, w)
    return tuple(outs)


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--weights", default="../checkpoint/best1105_live.pt")
    p.add_argument(
        "--imgsz", type=int, default=640,
        help="NPU 입력 해상도. Telechips 모델 주의 yolov8s가 640x640이므로 기본 640 (32의 배수여야 함)",
    )
    p.add_argument("--out", default=None, help="출력 onnx 경로 (기본: <weights 이름>_extracted.onnx)")
    p.add_argument(
        "--opset", type=int, default=12,
        help="ONNX opset. EnlightSDK 호스트가 torch 1.12 기준이라 12를 기본으로 둔다",
    )
    p.add_argument(
        "--simplify", action="store_true",
        help="onnx-simplifier를 추가로 적용 (선택). C2f를 forward_split으로 내보내면 보통 필요 없고, "
             "onnxsim 0.4.36 + onnx 1.19 조합은 macOS에서 segfault가 나므로 기본은 끔",
    )
    p.add_argument("--verify", action="store_true", help="onnxruntime으로 출력/디코드 일치 검증")
    p.add_argument("--verify-image", default=None, help="검증에 쓸 이미지 (없으면 랜덤 텐서로 1)번만 검증)")
    p.add_argument("--conf", type=float, default=0.4, help="검증 시 신뢰도 임계값")
    p.add_argument("--iou", type=float, default=0.5, help="검증 시 NMS IoU 임계값")
    return p.parse_args()


# ---- numpy 기준 후처리 (보드 post_process.c의 참조 구현) -------------------


def sigmoid(x):
    """오버플로 없는 sigmoid (logit이 -100 같은 값이어도 경고가 안 나게). post_process.c도 동일하게."""
    out = np.empty_like(x)
    pos = x >= 0
    out[pos] = 1.0 / (1.0 + np.exp(-x[pos]))
    e = np.exp(x[~pos])
    out[~pos] = e / (1.0 + e)
    return out


def softmax(x, axis):
    x = x - x.max(axis=axis, keepdims=True)
    e = np.exp(x)
    return e / e.sum(axis=axis, keepdims=True)


def decode_raw_outputs(raw, strides=(8, 16, 32), reg_max=16, conf_th=0.4):
    """box/cls conv 출력 -> xyxy 박스 + 점수.

    raw: [box_s8, cls_s8, box_s16, cls_s16, box_s32, cls_s32] (numpy, NCHW)
    반환: boxes (N,4) xyxy (입력 이미지 픽셀 좌표), scores (N,), classes (N,)
    """
    boxes_all, scores_all, cls_all = [], [], []
    bins = np.arange(reg_max, dtype=np.float32)

    for i, stride in enumerate(strides):
        box = raw[2 * i]      # (1, 4*reg_max, h, w)
        cls = raw[2 * i + 1]  # (1, nc, h, w)
        _, _, h, w = box.shape
        nc = cls.shape[1]

        # 1) 클래스 점수: raw logit -> sigmoid, 임계값 넘는 셀만 남겨 연산을 줄인다
        prob = sigmoid(cls[0])                           # (nc, h, w)
        best_cls = prob.argmax(axis=0)                   # (h, w)
        best_score = prob.max(axis=0)                    # (h, w)
        keep = best_score >= conf_th
        if not keep.any():
            continue
        ys, xs = np.nonzero(keep)

        # 2) DFL: 4변 각각 reg_max개 bin에 대한 softmax 기댓값 = 중심에서의 거리(grid 단위)
        d = box[0].reshape(4, reg_max, h, w)[:, :, ys, xs]   # (4, reg_max, N)
        d = (softmax(d, axis=1) * bins[None, :, None]).sum(axis=1)  # (4, N) = l, t, r, b

        # 3) 그리드 중심(+0.5) 기준으로 xyxy 복원 후 stride를 곱해 입력 픽셀 좌표로
        cx = (xs.astype(np.float32) + 0.5) * stride
        cy = (ys.astype(np.float32) + 0.5) * stride
        x1 = cx - d[0] * stride
        y1 = cy - d[1] * stride
        x2 = cx + d[2] * stride
        y2 = cy + d[3] * stride

        boxes_all.append(np.stack([x1, y1, x2, y2], axis=1))
        scores_all.append(best_score[ys, xs])
        cls_all.append(best_cls[ys, xs])

    if not boxes_all:
        return np.zeros((0, 4), np.float32), np.zeros((0,), np.float32), np.zeros((0,), np.int32)
    return (np.concatenate(boxes_all).astype(np.float32),
            np.concatenate(scores_all).astype(np.float32),
            np.concatenate(cls_all).astype(np.int32))


def nms(boxes, scores, iou_th):
    """클래스 구분 없는 단순 NMS (단일 클래스 표적용). 남길 인덱스를 점수 내림차순으로 반환."""
    order = scores.argsort()[::-1]
    area = (boxes[:, 2] - boxes[:, 0]).clip(0) * (boxes[:, 3] - boxes[:, 1]).clip(0)
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = (xx2 - xx1).clip(0) * (yy2 - yy1).clip(0)
        iou = inter / (area[i] + area[rest] - inter + 1e-9)
        order = rest[iou <= iou_th]
    return np.array(keep, dtype=np.int32)


# ---- 내보내기 --------------------------------------------------------------


def load_model(weights):
    from ultralytics import YOLO
    from ultralytics.nn.modules.block import C2f

    yolo = YOLO(weights)
    model = yolo.model.float().eval()
    for p in model.parameters():
        p.requires_grad_(False)

    # C2f의 기본 forward는 chunk()를 써서 ONNX에 Shape/Gather/Div(채널 수로 분할점 계산)가
    # 생긴다. 이 셋은 NPU 지원 목록에 없다. ultralytics 공식 export와 똑같이 forward_split
    # (정적 split)으로 바꾸면 Split 하나로 끝나고 Split은 NPU가 지원한다.
    for m in model.modules():
        if isinstance(m, C2f) and hasattr(m, "forward_split"):
            m.forward = m.forward_split
    return yolo, model


def export(args):
    import onnx
    from ultralytics.nn.modules.head import Detect

    weights = Path(args.weights)
    out_path = Path(args.out) if args.out else weights.with_name(weights.stem + "_extracted.onnx")

    yolo, model = load_model(weights)
    detect = model.model[-1]
    if not isinstance(detect, Detect):
        raise RuntimeError(f"마지막 레이어가 Detect가 아닙니다: {type(detect).__name__}")
    nc, reg_max, nl = detect.nc, detect.reg_max, detect.nl
    print(f"[export] 모델: {weights.name}  클래스 {nc}개 {model.names}  reg_max={reg_max}  스케일 {nl}개")
    if args.imgsz % 32 != 0:
        raise SystemExit(f"--imgsz는 32의 배수여야 합니다: {args.imgsz}")

    # Detect.forward를 raw 버전으로 교체 (원본은 검증에서 되돌려 쓴다)
    orig_forward = Detect.forward
    Detect.forward = _detect_forward_raw

    dummy = torch.zeros(1, 3, args.imgsz, args.imgsz)
    with torch.no_grad():
        raw = model(dummy)
    s = args.imgsz
    names_out = []
    for i, st in enumerate((8, 16, 32)[:nl]):
        names_out += [f"box_s{st}", f"cls_s{st}"]
    for name, t in zip(names_out, raw):
        print(f"[export]   출력 {name:8s} {tuple(t.shape)}")

    torch.onnx.export(
        model, dummy, str(out_path),
        input_names=["images"], output_names=names_out,
        opset_version=args.opset, do_constant_folding=True, dynamic_axes=None,
    )
    Detect.forward = orig_forward

    m = onnx.load(str(out_path))
    onnx.checker.check_model(m)

    if args.simplify:
        before = sorted({n.op_type for n in m.graph.node})
        try:
            import onnxsim

            m_s, ok = onnxsim.simplify(m)
            if not ok:
                print("[export] onnxsim 검증 실패 -> 원본 그래프를 그대로 둡니다")
            else:
                m = m_s
                onnx.save(m, str(out_path))
                after = sorted({n.op_type for n in m.graph.node})
                gone = [o for o in before if o not in after]
                print(f"[export] onnxsim 적용: 노드 종류 {len(before)} -> {len(after)}"
                      + (f", 사라진 연산 {', '.join(gone)}" if gone else ""))
        except ImportError:
            print("[export] onnxsim이 없어 건너뜀 (pip install onnxsim 권장)")

    ops = sorted({n.op_type for n in m.graph.node})
    print(f"[export] 저장: {out_path}  ({out_path.stat().st_size / 1e6:.1f} MB), opset {args.opset}")
    print(f"[export] 그래프 연산 종류: {', '.join(ops)}")

    # NPU가 하드웨어로 지원하는 연산 목록 (NPU toolkit 문서 표 5.1) 기준 사전 점검
    npu_ok = {
        "Conv", "ConvTranspose", "Relu", "Clip", "LeakyRelu", "Sigmoid", "Tanh",
        "HardSwish", "HardSigmoid", "Mul", "Add", "MaxPool", "AveragePool",
        "GlobalAveragePool", "BatchNormalization", "Concat", "Slice", "Split",
        "Resize", "Pad", "MatMul", "ReduceMean", "ReduceSum", "Transpose", "Reshape",
        "Constant",  # 연산이 아니라 상수 공급 노드 (Sigmoid+Mul 쌍 = SiLU/Swish)
    }
    unknown = [o for o in ops if o not in npu_ok]
    if unknown:
        print(f"[export] !! 표 5.1에 없는 연산: {', '.join(unknown)} "
              f"-> converter가 거부하면 해당 연산을 헤드 밖으로 빼야 함")
    else:
        print("[export] 그래프의 모든 연산이 NPU 지원 목록 안에 있음")
    return out_path, nc, reg_max, nl


# ---- 검증 ------------------------------------------------------------------


def make_input(image_path, imgsz):
    """ultralytics와 동일한 letterbox 전처리로 (1,3,imgsz,imgsz) 입력을 만든다."""
    import cv2
    from ultralytics.data.augment import LetterBox

    img = cv2.imread(str(image_path))
    if img is None:
        raise SystemExit(f"이미지를 읽을 수 없습니다: {image_path}")
    lb = LetterBox((imgsz, imgsz), auto=False, scale_fill=False)
    padded = lb(image=img)
    x = padded[:, :, ::-1].transpose(2, 0, 1)  # BGR->RGB, HWC->CHW
    x = np.ascontiguousarray(x, dtype=np.float32) / 255.0
    return x[None], img, padded


def verify(args, onnx_path, nc, reg_max, nl):
    import onnxruntime as ort
    from ultralytics.nn.modules.head import Detect

    try:  # ultralytics 8.4에서 utils.ops -> utils.nms로 이동
        from ultralytics.utils.nms import non_max_suppression
    except ImportError:
        from ultralytics.utils.ops import non_max_suppression

    strides = (8, 16, 32)[:nl]
    yolo, model = load_model(args.weights)

    if args.verify_image:
        inp, _, _ = make_input(args.verify_image, args.imgsz)
        src = Path(args.verify_image).name
    else:
        rng = np.random.default_rng(0)
        inp = rng.random((1, 3, args.imgsz, args.imgsz), dtype=np.float32)
        src = "랜덤 텐서"
    t = torch.from_numpy(inp)

    # (1) torch raw vs onnx raw
    orig_forward = Detect.forward
    Detect.forward = _detect_forward_raw
    with torch.no_grad():
        torch_raw = [o.numpy() for o in model(t)]
    Detect.forward = orig_forward

    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    onnx_raw = sess.run(None, {"images": inp})

    print(f"\n[verify] 입력: {src}  {inp.shape}")
    worst = 0.0
    for name, a, b in zip([o.name for o in sess.get_outputs()], torch_raw, onnx_raw):
        d = float(np.abs(a - b).max())
        worst = max(worst, d)
        print(f"[verify] (1) raw 출력 {name:8s} 최대 오차 {d:.2e}")
    print(f"[verify] (1) {'OK' if worst < 1e-3 else '실패'}: torch vs onnx 최대 오차 {worst:.2e}")

    # (2) numpy 디코드 vs ultralytics 정식 후처리
    boxes, scores, classes = decode_raw_outputs(
        onnx_raw, strides=strides, reg_max=reg_max, conf_th=args.conf)
    if len(boxes):
        k = nms(boxes, scores, args.iou)
        boxes, scores, classes = boxes[k], scores[k], classes[k]

    with torch.no_grad():
        ref = model(t)
    ref = ref[0] if isinstance(ref, (list, tuple)) else ref
    ref_nms = non_max_suppression(ref, conf_thres=args.conf, iou_thres=args.iou, nc=nc)[0].numpy()

    print(f"[verify] (2) numpy 디코드 {len(boxes)}개 / ultralytics {len(ref_nms)}개 검출")
    if len(boxes) == len(ref_nms) and len(boxes):
        order_a = np.argsort(-scores)
        order_b = np.argsort(-ref_nms[:, 4])
        db = float(np.abs(boxes[order_a] - ref_nms[order_b, :4]).max())
        ds = float(np.abs(scores[order_a] - ref_nms[order_b, 4]).max())
        print(f"[verify] (2) 박스 좌표 최대 오차 {db:.3f} px, 점수 최대 오차 {ds:.2e}")
        print(f"[verify] (2) {'OK' if db < 1.0 and ds < 1e-3 else '실패'}: 디코드 로직 일치")
        for i, (b, s) in enumerate(zip(boxes[order_a], scores[order_a])):
            print(f"[verify]     #{i} xyxy=({b[0]:.1f}, {b[1]:.1f}, {b[2]:.1f}, {b[3]:.1f}) conf={s:.3f}")
    elif not len(boxes) and not len(ref_nms):
        print("[verify] (2) 양쪽 모두 검출 0개 (임계값을 낮추거나 표적이 있는 이미지로 확인하세요)")
    else:
        print("[verify] (2) 실패: 검출 개수가 다릅니다")


def main():
    args = parse_args()
    onnx_path, nc, reg_max, nl = export(args)
    if args.verify:
        verify(args, onnx_path, nc, reg_max, nl)
    return 0


if __name__ == "__main__":
    sys.exit(main())
