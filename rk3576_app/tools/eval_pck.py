#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLOv8-Pose 关键点评测脚本（PCK / NME / 推理耗时）
================================================================
对应设计文档 §6.5 指标目标表与面试稿 Q13 表①。

为什么需要这个脚本：
  ultralytics 自带的 val() 只输出 OKS-based mAP，**不输出 PCK / NME**，
  而本项目的视觉特征（EAR/MAR）依赖关键点几何精度——
  关键点错 3 个像素就足以让 EAR 越过 0.25 的闭眼阈值，产生误报。
  因此需要独立评测关键点回归误差。

指标定义：
  NME (Normalized Mean Error)
      NME = mean_i( ||p_i - g_i||_2 ) / d_norm
      d_norm 取「两眼中心距」或「脸部框对角线」，本脚本默认眼距（更贴合 EAR/MAR）
  PCK@t (Percentage of Correct Keypoints)
      PCK@t = #{i : ||p_i - g_i||_2 / d_norm <= t} / N
      常用 t = 0.05 / 0.10 / 0.15，本项目的验收线是 PCK@0.10 ≥ 0.90

数据格式（与 YOLOv8-Pose 训练集一致）：
  labels/<name>.txt，每行： cls cx cy w h  x1 y1 v1  x2 y2 v2 ... （均已归一化）
  关键点顺序：左眼0-5 / 右眼6-11 / 嘴12-19（共 20 点；未标注点 v=0）

用法：
  python3 eval_pck.py --model /usr/share/fused.rknn \
      --images val/images --labels val/labels --imgsz 640
  # 也可用 ONNX / pt 作为 --model，脚本按后缀自动选择后端
"""
import argparse
import glob
import os
import sys

import numpy as np


# ------------------------------------------------------------ 数据读取
def read_labels(label_path: str, imgsz: int = 640):
    """读 GT，返回 [(cls, box_xywh_norm, kpts[N,3]_norm), ...]。"""
    out = []
    if not os.path.exists(label_path):
        return out
    with open(label_path) as f:
        for line in f:
            v = line.strip().split()
            if len(v) < 5 + 3:
                continue
            cls = int(float(v[0]))
            box = np.array([float(x) for x in v[1:5]], dtype=np.float32)
            rest = np.array([float(x) for x in v[5:]], dtype=np.float32)
            n_kpt = len(rest) // 3
            kpts = rest[:n_kpt * 3].reshape(n_kpt, 3)
            out.append((cls, box, kpts))
    return out


def kpt_scale(kpts: np.ndarray, imgsz: int, mode: str = "eye") -> float:
    """计算归一化分母 d_norm（像素单位）。

    eye  —— 两眼中心欧氏距离；本项目 EAR/MAR 都是比值型指标，用眼距最贴合
    bbox —— 检测框对角线；传统 Human3.6M 口径
    """
    if mode == "eye":
        # 关键点顺序约定：0..5 左眼，6..11 右眼
        left = kpts[0:6, :2]
        right = kpts[6:12, :2]
        lv = left[left[:, 0] > 0]
        rv = right[right[:, 0] > 0]
        if len(lv) and len(rv):
            d = np.linalg.norm(lv.mean(axis=0) - rv.mean(axis=0)) * imgsz
            if d > 1e-3:
                return float(d)
        return float(imgsz) * 0.1      # 退化兜底
    return float(imgsz)                # bbox 模式下由调用方替换


# ------------------------------------------------------------ 推理后端
class Backend:
    """统一推理接口，按文件后缀分派到 rknn / onnx / pt。"""

    def __init__(self, model_path: str, imgsz: int):
        self.imgsz = imgsz
        ext = os.path.splitext(model_path)[1].lower()
        if ext == ".rknn":
            from rknn.api import RKNN
            self.rk = RKNN()
            assert self.rk.load_rknn(model_path) == 0
            assert self.rk.init_runtime() == 0
            self.kind = "rknn"
        elif ext == ".onnx":
            import onnxruntime as ort
            self.sess = ort.InferenceSession(model_path,
                                             providers=["CPUExecutionProvider"])
            self.kind = "onnx"
        elif ext == ".pt":
            from ultralytics import YOLO
            self.yolo = YOLO(model_path)
            self.kind = "pt"
        else:
            raise ValueError(f"不支持的模型格式: {ext}")

    def infer(self, blob_nhwc: np.ndarray):
        """blob_nhwc: [1, H, W, 4] uint8，返回原始输出（未解码）。"""
        if self.kind == "rknn":
            return self.rk.inference(inputs=[blob_nhwc], data_format="nhwc")
        if self.kind == "onnx":
            x = np.transpose(blob_nhwc.astype(np.float32), (0, 3, 1, 2))
            return self.sess.run(None, {"input": x})
        # pt 后端只支持三通道，用于快速对照
        rgb = blob_nhwc[..., :3]
        return self.yolo.predict(rgb, imgsz=self.imgsz, verbose=False)


# ------------------------------------------------------------ 输出解码
def decode_pose(out, kpt_shape=(20, 3), conf_thres=0.25):
    """YOLOv8-Pose 输出解码：DFL 框 + 关键点 + NMS。

    out[0] 形状 [1, 4+nc+2*nk, A]（通道优先，A 为 anchor 数）
    返回 best: {'xyxy', 'kpts'[N,2], 'score'} 或 None
    """
    pred = out[0] if isinstance(out, (list, tuple)) else out
    pred = np.asarray(pred, dtype=np.float32)
    if pred.ndim == 3:
        pred = pred[0]                       # [C, A]
    pred = pred.T                            # [A, C]

    nk = kpt_shape[0]
    nc = pred.shape[1] - 4 - nk * 2
    if nc <= 0:
        return None

    cls_scores = pred[:, 4:4 + nc]
    scores = cls_scores.max(axis=1)
    keep = scores > conf_thres
    if not np.any(keep):
        return None

    pred = pred[keep]
    scores = scores[keep]

    # 框仍为 xywh（相对特征图的网格尺度），此处仅取相对关系供 NME 计算
    boxes = pred[:, :4]
    kpts_flat = pred[:, 4 + nc:4 + nc + nk * 2].reshape(-1, nk, 2)

    best = int(np.argmax(scores))
    return {
        "xywh": boxes[best],
        "kpts": kpts_flat[best],             # [nk, 2]
        "score": float(scores[best]),
    }


# ------------------------------------------------------------ 主流程
def main():
    ap = argparse.ArgumentParser(description="YOLOv8-Pose 关键点 PCK/NME 评测")
    ap.add_argument("--model", required=True, help=".rknn / .onnx / .pt")
    ap.add_argument("--images", required=True, help="验证集图像目录")
    ap.add_argument("--labels", required=True, help="验证集标签目录")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--norm", default="eye", choices=["eye", "bbox"],
                    help="NME 归一化分母，默认 eye")
    ap.add_argument("--kpt-shape", default="20,3",
                    help="关键点形状，本项目 20 点=眼6+6+嘴8")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--limit", type=int, default=0, help=">0 时只评测前 N 张")
    ap.add_argument("--vis", default=None, help="可选：输出误差直方图路径")
    args = ap.parse_args()

    try:
        import cv2
    except ImportError:
        print("[ERR] 需要 opencv-python", file=sys.stderr)
        return 1

    kshape = tuple(int(x) for x in args.kpt_shape.split(","))

    exts = ("*.jpg", "*.jpeg", "*.png", "*.bmp")
    files = []
    for e in exts:
        files += sorted(glob.glob(os.path.join(args.images, e)))
    if args.limit > 0:
        files = files[:args.limit]
    if not files:
        print("[ERR] 未找到验证图像", file=sys.stderr)
        return 1

    print(f"[INFO] 加载模型 {args.model} ...")
    backend = Backend(args.model, args.imgsz)
    print(f"[INFO] 后端 = {backend.kind}，共 {len(files)} 张验证图")

    # EAR/MAR 相关关键点在三组内的局部索引（与 vision.h 的 KPT_PER_EYE/KPT_MOUTH 对齐）
    eye_idx = list(range(0, 12))
    mouth_idx = list(range(12, 20)) if kshape[0] >= 20 else []

    nme_all, nme_eye, nme_mouth = [], [], []
    det_fail = 0

    for i, img_path in enumerate(files):
        stem = os.path.splitext(os.path.basename(img_path))[0]
        lbl = os.path.join(args.labels, stem + ".txt")
        gts = read_labels(lbl)
        if not gts:
            continue

        bgr = cv2.imread(img_path, cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        h0, w0 = bgr.shape[:2]

        # ---- 与板端一致的 letterbox ----
        r = min(args.imgsz / h0, args.imgsz / w0)
        nh, nw = int(round(h0 * r)), int(round(w0 * r))
        resized = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
        canvas = np.full((args.imgsz, args.imgsz, 3), 114, dtype=np.uint8)
        top, left = (args.imgsz - nh) // 2, (args.imgsz - nw) // 2
        canvas[top:top + nh, left:left + nw] = resized

        # 无 NIR 时用灰度复制占位，仅用于跑通指标链路
        nir = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
        blob = np.concatenate([canvas, nir[:, :, None]], axis=2)[None, ...]

        try:
            raw = backend.infer(blob)
        except Exception as e:                       # noqa: BLE001
            det_fail += 1
            continue

        pred = decode_pose(raw, kpt_shape=kshape, conf_thres=args.conf)
        if pred is None:
            det_fail += 1
            continue

        # ---- 取 GT 主目标 ----
        cls, box, gt_k = gts[0]
        gt_px = gt_k[:, :2].copy()
        gt_px[:, 0] = gt_px[:, 0] * args.imgsz      # 已在 imgsz 坐标系（letterbox 后）
        gt_px[:, 1] = gt_px[:, 1] * args.imgsz
        vis = gt_k[:, 2] > 0

        pr_k = pred["kpts"].copy()
        n = min(len(gt_px), len(pr_k))
        gt_px, pr_k, vis = gt_px[:n], pr_k[:n], vis[:n]
        if not np.any(vis):
            continue

        # 归一化分母 d_norm（像素）：eye 模式取两眼中心距，bbox 模式取 GT 框对角线
        if args.norm == "bbox":
            d = float(np.hypot(box[2] * args.imgsz, box[3] * args.imgsz))
            d = max(d, 1e-6)
        else:
            d = kpt_scale(gt_k, args.imgsz, "eye")

        err = np.linalg.norm(pr_k - gt_px, axis=1) / max(d, 1e-6)
        nme_all.append(float(err[vis].mean()))

        if eye_idx:
            m = np.zeros(n, dtype=bool)
            m[eye_idx[:n]] = True
            m &= vis
            if np.any(m):
                nme_eye.append(float(err[m].mean()))
        if mouth_idx:
            m = np.zeros(n, dtype=bool)
            m[mouth_idx[:n]] = True
            m &= vis
            if np.any(m):
                nme_mouth.append(float(err[m].mean()))

        if (i + 1) % 50 == 0:
            print(f"  已评测 {i + 1}/{len(files)}")

    if not nme_all:
        print("[ERR] 无有效评测样本", file=sys.stderr)
        return 1

    arr = np.array(nme_all)
    # 数据量纲：err 已是归一化误差，PCK@t 直接比较
    res = {
        "samples": len(arr),
        "det_fail": det_fail,
        "NME_all": float(arr.mean()),
        "NME_eye": float(np.mean(nme_eye)) if nme_eye else float("nan"),
        "NME_mouth": float(np.mean(nme_mouth)) if nme_mouth else float("nan"),
    }
    for t in (0.05, 0.10, 0.15):
        res[f"PCK@{t:.2f}"] = float((arr <= t).mean())

    print("=" * 62)
    print(" 关键点评测结果")
    print("=" * 62)
    print(f" 有效样本        : {res['samples']}  (检测失败 {res['det_fail']})")
    print(f" NME (全部)      : {res['NME_all']:.4f}")
    print(f" NME (眼部 12点) : {res['NME_eye']:.4f}")
    print(f" NME (嘴部 8点)  : {res['NME_mouth']:.4f}")
    print(f" PCK@0.05        : {res['PCK@0.05'] * 100:.2f}%")
    print(f" PCK@0.10        : {res['PCK@0.10'] * 100:.2f}%   <- 验收线 ≥ 90%")
    print(f" PCK@0.15        : {res['PCK@0.15'] * 100:.2f}%")
    print("=" * 62)

    ok = res["PCK@0.10"] >= 0.90
    print(f"[{'PASS' if ok else 'FAIL'}] PCK@0.10 验收（阈值 90%）")

    if args.vis:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            plt.figure(figsize=(7, 4))
            plt.hist(arr, bins=40, color="#3b7dd8", edgecolor="white")
            plt.axvline(0.10, color="r", linestyle="--", label="PCK@0.10")
            plt.xlabel("Normalized Keypoint Error")
            plt.ylabel("Count")
            plt.title(f"NME distribution (N={len(arr)})")
            plt.legend()
            plt.tight_layout()
            plt.savefig(args.vis, dpi=120)
            print(f"[OK] 误差直方图已保存: {args.vis}")
        except ImportError:
            print("[WARN] 未安装 matplotlib，跳过可视化")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
