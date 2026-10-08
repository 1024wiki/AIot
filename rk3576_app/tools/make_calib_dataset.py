#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RKNN INT8 量化校准集生成脚本
================================================================
对应设计文档 §6.5：校准集必须与板端推理时的输入分布一致，
即 **4 通道 CHW float32 [4, 640, 640]**，而不是常见的 3 通道。

本脚本把成对的 RGB / NIR 原图，按 `vision.cc::preprocess_rga()` 完全相同的
几何与色彩管线处理，生成校准 npy，并写出 rknn-toolkit2 需要的 dataset.txt。

与板端预处理的一致性（关键，面试会问）：
  1. RGB:  YUYV -> BGR888 (imcvtcolor) -> letterbox 到 640x640 (imresize, 双线性)
  2. NIR:  GREY(Y8) -> 复制成 3 通道（仅用于拼通道，不做色彩变换）
  3. 归一化：float32，范围保持 0~255（**不做 /255**，mean/std 在 rknn.config 中给）
  4. 摆放：CHW，通道顺序 [B, G, R, NIR]

用法：
  python3 make_calib_dataset.py --rgb rgb/ --nir nir/ --out calib/ --num 300
"""
import argparse
import glob
import os
import sys

import numpy as np


# ------------------------------------------------------------ 几何变换
def letterbox(img: np.ndarray, new: int = 640, color: int = 114) -> np.ndarray:
    """等比缩放 + 灰边填充到 new x new，与 OpenCV/RGA 的 INTER_LINEAR 一致。

    必须与板端 imresize 的缩放策略一致，否则量化后的激活分布会偏移，
    典型现象是量化后 mAP 掉 3~5 个点。
    """
    import cv2

    h, w = img.shape[:2]
    r = min(new / h, new / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    if (nh, nw) != (h, w):
        img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)

    canvas = np.full((new, new, img.shape[2]), color, dtype=img.dtype)
    top = (new - nh) // 2
    left = (new - nw) // 2
    canvas[top:top + nh, left:left + nw] = img
    return canvas


def to_chw4(rgb_path: str, nir_path: str, imgsz: int = 640) -> np.ndarray:
    """读入一对 RGB/NIR 图，输出 [4, imgsz, imgsz] float32。"""
    import cv2

    rgb = cv2.imread(rgb_path, cv2.IMREAD_COLOR)        # BGR uint8
    if rgb is None:
        raise IOError(f"无法读取 {rgb_path}")

    # NIR 可能是单通道灰度或已经伪彩，统一取灰度
    nir = cv2.imread(nir_path, cv2.IMREAD_GRAYSCALE)
    if nir is None:
        raise IOError(f"无法读取 {nir_path}")

    rgb = letterbox(rgb, imgsz)                          # [H,W,3] BGR
    nir = letterbox(nir, imgsz)                          # [H,W]

    chw = np.empty((4, imgsz, imgsz), dtype=np.float32)
    chw[0] = rgb[:, :, 0]      # B
    chw[1] = rgb[:, :, 1]      # G
    chw[2] = rgb[:, :, 2]      # R
    chw[3] = nir               # NIR（灰度直接占单通道）
    return chw


# ------------------------------------------------------------ 主流程
def main():
    ap = argparse.ArgumentParser(description="生成 4 通道 RKNN 量化校准集")
    ap.add_argument("--rgb", required=True, help="RGB 图像目录")
    ap.add_argument("--nir", required=True, help="NIR 图像目录（文件名需与 RGB 对应）")
    ap.add_argument("--out", default="calib", help="输出目录")
    ap.add_argument("--num", type=int, default=300,
                    help="校准样本数，官方建议 200~500；过少会明显掉精度")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--list-name", default="dataset.txt", help="列表文件名")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    try:
        import cv2  # noqa: F401
    except ImportError:
        print("[ERR] 需要 opencv-python：pip install opencv-python", file=sys.stderr)
        return 1

    # 按文件名主干配对（rgb: 0001.jpg, nir: 0001.png 也能配上）
    def stem(p):
        return os.path.splitext(os.path.basename(p))[0]

    rgb_map = {stem(p): p for p in
               sorted(glob.glob(os.path.join(args.rgb, "*.*")))}
    nir_map = {stem(p): p for p in
               sorted(glob.glob(os.path.join(args.nir, "*.*")))}

    pairs = [(k, rgb_map[k], nir_map[k]) for k in sorted(rgb_map) if k in nir_map]
    if not pairs:
        print("[ERR] 未找到任何配对的 RGB/NIR 图像，请检查文件名", file=sys.stderr)
        return 1

    # 均匀抽样，避免只取到同一段连续录制的画面（分布偏差）
    if len(pairs) > args.num:
        step = len(pairs) / args.num
        pairs = [pairs[int(i * step)] for i in range(args.num)]

    print(f"[INFO] 配对成功 {len(pairs)} 组，开始生成 4 通道校准数据 ...")

    entries = []
    for i, (key, rgb_p, nir_p) in enumerate(pairs):
        try:
            chw = to_chw4(rgb_p, nir_p, args.imgsz)
        except IOError as e:
            print(f"[WARN] 跳过 {key}: {e}")
            continue

        npy_name = f"calib_{i:04d}.npy"
        npy_path = os.path.join(args.out, npy_name)
        np.save(npy_path, chw)
        entries.append(os.path.abspath(npy_path))

        if (i + 1) % 50 == 0:
            print(f"  已生成 {i + 1}/{len(pairs)}")

    if not entries:
        print("[ERR] 没有任何有效样本", file=sys.stderr)
        return 1

    list_path = os.path.join(args.out, args.list_name)
    with open(list_path, "w") as f:
        f.write("\n".join(entries) + "\n")

    total_mb = sum(os.path.getsize(p) for p in entries) / 1024 / 1024
    print("=" * 62)
    print(f"[OK] 校准集生成完成")
    print(f"     样本数 : {len(entries)}")
    print(f"     数据量 : {total_mb:.1f} MB")
    print(f"     列表   : {list_path}")
    print("=" * 62)
    print("[下一步] python3 tools/export_rknn.py --weights fused.pt "
          f"--calib {list_path} --out fused.rknn")
    return 0


if __name__ == "__main__":
    sys.exit(main())
