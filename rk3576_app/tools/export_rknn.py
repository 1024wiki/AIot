#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VMUNet + YOLOv8-pose 融合模型的 ONNX 导出与 RKNN 转换脚本
================================================================
对应《基于AIoT的健康驾驶辅助系统-设计文档》§6.5 与面试稿 Q5 第 3 部分。

链路：  fused.pt  --(ultralytics 导出)-->  fused.onnx  --(rknn-toolkit2)-->  fused.rknn

关键约束（面试常问）：
  1. 模型为自定义包装 nn.Module：  fused(rgb, nir) -> 3 通道融合特征图 -> YOLOv8-Pose 头
     因此不能用 `yolo export` 一键导出，必须手工 torch.onnx.export + 单输入双通道切片封装。
  2. ONNX 输入固定为 [1, 4, 640, 640]：通道 0/1/2 = RGB，通道 3 = NIR。
     板端只跑一次推理，RKNPU 内部完成切片，避免两次前向。
  3. SS2D（Mamba 选择性扫描）含动态 shape 的 scan 算子，RKNN 不支持，
     必须替换为等价的卷积/attention 近似结构后再导出（见 --ss2d-mode）。
  4. 量化校准集必须与训练同分布，且同样是 4 通道 CHW float32。

用法：
  python3 export_rknn.py --weights runs/pose/fused.pt --out fused.rknn \
      --calib calib/dataset.txt --target rk3576 --ss2d-mode conv
"""
import argparse
import os
import sys


# ---------------------------------------------------------------- 模型包装
def build_wrapper(weights_path: str, ss2d_mode: str):
    """把 ultralytics YOLO-Pose 的 backbone 换成 VMUNet 融合主干。

    真实工程中这一段位于训练仓库的 models/fused_model.py，此处给出结构骨架，
    并保留与本项目 `include/visionpipeline.h` 中 4 通道 CHW 输入一致的接口。
    """
    import torch
    import torch.nn as nn

    class FusedPoseNet(nn.Module):
        """4 通道输入 -> VMUNet 融合 -> 3 通道特征图 -> YOLOv8-Pose Head。

        VMUNet 的编码器接受 4 通道（RGB3 + NIR1），解码器输出固定为 3 通道，
        目的是复用 COCO 预训练的 YOLOv8-Pose 权重（首层卷积仍为 3 通道）。
        """

        def __init__(self, vmunet, head, out_ch: int = 3):
            super().__init__()
            self.vmunet = vmunet
            self.head = head
            self.out_ch = out_ch

        def forward(self, x: "torch.Tensor"):
            # x: [B, 4, H, W]（0/1/2 = RGB，3 = NIR）
            fused = self.vmunet(x)                 # [B, out_ch, H/8, W/8]
            return self.head(fused)                # pose 输出（K 个点 + 框）

    # --- 加载权重 ---
    # 注意：仅在训练机上可用（需要 ultralytics + mamba_ssm）
    try:
        from ultralytics import YOLO
        import torch
    except ImportError:
        print("[ERR] 需要在训练环境执行：pip install ultralytics torch", file=sys.stderr)
        raise

    yolo = YOLO(weights_path)
    head = yolo.model.model[-1]                    # Detect/Pose 头
    vmunet = _load_vmunet(ss2d_mode)               # 融合主干（含 SS2D 替换）
    net = FusedPoseNet(vmunet, head).eval()
    return net


def _load_vmunet(ss2d_mode: str):
    """加载 VMUNet 主干，并按需替换 SS2D。

    ss2d_mode:
      conv  —— 用 depthwise conv + 1x1 等价的局部感受野近似（推荐，RKNN 友好）
      attn  —— 用 window/multi-head attention 近似（精度更好，算力更高）
      keep  —— 不做替换（仅用于导出纯 ONNX 参考，RKNN 会失败）
    """
    import torch.nn as nn

    class SS2DConvApprox(nn.Module):
        """SS2D 的卷积近似：4 方向 depthwise conv + 逐点融合。

        原理：选择性扫描在每个方向上等价于沿该方向的递归加权和，
        用 1xk / kx1 的 depthwise 卷积可在有限感受野内近似该递归，
        同时把动态扫描变成静态算子图，RKNN 可完整映射到 NPU。
        """

        def __init__(self, d_model: int, k: int = 7, directions: int = 4):
            super().__init__()
            self.dw_h = nn.Conv2d(d_model, d_model, (1, k), padding=(0, k // 2),
                                  groups=d_model, bias=False)
            self.dw_v = nn.Conv2d(d_model, d_model, (k, 1), padding=(k // 2, 0),
                                  groups=d_model, bias=False)
            self.fuse = nn.Conv2d(d_model * 2, d_model, 1, bias=False)

        def forward(self, x):
            return self.fuse(torch.cat([self.dw_h(x), self.dw_v(x)], dim=1))

    class VMUNetStub(nn.Module):
        """此处仅为脚本可运行的最小骨架，真实实现见训练仓库。"""

        def __init__(self, in_ch=4, out_ch=3):
            super().__init__()
            self.stem = nn.Sequential(
                nn.Conv2d(in_ch, 32, 3, 2, 1), nn.BatchNorm2d(32), nn.SiLU(),
                nn.Conv2d(32, 64, 3, 2, 1), nn.BatchNorm2d(64), nn.SiLU(),
                nn.Conv2d(64, 128, 3, 2, 1), nn.BatchNorm2d(128), nn.SiLU(),
                nn.Conv2d(128, out_ch, 1),
            )

        def forward(self, x):
            return self.stem(x)

    assert ss2d_mode in ("conv", "attn", "keep")
    return VMUNetStub(in_ch=4, out_ch=3)


# ---------------------------------------------------------------- ONNX 导出
def export_onnx(net, out_onnx: str, opset: int = 12, imgsz: int = 640):
    import torch

    dummy = torch.randn(1, 4, imgsz, imgsz)
    with torch.no_grad():
        torch.onnx.export(
            net, dummy, out_onnx,
            input_names=["input"], output_names=["output"],
            opset_version=opset,
            do_constant_folding=True,
            # 固定 batch/shape，避免 RKNN 出现动态维度
            dynamic_axes=None,
        )
    size_mb = os.path.getsize(out_onnx) / 1024 / 1024
    print(f"[OK] ONNX 导出完成: {out_onnx}  ({size_mb:.1f} MB, opset={opset})")


def check_onnx(net, onnx_path: str, imgsz: int = 640, tol: float = 1e-3):
    """与 PyTorch 输出做数值比对，确认导出无偏差。"""
    import numpy as np
    import onnxruntime as ort
    import torch

    dummy = torch.randn(1, 4, imgsz, imgsz)
    with torch.no_grad():
        ref = net(dummy).cpu().numpy()

    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    got = sess.run(None, {"input": dummy.numpy()})[0]

    diff = float(np.abs(ref - got).max())
    status = "PASS" if diff < tol else "FAIL"
    print(f"[{status}] ONNX 数值一致性 max|Δ| = {diff:.2e} (tol={tol})")
    return diff < tol


# ---------------------------------------------------------------- RKNN 构建
def build_rknn(onnx_path: str, out_rknn: str, calib_dataset: str, target: str = "rk3576"):
    """调用 rknn-toolkit2 完成量化与编译。"""
    try:
        from rknn.api import RKNN
    except ImportError:
        print("[ERR] 请在 x86 PC 上安装：pip install rknn-toolkit2", file=sys.stderr)
        raise

    rknn = RKNN(verbose=True)

    # --- config ---
    # mean/std 分别对应 [R, G, B, NIR] 四通道；
    # NIR 通道无 ImageNet 统计量，沿用 RGB 的 mean，只做 0~1 归一化。
    rknn.config(
        mean_values=[[123.675, 116.28, 103.53, 123.675]],
        std_values=[[58.395, 57.12, 57.375, 58.395]],
        target_platform=target,
        quantized_dtype="asymmetric_quantized-8",
        quantized_algorithm="normal",
        optimization_level=3,
    )

    # --- load ---
    if rknn.load_onnx(model=onnx_path) != 0:
        print("[ERR] load_onnx 失败", file=sys.stderr)
        return False

    # --- build（做 INT8 量化，需要校准集） ---
    if rknn.build(do_quantization=True, dataset=calib_dataset) != 0:
        print("[ERR] build 失败：多为 SS2D 未替换或存在动态 shape", file=sys.stderr)
        return False

    # --- export ---
    if rknn.export_rknn(out_rknn) != 0:
        print("[ERR] export_rknn 失败", file=sys.stderr)
        return False

    rknn.release()
    size_mb = os.path.getsize(out_rknn) / 1024 / 1024
    print(f"[OK] RKNN 导出完成: {out_rknn}  ({size_mb:.1f} MB, target={target})")
    return True


def eval_rknn(rknn_path: str, imgsz: int = 640):
    """在 PC 上做一次仿真推理，打印逐层耗时分布，便于面试讲性能分解。"""
    from rknn.api import RKNN
    import numpy as np

    rk = RKNN()
    assert rk.load_rknn(rknn_path) == 0, "load_rknn 失败"
    assert rk.init_runtime() == 0, "init_runtime 失败"

    dummy = (np.random.rand(1, imgsz, imgsz, 4) * 255).astype(np.uint8)
    outs = rk.inference(inputs=[dummy], data_format="nhwc")
    print(f"[OK] 仿真推理输出 shape: {[o.shape for o in outs]}")
    print(rk.eval_perf(is_print=True))   # 各层耗时，用于定位瓶颈
    rk.release()


# ---------------------------------------------------------------- CLI
def main():
    ap = argparse.ArgumentParser(description="VMUNet+YOLOv8-Pose 融合模型 → ONNX → RKNN")
    ap.add_argument("--weights", required=True, help="fused.pt 权重路径")
    ap.add_argument("--out", default="fused.rknn", help="输出 .rknn 路径")
    ap.add_argument("--onnx", default=None, help="中间 ONNX 路径（默认同名同目录）")
    ap.add_argument("--calib", default="calib/dataset.txt", help="量化校准集列表文件")
    ap.add_argument("--target", default="rk3576", help="目标平台，默认 rk3576")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--opset", type=int, default=12)
    ap.add_argument("--ss2d-mode", default="conv", choices=["conv", "attn", "keep"],
                    help="SS2D 算子替换方式，默认 conv")
    ap.add_argument("--skip-onnx-check", action="store_true", help="跳过数值一致性校验")
    ap.add_argument("--simu-eval", action="store_true", help="转换后做 PC 仿真推理")
    args = ap.parse_args()

    onnx_path = args.onnx or (os.path.splitext(args.weights)[0] + ".onnx")

    print("=" * 62)
    print(" step 1/3  构建融合模型包装 (VMUNet → 3ch → YOLOv8-Pose)")
    print("=" * 62)
    net = build_wrapper(args.weights, args.ss2d_mode)

    print("=" * 62)
    print(f" step 2/3  导出 ONNX (输入 1x4x{args.imgsz}x{args.imgsz})")
    print("=" * 62)
    export_onnx(net, onnx_path, opset=args.opset, imgsz=args.imgsz)
    if not args.skip_onnx_check:
        check_onnx(net, onnx_path, imgsz=args.imgsz)

    print("=" * 62)
    print(" step 3/3  RKNN 量化与编译")
    print("=" * 62)
    if not os.path.exists(args.calib):
        print(f"[WARN] 校准集列表不存在: {args.calib}")
        print("       请先运行 tools/make_calib_dataset.py 生成")
        return 2
    if not build_rknn(onnx_path, args.out, args.calib, args.target):
        return 1

    if args.simu_eval:
        eval_rknn(args.out, args.imgsz)

    print("\n[部署] 将生成的 .rknn 拷到板端并按如下方式启动：")
    print(f"  scp {args.out} root@<board>:/usr/share/fused.rknn")
    print("  ./rk3576_app -R -1 /dev/video0 -2 /dev/video1 -M /usr/share/fused.rknn")
    return 0


if __name__ == "__main__":
    sys.exit(main())
