# rk3576_app —— 基于 AIoT 的健康驾驶辅助系统（RK3576 中央决策端）

> 对应项目：**智键驾行——基于AIoT的健康驾驶辅助系统**
> 本目录是 **RK3576 侧主控守护进程**的完整工程实现，STM32 侧为纯采集节点（Sensor Hub）。

---

## 1. 架构定位

```
┌─────────────────────────────┐          ┌──────────────────────────┐
│        STM32F407            │  UART    │        RK3576            │
│      纯采集节点 (Hub)        │ ───────► │   中央决策节点 (Linux)     │
│                             │  115200  │                          │
│ · MAX30102 (I2C, 50Hz)      │          │ · 帧解析 + CRC16 校验      │
│ · FSR402 ×4 (ADC, 1Hz)      │ ◄─────── │ · 工厂模式 特征提取        │
│ · MQ-3 / MQ-135 (ADC,0.1Hz) │  预警帧   │ · 策略模式 异常仲裁        │
│                             │          │ · 视觉 AI 推理 (NPU)       │
│ 只做采集、组帧、执行预警      │          │ · NB-IoT/MQTT 上行        │
└─────────────────────────────┘          └──────────────────────────┘
                 ▲                                    │
                 │                                    ▼ USB3.0 UVC 双目模组
                 └────────────────────────────────────┘
                      RGB 流 /dev/video0 + NIR 流 /dev/video1 (OV2312 RGB-IR)
```

**设计原则**：STM32 **不承担任何算法**，只负责采样、打时间戳、组帧上传；
所有判定、融合、视觉推理、功耗调度与云端上报全部在 RK3576 侧完成。
这样做的收益是：算法迭代只改上层，无需重新烧录 MCU。

---

## 2. 目录结构

```
rk3576_app/
├── CMakeLists.txt              # 构建脚本，含 USE_RKNN / USE_RGA 开关
├── config/                     # 运行期配置文件（KNN 样本、阈值等）
├── include/
│   ├── proto.h                 # 帧协议定义（0x01~0x04 上行 / 0x81~0x85 下行）
│   ├── crc.h  / crc.cc         # CRC16-CCITT 查表实现
│   ├── uart.h                  # 串口封装（termios，8N1，非阻塞可选）
│   ├── spsc_queue.h            # 单生产者单消费者无锁环形队列
│   ├── reactor.h               # epoll Reactor（串口 fd 事件驱动）
│   ├── workers.h / workers.cc  # HrvWorker / PostureKnn / EnvWorker / VisionStub
│   ├── fusion.h  / fusion.cc   # FusionEngine + 三种策略 + NbIotReporter
│   ├── vision.h                # ★ 视觉特征层：EAR/MAR 状态机 + PERCLOS 滑窗
│   ├── visionpipeline.h        # ★ V4L2 双路采集 + RGA 预处理 + RKNN 推理
│   └── config.h                # 命令行参数与运行期配置
├── src/
│   ├── main.cc                 # 主线程：Reactor 事件循环 + 决策线程 + 门控线程
│   └── crc.cc / workers.cc / fusion.cc / vision.cc
└── tools/
    ├── stm32_simulator.py      # STM32 采集节点模拟器（配合 socat 联调）
    ├── export_rknn.py          # pt → onnx → rknn 全链路转换
    ├── make_calib_dataset.py   # 生成 4 通道 INT8 量化校准集
    └── eval_pck.py             # 关键点 PCK / NME 独立评测
```

★ 标记为本轮新增的视觉模块。

---

## 3. 编译

### 3.1 PC 开发（无 NPU，用视觉 Stub）

```bash
mkdir -p build && cd build
cmake ..
make -j4
```

### 3.2 板端（启用 RKNN + RGA 硬件加速）

```bash
cd build
cmake .. -DUSE_RKNN=ON -DUSE_RGA=ON
make -j4
```

| 选项 | 作用 | 依赖 |
|------|------|------|
| `USE_RKNN=ON` | 定义 `HAVE_RKNN`，链接 `librknnrt`，启用 NPU 推理 | rknn-toolkit2 runtime（板端 .so） |
| `USE_RGA=ON`  | 定义 `HAVE_RGA`，链接 `librga`，用硬件做 CSC + 缩放 | librga |

两个开关都不开时编译为「主链路 + 视觉桩」，可在 PC 上完整验证协议与融合逻辑。

---

## 4. 运行

```bash
./rk3576_app [opts]

  -u --uart STM32串口       默认 /dev/ttyS1
  -n --nbuart NB-IoT 串口   默认 /dev/ttyS2
  -b --baud STM32 波特率
  -B --nbbaud NB-IoT 波特率
  -k --knn 坐姿KNN样本文件   默认 /usr/share/posture_knn.bin
  -N --enable-nb            启用 NB-IoT/MQTT 上报
  -V --vision-stub          使用视觉 Stub（默认）
  -R --real-vision          使用真实视觉管线 (V4L2 + RGA + rknn)
  -1 --rgb  RGB 流设备节点   默认 /dev/video0
  -2 --nir  NIR 流设备节点   默认 /dev/video1
  -M --model rknn 模型路径   默认 /usr/share/fused.rknn
  -q --novision             禁止视觉门控（只跑生理/环境通道）
  -f --fms 融合决策周期 ms
```

### 板端实跑示例

```bash
./rk3576_app -R -1 /dev/video0 -2 /dev/video1 \
             -M /usr/share/fused.rknn \
             -u /dev/ttyS1 -n /dev/ttyS2 -N
```

---

## 5. PC 联调（无需硬件）

用 `socat` 造一对虚拟串口，一边接模拟器当 STM32，一边接主程序：

```bash
# 终端 1：虚拟串口对
socat -d -d PTY,raw,echo=0,link=/tmp/vttyA PTY,raw,echo=0,link=/tmp/vttyB

# 终端 2：STM32 模拟器（持续上传 0x01~0x04 传感器帧）
python3 tools/stm32_simulator.py /tmp/vttyB

# 终端 3：主程序
./build/rk3576_app -u /tmp/vttyA -V -f 200
```

**已验证**：主程序 30s 周期下发 `0x83` 时间同步帧，模拟器成功解析并校验 CRC；
上行传感器帧被主程序解析后写入 `BioState`，融合决策线程正常输出预警等级。

---

## 6. 核心模块说明

### 6.1 帧协议（`proto.h`）

```
| HEAD 0x55AA (2B) | VER (1B) | TYPE (1B) | LEN (2B LE) | TICK (4B LE) | PAYLOAD | CRC16 (2B) |
```

CRC 计算范围：`VER` 起至 `PAYLOAD` 末（即跳过 2 字节 HEAD）。
上行类型：`0x01` 心率血氧 FIFO、`0x02` 压力、`0x03` 环境、`0x04` 心跳。
下行类型：`0x81` 预警命令、`0x82` 采样配置、`0x83` 时间同步、`0x84` 模式、`0x85` 复位。

### 6.2 设计模式

* **工厂模式**（`workers.cc`）：`SensorFactory` 单例按帧类型注册/返回 `ISensorWorker*` 子类
  （`HrvWorker` / `PostureKnn` / `EnvWorker`），统一 `on_frame()` 接口。
* **策略模式**（`fusion.cc`）：`IStrategy` 派生 `EnvironmentStrategy` / `PostureStrategy` /
  `FatigueStrategy`，`FusionEngine::decide()` 按优先级顺序仲裁。

注册顺序：**Environment → Posture → Fatigue**
（安全优先级：酒精 > 坐姿 > 疲劳）。

### 6.3 视觉管线（`vision.h` / `vision.cc` / `visionpipeline.h`）

**触发方式（门控）**：生理/坐姿通道异常（RMSSD 下降或坐姿不良 >60s）才唤醒视觉，
单次会话 60s，日均可省电约 76.8%。

**数据流**：

```
/dev/video0 (RGB YUYV) ─┐
                        ├─ select 双路等待 ─► 时间戳配对 (|Δt| < 5ms)
/dev/video1 (NIR Y8)   ─┘                            │
                                                     ▼
                                    RGA: imcvtcolor(YUYV→BGR888) + imresize(640×640)
                                                     │
                                                     ▼
                                   拼 4 通道 CHW [B,G,R,NIR] → rknn_inputs_set
                                                     │
                                                     ▼
                                          RKNN 推理 (fused.rknn)
                                                     │
                                                     ▼
                        解码关键点（左眼6 + 右眼6 + 嘴8 = 20 点，未用点 v=0）
                                                     │
                                                     ▼
                       EAR / MAR 双状态机 + 60s 滑动窗口 → VisionMetrics
                                                     │
                                                     ▼
                                   写入 BioState（供融合决策线程读取）
```

**EAR（眼睛闭合度）**：

$$\text{EAR}=\frac{\lVert p_1-p_4\rVert+\lVert p_2-p_5\rVert}{2\cdot\lVert p_0-p_3\rVert}$$

阈值 `EAR_CLOSE_THRESH = 0.25`，取左右眼较大者（更不易漏检）。

**MAR（嘴部张开度）**：

$$\text{MAR}=\frac{\lVert p_1-p_5\rVert+\lVert p_2-p_6\rVert+\lVert p_3-p_7\rVert}{3\cdot\lVert p_0-p_4\rVert}$$

阈值：`MAR < 0.20` 闭合，`> 0.75` 完全张开。

**状态机事件判定**（`VisionAnalyzer::update()`）：

| 事件 | 判据 | 去重/约束 |
|------|------|-----------|
| 眨眼 | 闭眼持续 < 1000ms 后重新睁眼 | 计入 `blink_events_` |
| 闭眼段 | EAR < 0.25 持续时长 | `closed_events_` 存 `(结束时刻, 时长)` |
| 哈欠 | 嘴张开持续 ≥ 2000ms 后闭合 | 两次间隔 > 10000ms 去重 |

**60s 滑动窗口输出**（`prune_and_eval()`）：

| 字段 | 含义 |
|------|------|
| `perclos` | 窗口内闭眼时间占比（PERCLOS） |
| `blink_per_min` | 窗口内眨眼次数（等价次/分） |
| `yawn_count` | 窗口内哈欠次数 |
| `closed_ms` | 窗口内闭眼累计时长 |
| `ear` / `mar` | 当前帧瞬时值 |

窗口边界处被截断的闭眼段按 `end - lo` 折算，避免边界丢时长。

### 6.4 融合决策（`fusion.cc`）

**FatigueStrategy**：`bio` 与 `vision` 各占 50%

```cpp
bio:    scale<0.75 +0.5 ; scale<0.60 +0.2 ; posture_bad_ms>60s(门控) / >20s(加权) +0.3
vision: PERCLOS>0.15 +0.4 ; >0.30 +0.2 ; blink<12 +0.2 ; yawn>=2 +0.2 ; eye_closed_ms>9s +0.2
score = 0.5*vision + 0.5*bio
```

分级输出：

| score | 级别 | 马达 | 蜂鸣器 | 时长 |
|-------|------|------|--------|------|
| ≥ 0.8 | 重度疲劳 | 3 | 3 | 10s |
| ≥ 0.5 | 轻度疲劳 | 2 | 0 | 5s |
| 视觉失效且 ≥ 0.7 | 降级告警 | 1 | 0 | 3s |

**PostureStrategy**：`posture_bad_ms > 60s` → score 0.4~0.6，马达 1 档，3s。
**EnvironmentStrategy**：酒精 → score 1.0（马达 3 + 蜂鸣 3，15s）；空气超标 → score 0.7（马达 1 + 蜂鸣 1，10s）。

**酒精阻断**：`EnvironmentStrategy` 判定酒精超标时直接短路返回，不再评估疲劳 —
安全优先级最高，且同类告警在 `warn_gap_ms` 内静默去抖。

### 6.5 NB-IoT 上报（`NbIotReporter`）

模组挂 RK3576 第二路 UART（`/dev/ttyS2`），AT 指令初始化后
`AT+MQTTCONN` 建连并 `publish_json()` 上报聚合状态；
另有 `tick_heartbeat()` 周期心跳。JSON 字段含环境、生理（RMSSD/坐姿）与视觉特征。

---

## 7. 模型部署链路

```bash
# step 1  生成 4 通道 INT8 量化校准集（200~500 张）
python3 tools/make_calib_dataset.py --rgb rgb/ --nir nir/ --out calib/ --num 300

# step 2  pt → onnx → rknn（SS2D 替换为卷积近似）
python3 tools/export_rknn.py --weights fused.pt --out fused.rknn \
        --calib calib/dataset.txt --target rk3576 --ss2d-mode conv --simu-eval

# step 3  关键点精度独立评测（ultralytics 不输出 PCK）
python3 tools/eval_pck.py --model fused.rknn --images val/images --labels val/labels

# step 4  部署
scp fused.rknn root@<board>:/usr/share/fused.rknn
```

**模型结构**：`fused(rgb, nir) = YOLOv8PoseHead( VMUNet(rgb ⊕ nir) )`
输入 `[1, 4, 640, 640]`（通道 0/1/2 = RGB，通道 3 = NIR），
VMUNet 解码输出**固定 3 通道**以复用 COCO 预训练的 Pose 头权重。

**SS2D 处理**：Mamba 选择性扫描含动态 shape，RKNN 无法映射，
`export_rknn.py` 提供 `conv`（depthwise 四方向近似，推荐）/ `attn` / `keep` 三种模式。

---

## 8. 已知限制与后续工作

| 项 | 状态 | 说明 |
|----|------|------|
| `decode_keypoints()` | TODO | DFL 解码 + NMS 需按实际训练头输出对齐 |
| 坐姿 KNN 样本 | 待生成 | 默认 `/usr/share/posture_knn.bin` 缺失时仅告警并降级 |
| NIR 通道归一化 | 沿用 RGB 统计量 | NIR 无 ImageNet 均值，仅做 0~1 归一化 |
| 视觉门控阈值 | 60s | 与设计文档 §6.3 保持一致 |

---

## 9. 文档索引

| 文档 | 内容 |
|------|------|
| `../基于AIoT的健康驾驶辅助系统-设计文档.md` | 系统设计、协议、算法、部署 |
| `../面试问题.md` | Q1~Q13 面试问答稿（含代码与指标表） |
| `../秋招简历.md` | 简历项目描述 |
