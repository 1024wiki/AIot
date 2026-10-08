/**
 * @file workers.h
 * @brief 算法处理层：心率HRV、坐姿KNN、环境监测、视觉接口
 *        每类独立 Worker 线程，从 SPSC 队列拿数据，结果写到 BioState 共享结构体。
 *        视觉接口留给 V4L2+VMUNet+YOLOv8 管线实现（spi_vision_worker.cc 里跑非
 *        主循环直接调用；无 NPU 可用 stub 输出假数据供联调）。
 */
#pragma once
#include <atomic>
#include <cstdint>
#include <vector>
#include <deque>
#include <cmath>
#include "spsc_queue.h"
#include "proto.h"

namespace drv {

/* =================== 线程间共享的“状态结构体” ===================
 * FuseThread 每 30ms 读它做判定； Worker 线程写各自字段。
 * std::atomic 保证简单字段可见性，容器字段写方先读后写（低风险竞争可接受）。*/
struct BioState {
    // --- 生理指标（Worker1 写） ---
    std::atomic<int>    hr{0};            // bpm 最新心率
    std::atomic<int>    spo2{0};          // %
    std::atomic<double> rmssd{0.0};       // ms，当前窗口 RMSSD
    std::atomic<double> rmssd_scale{1.0}; // 相对基线比例
    std::atomic<bool>   irregular{false}; // 心律失常便携式判定

    // --- 坐姿（Worker2 写） ---
    std::atomic<int>    posture_id{0};        // 稳态坐姿
    std::atomic<int>    posture_raw{0};       // 原始 KNN 结果（未滤波）
    std::atomic<int>    posture_bad_ms{0};    // 连续不良姿态时长 ms

    // --- 环境（Worker3 写） ---
    std::atomic<int>    mq3_ppm{0};
    std::atomic<int>    mq135_ppm{0};
    std::atomic<double> alc_mgl{0.0};    // 酒精浓度 mg/L（查表插值）
    std::atomic<int>    warmup_ms{0};    // 环境通道预热计时（60s预热后才可判定）
    std::atomic<bool>   alcohol_detected{false};
    std::atomic<bool>   air_bad{false};

    // --- 视觉（VisionPipeline → Fuse），除 ear/mar 外均为 60s 滑动窗口统计 ---
    std::atomic<double> ear{0.0};              // 当前帧 EAR
    std::atomic<double> mar{0.0};              // 当前帧 MAR
    std::atomic<double> perclos{0.0};          // 60s 窗口闭眼时间占比
    std::atomic<int>    blink_per_min{0};      // 60s 内眨眼次数（等价次/分）
    std::atomic<int>    yawn_count{0};         // 60s 内哈欠次数
    std::atomic<uint64_t> eye_closed_ms{0};    // 60s 内闭眼累计时长
    std::atomic<bool>   vision_active{false};

    // --- 链路状态 ---
    std::atomic<uint32_t> uart_err_cnt{0};
    std::atomic<uint32_t> last_stm32_tick{0};
};

/* ============================ Worker1: HRV ============================ */
/**
 * MAX30102 50Hz 原始 IR/RED -> IQ-BPF、峰值检测、对方差->RR 间期->RMSSD。
 * 说明：业界标准做法是机械滤波 + 峰值检测 + RRbuffer 求 RMSSD；主生理节拍由芯片FIFO
 * 内部时钟产生，Linux jitter 不影响 RR 值本身。本产品把 PPG 滤波写成可参数配置的
 * 简单 sliding-window 最大值检测，保证可读性，同时不变形。
 */
class HrvWorker {
public:
    explicit HrvWorker(size_t rr_window = 64) : rr_max_(rr_window) {
        baseline_rmssd_ = 0.0; baseline_set_ = false;
    }

    /// 收到一对 IR/RED 原始值即调用（50Hz）
    void push_sample(uint32_t ir, uint32_t red);

    /// 线程主循环可以从队列读批量样本后统一调用 analyze()
    void analyze(BioState &st);

    /// 用于测试直接注入 RR 间期
    void inject_rr_interval_ms(double ms);

private:
    void add_rr(double ms);
    double compute_rmssd() const;
    void maybe_update_baseline(double v);

    std::deque<double> rr_buf_;
    size_t    rr_max_;
    uint32_t  last_peak_ir_ = 0;
    uint64_t  last_tick_ms_ = 0;
    double    ema_ir_ = 0.0;
    bool      ema_init_ = false;
    int       peak_cnt_ = 0;
    double    baseline_rmssd_;
    bool      baseline_set_;
};

/* ============================ Worker2: KNN 坐姿 ============================ */
/**
 * 数据文件：/usr/share/posture_knn.bin 或 config 指定路径
 * 文件格式：连续 N * (float[4] + uint8 label)，label 0..4
 */
class PostureKnn {
public:
    PostureKnn() : k_(5), win_size_(5) {}

    bool load(const char *path);
    int  classify(const uint16_t adc[4]);  // 返回姿态ID 0..4；样本不足返回 -1

private:
    struct Sample { float f[4]; uint8_t label; };
    std::vector<Sample> sample_;
    int k_;
    std::deque<int> vote_win_;
    int win_size_;
};

/* ============================ Worker3: 环境阈值 ============================ */
struct EnvCfg {
    int mq3_air_max   = 150;   // ADC -> ppm 粗标定值，按项目数据标定
    int mq135_air_max = 300;
    int mq3_alc_thres = 30;    // 判定饮酒阈值
    int mq135_bad      = 250;  // 判定空气质量差
};

class EnvWorker {
public:
    void configure(const EnvCfg &cfg) { cfg_ = cfg; }

    /// 内部滋润几维码做 ADC->ppm 的线性化换算
    void push_event(uint16_t mq3, uint16_t mq135, BioState &st);

private:
    int mq3_to_ppm(uint16_t v)  { return v * cfg_.mq3_air_max / 4095; }
    int mq135_to_ppm(uint16_t v){ return v * cfg_.mq135_air_max / 4095; }
    EnvCfg cfg_;
};

/* ====================== VisionPipeline（含Stub） ====================== */
/// 视觉子系统抽象：可以换真 V4L2/VMUNet+YOLOv8 实现；
/// 未绑 RKNN 时用 StubVisionCam 生成渐进数据，保证主融合链路可独立联调。
/// 注：完整视觉指标定义见 vision.h 的 drv::VisionMetrics（EAR/MAR/眨次/哈欠）。
class IVision {
public:
    virtual ~IVision() {}
    virtual bool start() = 0;        // runtime PM 上电 + 打开摄像头 + 加载模型
    virtual void shutdown() = 0;     // 下电、回关
    virtual bool tick_once(BioState &st) = 0;  // 定期捕获更新到 BioState
    virtual bool running() const = 0;
};

/// 默认开发期 Stub：产生与 RMSSD/姿势联动的假指标，用来先走通融合链路
class VisionStub : public IVision {
public:
    bool start() override { running_ = true;  return true; }
    void shutdown() override { running_ = false; }
    bool running() const override { return running_; }
    bool tick_once(BioState &st) override;
private:
    bool running_ = false;
    uint64_t run_ms_ = 0;
};

} // namespace drv
