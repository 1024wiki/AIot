/**
 * @file visionpipeline.h
 * @brief 视觉管线：V4L2 双路采集(RGB/NIR) + RGA 预处理 + rknn 推理 + 指标计算
 *
 *   职责划分（对应设计文档 §6.4）：
 *     V4L2Stream      —— 单路 V4L2 采集封装（非阻塞 open + MMAP + DQBUF/QBUF）
 *     VisionPipeline  —— 实现在 IVision 接口上，被门控线程按需 start/run/shutdown
 *
 *   编译开关：
 *     -DHAVE_RKNN  启用 rknn-toolkit2/runtime 真推理（否则走桩）
 *     -DHAVE_RGA   启用 librga 硬件色彩转换与缩放（否则走桩）
 */
#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <atomic>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>

#include "workers.h"      // IVision / BioState
#include "vision.h"       // Keypoints / VisionAnalyzer

#ifdef HAVE_RKNN
#include "rknn_api.h"
#endif

namespace drv {

/* ===================== 单路 V4L2 采集 ===================== */
class V4L2Stream {
public:
    static constexpr uint32_t BUF_CNT = 4;

    ~V4L2Stream() { close_dev(); }

    bool open_dev(const char *dev, uint32_t w, uint32_t h, uint32_t fmt);
    void close_dev();

    /// 非阻塞取帧；成功后必须调用 qbuf_back() 归还 buffer
    bool dqbuf(const uint8_t **data, size_t *len, uint64_t *ts_ns);
    void qbuf_back();

    int      fd()      const { return fd_; }
    uint32_t width()   const { return width_; }
    uint32_t height()  const { return height_; }
    uint32_t format()  const { return format_; }
    bool     running() const { return running_; }

private:
    struct Buf { void *start = nullptr; size_t length = 0; };
    int      fd_       = -1;
    uint32_t width_    = 0;
    uint32_t height_   = 0;
    uint32_t format_   = 0;
    bool     running_  = false;
    int      cur_index_= -1;
    std::vector<Buf> bufs_;
};

/* ===================== 视觉管线（实现 IVision） ===================== */
class VisionPipeline : public IVision {
public:
    VisionPipeline() { bgr640_.resize(640 * 640 * 3); nir640_.resize(640 * 640);
                       input_chw_.resize(4 * 640 * 640); }
    ~VisionPipeline() override { shutdown(); }

    /// 初始化：打开双路 V4L2 + 加载 rknn 模型
    bool init(const char *rgb_dev, const char *nir_dev, const char *rknn_path = nullptr);

    /// 运行一次会话：采集 + 推理 + 指标累计，期间实时写入 BioState
    void run_session(int duration_s, BioState &st, uint32_t frame_period_ms = 33);

    /* ---- IVision 接口 ---- */
    bool start() override { running_ = true; return true; }
    void shutdown() override;
    bool tick_once(BioState &st) override {
        // 单步推进（供主循环按节拍调用）：等价于极短会话
        run_session(0, st, 1);
        return running_;
    }
    bool running() const override { return running_; }

    VisionAnalyzer &analyzer() { return analyzer_; }

private:
    bool load_model(const char *path);
    bool grab_and_pair(Keypoints *kp, bool *got);
    bool infer(Keypoints *kp);
    bool preprocess_rga();
    bool decode_keypoints(const float *raw, Keypoints *kp);   // DFL 解码 + NMS
    bool infer_stub(Keypoints *kp);                            // 无 NPU 时的桩

    V4L2Stream      rgb_, nir_;
    VisionAnalyzer  analyzer_;
    std::atomic<bool> running_{false};

    std::vector<uint8_t> rgb_frame_, nir_frame_;   // 应用层帧副本（DQBUF 后 memcpy）
    std::vector<uint8_t> bgr640_, nir640_;         // RGA 输出
    std::vector<uint8_t> input_chw_;               // 4 通道 CHW 输入张量

    uint64_t rgb_slot_ts_ = 0, nir_slot_ts_ = 0;

#ifdef HAVE_RKNN
    rknn_context ctx_  = 0;
    bool model_loaded_ = false;
#endif
};

} // namespace drv
