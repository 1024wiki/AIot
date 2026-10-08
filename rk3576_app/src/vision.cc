/**
 * @file vision.cc  视觉管线实现
 *
 *   V4L2 双路采集(RGB/NIR) → RGA 预处理(imcvtcolor+imresize→640x640, 拼4通道)
 *   → rknn_init/inputs_set/run/outputs_get → DFL解码+NMS(CPU) → 14关键点
 *   → VisionAnalyzer(EAR/MAR状态机) → BioState
 *
 *   设计说明：
 *   - 真实推理依赖 RKNN runtime 与 librga，编译时用 -DHAVE_RKNN / -DHAVE_RGA 开启；
 *     未开启时走 SoftVision 桩实现，便于 PC 端先跑通全链路。
 *   - 采集采用「按需启停」：由门控条件触发 start()，会话结束后 shutdown()。
 */
#include "visionpipeline.h"

#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <thread>
#include <chrono>

#ifdef HAVE_RGA
#include <im2d.h>        // librga IM2D API：imcvtcolor / imresize
#endif

#ifdef HAVE_RKNN
#include "rknn_api.h"
#endif

namespace drv {

static uint64_t now_ms() {
    using namespace std::chrono;
    return (uint64_t)duration_cast<milliseconds>(
               steady_clock::now().time_since_epoch()).count();
}

/* ===================== 单路 V4L2 采集 ===================== */
bool V4L2Stream::open_dev(const char *dev, uint32_t w, uint32_t h, uint32_t fmt) {
    fd_ = ::open(dev, O_RDWR | O_NONBLOCK, 0);   // 关键：非阻塞，配合 select/epoll
    if (fd_ < 0) { perror("[vision] open v4l2"); return false; }

    // 1) 设定格式
    struct v4l2_format f{};
    f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    f.fmt.pix.width = w;  f.fmt.pix.height = h;
    f.fmt.pix.pixelformat = fmt;
    f.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(fd_, VIDIOC_S_FMT, &f) < 0) { perror("[vision] S_FMT"); return false; }
    width_ = f.fmt.pix.width; height_ = f.fmt.pix.height; format_ = f.fmt.pix.pixelformat;

    // 2) 申请内核缓冲
    struct v4l2_requestbuffers rb{};
    rb.count = BUF_CNT; rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; rb.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd_, VIDIOC_REQBUFS, &rb) < 0) { perror("[vision] REQBUFS"); return false; }

    // 3) 查询并 MMAP 映射
    bufs_.resize(rb.count);
    for (uint32_t i = 0; i < rb.count; ++i) {
        struct v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
        if (ioctl(fd_, VIDIOC_QUERYBUF, &b) < 0) { perror("[vision] QUERYBUF"); return false; }
        bufs_[i].length = b.length;
        bufs_[i].start = mmap(nullptr, b.length, PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd_, b.m.offset);
        if (bufs_[i].start == MAP_FAILED) { perror("[vision] mmap"); return false; }
    }

    // 4) 全部入队 + 开启流
    for (uint32_t i = 0; i < rb.count; ++i) {
        struct v4l2_buffer b{};
        b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; b.memory = V4L2_MEMORY_MMAP; b.index = i;
        if (ioctl(fd_, VIDIOC_QBUF, &b) < 0) { perror("[vision] QBUF"); return false; }
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) { perror("[vision] STREAMON"); return false; }

    running_ = true;
    return true;
}

void V4L2Stream::close_dev() {
    if (fd_ >= 0) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);
        for (auto &b : bufs_)
            if (b.start && b.start != MAP_FAILED) munmap(b.start, b.length);
        ::close(fd_);
        fd_ = -1;
    }
    bufs_.clear(); running_ = false;
}

/**
 * 非阻塞取帧：成功时把该 buffer 的 mmap 指针与硬件时间戳交给调用方。
 * 注意：调用方使用完必须 dqbuf_done() 归还 buffer。
 */
bool V4L2Stream::dqbuf(const uint8_t **data, size_t *len, uint64_t *ts_ns) {
    struct v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd_, VIDIOC_DQBUF, &b) < 0) {
        if (errno == EAGAIN) return false;    // 暂无帧（非阻塞语义）
        return false;
    }
    cur_index_ = b.index;
    *data = (const uint8_t *)bufs_[b.index].start;
    *len  = b.bytesused;
    *ts_ns = (uint64_t)b.timestamp.tv_sec * 1000000000ULL + b.timestamp.tv_usec * 1000ULL;
    return true;
}

void V4L2Stream::qbuf_back() {
    if (cur_index_ < 0) return;
    struct v4l2_buffer b{};
    b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    b.memory = V4L2_MEMORY_MMAP;
    b.index = (uint32_t)cur_index_;
    ioctl(fd_, VIDIOC_QBUF, &b);
    cur_index_ = -1;
}

/* ===================== 视觉管线 ===================== */
bool VisionPipeline::init(const char *rgb_dev, const char *nir_dev, const char *rknn_path) {
    if (!rgb_.open_dev(rgb_dev, 1600, 1300, V4L2_PIX_FMT_YUYV)) return false;
    if (!nir_.open_dev(nir_dev, 1600, 1300, V4L2_PIX_FMT_GREY)) { rgb_.close_dev(); return false; }
    if (rknn_path && !load_model(rknn_path)) {
        fprintf(stderr, "[vision] 模型加载失败：%s（将使用桩推理）\n", rknn_path);
    }
    analyzer_.reset();
    running_ = true;
    return true;
}

bool VisionPipeline::load_model(const char *path) {
#ifdef HAVE_RKNN
    FILE *fp = fopen(path, "rb");
    if (!fp) return false;
    fseek(fp, 0, SEEK_END); long sz = ftell(fp); fseek(fp, 0, SEEK_SET);
    std::vector<uint8_t> blob(sz);
    if (fread(blob.data(), 1, sz, fp) != (size_t)sz) { fclose(fp); return false; }
    fclose(fp);
    // rknn_init → 加载 rknn 模型到 NPU
    if (rknn_init(&ctx_, blob.data(), blob.size(), 0, nullptr) < 0) return false;
    model_loaded_ = true;
    return true;
#else
    (void)path;
    return false;
#endif
}

/**
 * 单次会话：运行 duration_s 秒的采样 + 推理 + 指标累计。
 * 每帧流程：select 等待双路就绪 → DQBUF(memcpy 后立即 QBUF) → RGA 预处理
 *          → rknn 推理 → 关键点解码 → VisionAnalyzer.update()
 */
void VisionPipeline::run_session(int duration_s, BioState &st, uint32_t frame_period_ms) {
    uint64_t t_end = now_ms() + (uint64_t)duration_s * 1000;

    while (now_ms() < t_end && running_) {
        // ---- 1) select 等待双路任一路可读（超时 5ms，用于无帧时也推进状态机）----
        fd_set rfds; FD_ZERO(&rfds);
        int maxfd = 0;
        if (rgb_.fd() >= 0) { FD_SET(rgb_.fd(), &rfds); maxfd = std::max(maxfd, rgb_.fd()); }
        if (nir_.fd() >= 0) { FD_SET(nir_.fd(), &rfds); maxfd = std::max(maxfd, nir_.fd()); }
        if (maxfd > 0) {
            struct timeval tv{0, 5000};
            int ret = select(maxfd + 1, &rfds, nullptr, nullptr, &tv);
            if (ret < 0 && errno != EINTR) break;
        }

        Keypoints kp{};
        bool got = false;

        // ---- 2) 取帧并做时间戳配对（<5ms 视为同一时刻）----
        if (grab_and_pair(&kp, &got) && got) {
            // ---- 3) RGA 预处理 + 推理 + 关键点解码 ----
            if (infer(&kp)) analyzer_.update(kp, now_ms());
        }

        // ---- 4) 把指标写入共享状态（供融合决策线程读取）----
        const VisionMetrics &m = analyzer_.metrics();
        st.ear.store(m.ear);
        st.mar.store(m.mar);
        st.perclos.store(m.perclos);
        st.blink_per_min.store(m.blink_per_min);
        st.yawn_count.store(m.yawn_count);
        st.eye_closed_ms.store(m.closed_ms);
        st.vision_active.store(true);

        std::this_thread::sleep_for(std::chrono::milliseconds(frame_period_ms));
    }
}

bool VisionPipeline::grab_and_pair(Keypoints *kp, bool *got) {
    (void)kp;
    const uint8_t *rgb_data = nullptr, *nir_data = nullptr;
    size_t rgb_len = 0, nir_len = 0;
    uint64_t rgb_ts = 0, nir_ts = 0;
    bool have_rgb = false, have_nir = false;

    // DQBUF 后必须 memcpy 到应用层内存再 QBUF —— 否则内核复用 buffer 会覆盖数据
    if (rgb_.dqbuf(&rgb_data, &rgb_len, &rgb_ts)) {
        rgb_frame_.assign(rgb_data, rgb_data + rgb_len);
        rgb_.qbuf_back();
        rgb_slot_ts_ = rgb_ts;
        have_rgb = true;
    }
    if (nir_.dqbuf(&nir_data, &nir_len, &nir_ts)) {
        nir_frame_.assign(nir_data, nir_data + nir_len);
        nir_.qbuf_back();
        nir_slot_ts_ = nir_ts;
        have_nir = true;
    }
    if (!have_rgb || !have_nir) { *got = false; return true; }

    // 时间戳配对：|Δt| < 5ms 视为同步帧
    int64_t dt = (int64_t)rgb_slot_ts_ - (int64_t)nir_slot_ts_;
    if (dt < 0) dt = -dt;
    *got = (dt < 5 * 1000 * 1000LL);   // 5ms in ns
    return true;
}

/**
 * 预处理 + 推理 + 关键点解码。
 * 真实实现：RGA imcvtcolor(YUV422→BGR24) + imresize(→640x640)，拼 NIR 成 4 通道，
 *          归一化 + HWC→CHW，再 rknn_inputs_set / rknn_run / rknn_outputs_get，
 *          最后 DFL 解码 + NMS 得到 3 框与 14 关键点。
 */
bool VisionPipeline::infer(Keypoints *kp) {
#ifdef HAVE_RKNN
    if (model_loaded_) {
        // ---- 预处理（RGA 硬件完成色彩转换与缩放）----
        if (!preprocess_rga()) return false;

        // ---- 送入 NPU ----
        rknn_input in{}; in.index = 0; in.buf = input_chw_.data();
        in.size = (uint32_t)input_chw_.size();
        in.type = RKNN_TENSOR_UINT8; in.fmt = RKNN_TENSOR_NCHW; in.pass_through = 0;
        if (rknn_inputs_set(ctx_, 1, &in) < 0) return false;
        if (rknn_run(ctx_, nullptr) < 0) return false;

        rknn_output out{}; out.want_float = 1;
        if (rknn_outputs_get(ctx_, 1, &out, nullptr) < 0) return false;
        bool ok = decode_keypoints((const float *)out.buf, kp);   // DFL 解码 + NMS
        rknn_outputs_release(ctx_, 1, &out);
        return ok;
    }
#endif
    // 无 NPU/模型时：桩数据，保证 PC 端全链路可跑（对应 VisionStub 行为）
    return infer_stub(kp);
}

#ifdef HAVE_RGA
/**
 * RGA 预处理：双路 1600x1300 → 640x640
 *   imcvtcolor: YUV422 → BGR24（硬件色彩空间转换）
 *   imresize:   分辨率缩放
 * 全程不占用 CPU（这也是选择 RK 平台的关键原因）。
 */
bool VisionPipeline::preprocess_rga() {
    rga_buffer_t src = wrapbuffer_virtualaddr((void *)rgb_frame_.data(), 1600, 1300, RK_FORMAT_YUYV_422);
    rga_buffer_t dst = wrapbuffer_virtualaddr(bgr640_.data(), 640, 640, RK_FORMAT_BGR_888);
    imcvtcolor(src, dst, RK_FORMAT_YUYV_422, RK_FORMAT_BGR_888);
    imresize(src, dst);

    // 拼 4 通道：BGR 3 通道 + NIR 单通道（取灰度缩放后结果）
    for (int y = 0; y < 640; ++y)
        for (int x = 0; x < 640; ++x) {
            int di = (y * 640 + x);
            input_chw_[0 * 640 * 640 + di] = bgr640_[di * 3 + 0];
            input_chw_[1 * 640 * 640 + di] = bgr640_[di * 3 + 1];
            input_chw_[2 * 640 * 640 + di] = bgr640_[di * 3 + 2];
            input_chw_[3 * 640 * 640 + di] = nir640_[di];
        }
    return true;
}
#else
bool VisionPipeline::preprocess_rga() { return false; }
#endif

bool VisionPipeline::decode_keypoints(const float *raw, Keypoints *kp) {
    // TODO(板端接入)：DFL 分布解码 + 置信度过滤 + NMS，得到 3 框与 14 关键点
    // 参考 rknn_model_zoo/examples/yolov8/cpp/postprocess.cc
    (void)raw; (void)kp;
    return false;
}

bool VisionPipeline::infer_stub(Keypoints *kp) {
    // 桩：给出几何合理的假关键点，便于验证 EAR/MAR 状态机与融合链路
    static int phase = 0;
    phase = (phase + 1) % 120;
    for (int i = 0; i < KPT_PER_EYE; ++i) {
        kp->eye_l[i][0] = 100.f; kp->eye_l[i][1] = 100.f + (float)i;
        kp->eye_r[i][0] = 140.f; kp->eye_r[i][1] = 100.f + (float)i;
    }
    // 每 60 帧模拟一次闭眼（把上睑下压，EAR 变小）
    if (phase > 50) { kp->eye_l[1][1] = 108.f; kp->eye_l[2][1] = 108.f; }
    for (int i = 0; i < KPT_MOUTH; ++i) {
        kp->mouth[i][0] = 100.f + (float)i * 5.f;
        kp->mouth[i][1] = 200.f;
    }
    if (phase > 100) { kp->mouth[2][1] = 220.f; kp->mouth[6][1] = 220.f; }
    kp->valid = true;
    return true;
}

void VisionPipeline::shutdown() {
    running_ = false;
#ifdef HAVE_RKNN
    if (model_loaded_) { rknn_destroy(ctx_); model_loaded_ = false; }
#endif
    rgb_.close_dev();
    nir_.close_dev();
}

} // namespace drv
