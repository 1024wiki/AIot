/**
 * @file workers.cc  算法 Worker 实现
 */
#include "workers.h"
#include <algorithm>
#include <fstream>
#include <numeric>
#include <cmath>
#include <stdio.h>

namespace drv {

/* ======================= HrvWorker ======================= */

// 简单滑动-EMA 波峰检测：EMA 减去滞留垒值优势的“隆起”视为心跳峰
void HrvWorker::push_sample(uint32_t ir, uint32_t red)
{
    (void)red;
    // 1Hz EMA 平滑 (alpha 0.15)
    if (!ema_init_) { ema_ir_ = ir; ema_init_ = true; }
    double diff = (double)ir - ema_ir_;
    ema_ir_ = 0.85 * ema_ir_ + 0.15 * ir;

    uint64_t now_ns = 0; // 单调时间戳由调用处统一入库，这里用相对样本计数补零
    // 使用样本计数 20ms/step（50Hz）
    static uint64_t sample_cnt = 0;
    now_ns = sample_cnt++ * 20000000ULL;

    // 波峰检测：diff 从正转负且峰值超过阈值
    static double prev_diff = 0.0;
    if (prev_diff > 0 && diff <= 0) {
        if (last_tick_ms_ > 0) {
            double rr_ms = (now_ns / 1e6) - last_tick_ms_;
            if (rr_ms > 300 && rr_ms < 2000) // 生理范围内 30–200bpm
                add_rr(rr_ms);
        }
        last_tick_ms_ = now_ns / 1e6;
        last_peak_ir_ = ir;
        peak_cnt_++;
    }
    prev_diff = diff;
}

void HrvWorker::add_rr(double ms)
{
    rr_buf_.push_back(ms);
    if (rr_buf_.size() > rr_max_) rr_buf_.pop_front();
}

double HrvWorker::compute_rmssd() const
{
    if (rr_buf_.size() < 2) return 0.0;
    double sum = 0.0;
    for (size_t i = 1; i < rr_buf_.size(); ++i) {
        double d = rr_buf_[i] - rr_buf_[i-1];
        sum += d * d;
    }
    return std::sqrt(sum / (rr_buf_.size() - 1));
}

void HrvWorker::inject_rr_interval_ms(double ms) { add_rr(ms); }

void HrvWorker::analyze(BioState &st)
{
    double rmssd = compute_rmssd();
    st.rmssd.store(rmssd);
    if (rr_buf_.size() >= 4) {
        double mean = std::accumulate(rr_buf_.begin(), rr_buf_.end(), 0.0) / rr_buf_.size();
        if (mean > 0) st.hr.store((int)(60000.0 / mean));
    } else st.hr.store(0);
    st.spo2.store(98); // 简易：真实项目用 IR/RED 比值脉冲氧标定

    // 基线自适应：前 60s 数据求均值作为基线； RMSSD 相对比 = rmssd/ baseline
    maybe_update_baseline(rmssd);
    if (baseline_set_ && baseline_rmssd_ > 1.0)
        st.rmssd_scale.store(rmssd / baseline_rmssd_);
    else
        st.rmssd_scale.store(1.0);
}

void HrvWorker::maybe_update_baseline(double v)
{
    if (baseline_set_) return;
    if (v <= 0.0) return;
    static std::deque<double> warm;
    warm.push_back(v);
    if (warm.size() > 60) {        // 约 60s 样本
        baseline_rmssd_ = std::accumulate(warm.begin(), warm.end(), 0.0) / warm.size();
        baseline_set_ = (baseline_rmssd_ > 1.0);
    }
}

/* ======================= PostureKnn ======================= */

bool PostureKnn::load(const char *path)
{
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return false;
    while (ifs) {
        Sample s{};
        ifs.read(reinterpret_cast<char*>(s.f), sizeof s.f);
        if (!ifs) break;
        ifs.read(reinterpret_cast<char*>(&s.label), 1);
        sample_.push_back(s);
    }
    return !sample_.empty();
}

int PostureKnn::classify(const uint16_t adc[4])
{
    if (sample_.empty()) return POSTURE_UNKNOWN;

    float total = 0.f;
    for (int i = 0; i < 4; ++i) total += (float)adc[i];
    if (total <= 1e-6f) return POSTURE_UNKNOWN;
    float f[4] = { (float)adc[0]/total, (float)adc[1]/total,
                   (float)adc[2]/total, (float)adc[3]/total };

    // 求最近 k 个样本（曼哈顿距离）
    struct Cand { float d; int label; };
    std::vector<Cand> cand;
    cand.reserve(sample_.size());
    for (auto &s : sample_) {
        float d = 0;
        for (int i = 0; i < 4; ++i) d += std::fabs(f[i] - s.f[i]);
        cand.push_back({d, s.label});
    }
    std::partial_sort(cand.begin(), cand.begin() + std::min<int>(k_, (int)cand.size()),
                      cand.end(), [](auto&a, auto&b){ return a.d < b.d; });

    int vote[8] = {};
    for (int i = 0; i < std::min<int>(k_, (int)cand.size()); ++i)
        vote[cand[i].label]++;
    int best = 0, lbl = POSTURE_UNKNOWN;
    for (int i = 0; i < 8; ++i)
        if (vote[i] > best) { best = vote[i]; lbl = i; }

    // 稳态滤波：win_size 窗口多数投票
    vote_win_.push_back(lbl);
    if ((int)vote_win_.size() > win_size_) vote_win_.pop_front();
    int wv[8] = {};
    for (int x : vote_win_) if (x >= 0 && x < 8) wv[x]++;
    int wb = 0, wl = lbl;
    for (int i = 0; i < 8; ++i) if (wv[i] > wb) { wb = wv[i]; wl = i; }
    return wl;
}

/* ======================= EnvWorker =======================
 * 查表标定（与面试Q9一致）：ADC→电压→查表插值→浓度
 *   MQ-3  ：V 1.10V→0.09mg/L 饮酒线，V 2.20V→0.36mg/L 醉酒线
 *   MQ-135：V 1.90V→300ppm 空气异常线
 * 预热60s + 连续3次(3s)超阈值才置位，防开机漂移与瞬时干扰（香水等）
 */
void EnvWorker::push_event(uint16_t mq3, uint16_t mq135, BioState &st)
{
    // 1) 预热 60s（MQ 气敏元件加热丝热平衡期，数据不可信直接丢弃）
    int warm = st.warmup_ms.load() + 1000;   // 帧节奏约 1Hz
    st.warmup_ms.store(warm);
    if (warm < 60 * 1000) return;

    // 2) ADC(0~4095, 3.3V) → 电压
    double v_alc = mq3   * 3.3 / 4095.0;
    double v_air = mq135 * 3.3 / 4095.0;

    // 3) 酒精查表 + 线性插值（mg/L）
    static const double alc_tbl[][2] = {
        {0.55, 0.00}, {1.10, 0.09}, {1.65, 0.20}, {2.20, 0.36}, {2.80, 0.60} };
    double alc = 0.0;
    if (v_alc <= alc_tbl[0][0])        alc = 0.0;
    else if (v_alc >= alc_tbl[4][0])   alc = alc_tbl[4][1];
    else for (int i = 0; i < 4; ++i)
        if (v_alc <= alc_tbl[i+1][0]) {
            double k = (v_alc - alc_tbl[i][0]) / (alc_tbl[i+1][0] - alc_tbl[i][0]);
            alc = alc_tbl[i][1] + k * (alc_tbl[i+1][1] - alc_tbl[i][1]);
            break;
        }

    // 4) 空气查表 + 线性插值（等效 ppm）
    static const double air_tbl[][2] = {
        {1.00, 0}, {1.90, 300}, {2.60, 700}, {3.10, 1500} };
    double air = 0.0;
    if (v_air <= air_tbl[0][0])        air = 0.0;
    else if (v_air >= air_tbl[3][0])   air = air_tbl[3][1];
    else for (int i = 0; i < 3; ++i)
        if (v_air <= air_tbl[i+1][0]) {
            double k = (v_air - air_tbl[i][0]) / (air_tbl[i+1][0] - air_tbl[i][0]);
            air = air_tbl[i][1] + k * (air_tbl[i+1][1] - air_tbl[i][1]);
            break;
        }

    st.alc_mgl.store(alc);
    st.mq135_ppm.store((int)air);

    // 5) 连续 3 次（约3s）超阈值才置位告警（过滤瞬时干扰）
    static int alc_cnt = 0, air_cnt = 0;
    if (alc >= cfg_.mq3_alc_thres) ++alc_cnt; else alc_cnt = 0;
    if (air >= cfg_.mq135_bad)     ++air_cnt; else air_cnt = 0;
    st.alcohol_detected.store(alc_cnt >= 3);
    st.air_bad.store(air_cnt >= 3);
}

/* ======================= VisionStub ======================= */

bool VisionStub::tick_once(BioState &st)
{
    if (!running_) return false;
    run_ms_ += 100; // 假定外部 100ms 周期回调
    // 模拟场景：RMSSD 下降 → 视觉 EAR 变低 → PERCLOS 上升
    double scale = st.rmssd_scale.load();
    st.ear.store(0.28 - 0.15 * (1.0 - scale));           // EAR ~0.13-0.28
    st.perclos.store(std::max(0.0, 0.05 + (1.0 - scale) * 0.35));
    st.blink_per_min.store((int)(20 - 10 * (1.0 - scale)));
    st.yawn_count.store(scale < 0.75 ? 1 : 0);
    st.eye_closed_ms.store((uint64_t)((0.05 + (1.0 - scale) * 0.35) * 60000));
    st.vision_active.store(true);
    return true;
}

} // namespace drv
