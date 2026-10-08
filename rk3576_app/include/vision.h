/**
 * @file vision.h
 * @brief 视觉特征计算：EAR/MAR 状态机 → 眨眼次数 / 闭眼时长 / PERCLOS / 哈欠次数
 *
 *        对应设计文档 §6.4 与面试稿 Q4：
 *          - EAR = (||p1-p4|| + ||p2-p5||) / (2*||p0-p3||)，< EAR_THRESH 判闭眼
 *          - MAR = (||p1-p5||+||p2-p6||+||p3-p7||) / (3*||p0-p4||)，> MAR_OPEN 判全张
 *          - 眼睛状态机：开→闭沿记起点，闭→开沿算时长；<1000ms 记一次眨眼
 *          - 嘴部状态机：MAR>0.75 持续≥2s 且间隔>10s 才记一次哈欠（排除说话）
 *          - 所有事件进 60s 滑动窗口，实时产出 blink_per_min / closed_ms / PERCLOS
 */
#pragma once
#include <cstdint>
#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>

namespace drv {

/* ===================== 判据阈值（与面试稿 Q13 一致） ===================== */
constexpr double EAR_CLOSE_THRESH   = 0.25;   // EAR < 0.25 判闭眼
constexpr double MAR_CLOSE_THRESH   = 0.20;   // MAR < 0.2 判嘴闭合
constexpr double MAR_OPEN_THRESH    = 0.75;   // MAR > 0.75 判完全张开
constexpr uint64_t BLINK_MAX_MS     = 1000;   // 闭眼 <1s 计为眨眼（否则视为瞌睡长闭眼）
constexpr uint64_t YAWN_MIN_MS      = 2000;   // 张口持续 ≥2s 才算哈欠
constexpr uint64_t YAWN_DEDUP_MS    = 10000;  // 两次哈欠最小间隔（去重）
constexpr uint64_t WINDOW_MS        = 60000;  // 60s 滑动窗口
constexpr int      KPT_PER_EYE      = 6;      // 每只眼 6 关键点
constexpr int      KPT_MOUTH        = 8;      // 嘴部 8 关键点
constexpr int      KPT_TOTAL        = 14;     // 眼睛 6+6 + 嘴 8（kpt_shape=[14,3]）

/* ===================== 单帧关键点（来自 rknn 输出，像素坐标） ===================== */
struct Keypoints {
    float eye_l[KPT_PER_EYE][2];   // 左眼 6 点：p0 内眦 p1/p2 上睑 p3 外眦 p4/p5 下睑
    float eye_r[KPT_PER_EYE][2];   // 右眼 6 点
    float mouth[KPT_MOUTH][2];     // 嘴 8 点：p0/p4 口角 p1..p3 上唇 p5..p7 下唇
    bool  valid = false;           // 本帧是否检测到有效人脸
};

/* ===================== 视觉指标（写入 BioState 供融合决策） ===================== */
struct VisionMetrics {
    double ear          = 0.0;   // 当前帧 EAR（取左右眼较大者）
    double mar          = 0.0;   // 当前帧 MAR
    double perclos      = 0.0;   // 60s 窗口闭眼时间占比（0~1）
    int    blink_per_min= 0;     // 60s 内眨眼次数
    uint64_t closed_ms  = 0;     // 60s 内闭眼累计时长
    int    yawn_count   = 0;     // 60s 内哈欠次数
    bool   eyes_closed  = false;
    bool   mouth_open   = false;
    bool   valid        = false; // 本帧是否检测到有效人脸
};

/* ===================== 几何计算 ===================== */
inline double dist2d(const float a[2], const float b[2]) {
    double dx = a[0] - b[0], dy = a[1] - b[1];
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * 眼睛闭合度 EAR —— 取左右眼较大者（更不易漏检）
 *   EAR = (||p1-p4|| + ||p2-p5||) / (2 * ||p0-p3||)
 * p0 内眦 / p1 上睑内1/3 / p2 上睑中 / p3 外眦 / p4 下睑内1/3 / p5 下睑中
 */
inline double calc_ear(const float eye[KPT_PER_EYE][2]) {
    double denom = 2.0 * dist2d(eye[0], eye[3]);
    if (denom < 1e-6) return 0.0;
    return (dist2d(eye[1], eye[4]) + dist2d(eye[2], eye[5])) / denom;
}

/**
 * 嘴部张开度 MAR
 *   MAR = (||p1-p5|| + ||p2-p6|| + ||p3-p7||) / (3 * ||p0-p4||)
 * p0 左上口角 / p4 右上口角 / p1..p3 上唇左中右 / p5..p7 下唇左中右
 */
inline double calc_mar(const float mouth[KPT_MOUTH][2]) {
    double denom = 3.0 * dist2d(mouth[0], mouth[4]);
    if (denom < 1e-6) return 0.0;
    return (dist2d(mouth[1], mouth[5]) + dist2d(mouth[2], mouth[6]) +
            dist2d(mouth[3], mouth[7])) / denom;
}

/* ===================== 视觉特征计算器（含双状态机 + 滑动窗口） ===================== */
class VisionAnalyzer {
public:
    VisionAnalyzer() { reset(); }

    void reset() {
        eye_closed_ = mouth_open_ = false;
        close_start_ms_ = open_start_ms_ = last_yawn_ms_ = 0;
        blink_events_.clear();
        closed_events_.clear();   // pair(结束时刻, 持续时长)
        yawn_events_.clear();
        ear_ema_init_ = false;
        ear_ema_ = mar_ema_ = 0.0;
        m_ = VisionMetrics();     // 值语义重置
    }

    /// 每帧调用一次（由视觉线程在拿到关键点后驱动）
    void update(const Keypoints &kp, uint64_t now_ms) {
        if (!kp.valid) { m_.valid = false; return; }
        m_.valid = true;

        // ---- 逐帧几何量：取左右眼较大者，并做 3 帧 EMA 抗抖动 ----
        double ear_raw = std::max(calc_ear(kp.eye_l), calc_ear(kp.eye_r));
        double mar_raw = calc_mar(kp.mouth);
        if (!ear_ema_init_) { ear_ema_ = ear_raw; mar_ema_ = mar_raw; ear_ema_init_ = true; }
        ear_ema_ = 0.6 * ear_ema_ + 0.4 * ear_raw;
        mar_ema_ = 0.6 * mar_ema_ + 0.4 * mar_raw;
        m_.ear = ear_ema_;
        m_.mar = mar_ema_;

        // ---- 眼睛状态机 ----
        bool closed_now = (ear_ema_ < EAR_CLOSE_THRESH);
        if (!eye_closed_ && closed_now) {            // 开→闭沿
            eye_closed_ = true;  close_start_ms_ = now_ms;
        } else if (eye_closed_ && !closed_now) {     // 闭→开沿
            eye_closed_ = false;
            uint64_t dt = now_ms - close_start_ms_;
            if (dt < BLINK_MAX_MS) blink_events_.push_back(now_ms);  // 一次眨眼
            closed_events_.push_back({now_ms, dt});                  // 无论长短都计闭眼时长
        }
        m_.eyes_closed = eye_closed_;

        // ---- 嘴部状态机（持续时间约束，排除说话/唱歌） ----
        bool open_now = (mar_ema_ > MAR_OPEN_THRESH);
        if (!mouth_open_ && open_now) {
            mouth_open_ = true;  open_start_ms_ = now_ms;
        } else if (mouth_open_ && !open_now) {
            mouth_open_ = false;
            uint64_t dur = now_ms - open_start_ms_;
            if (dur >= YAWN_MIN_MS && (now_ms - last_yawn_ms_) > YAWN_DEDUP_MS) {
                yawn_events_.push_back(now_ms);
                last_yawn_ms_ = now_ms;
            }
        }
        m_.mouth_open = mouth_open_;

        // ---- 60s 滑动窗口：出队窗外事件并重算 ----
        prune_and_eval(now_ms);
    }

    const VisionMetrics &metrics() const { return m_; }

private:
    void prune_and_eval(uint64_t now_ms) {
        uint64_t lo = (now_ms > WINDOW_MS) ? (now_ms - WINDOW_MS) : 0;

        while (!blink_events_.empty()  && blink_events_.front()  < lo) blink_events_.pop_front();
        while (!yawn_events_.empty()   && yawn_events_.front()   < lo) yawn_events_.pop_front();

        // 闭眼时长：窗口内事件求和；边界处被截断的段落按比例折算
        uint64_t sum = 0;
        while (!closed_events_.empty() && closed_events_.front().first < lo) {
            auto &e = closed_events_.front();
            if (e.first > lo) sum += (e.first - lo);   // 部分落在窗口内
            closed_events_.pop_front();
        }
        for (auto &e : closed_events_) sum += e.second;

        // 当前仍处于闭眼状态：补上这段已持续的时间
        if (eye_closed_ && close_start_ms_ < now_ms) {
            uint64_t s = std::max<uint64_t>(close_start_ms_, lo);
            sum += (now_ms - s);
        }

        m_.blink_per_min = (int)blink_events_.size();
        m_.yawn_count    = (int)yawn_events_.size();
        m_.closed_ms     = sum;
        m_.perclos       = std::min(1.0, (double)sum / (double)WINDOW_MS);
    }

    VisionMetrics m_;
    bool     eye_closed_    = false;
    bool     mouth_open_    = false;
    bool     ear_ema_init_  = false;
    double   ear_ema_       = 0.0;
    double   mar_ema_       = 0.0;
    uint64_t close_start_ms_= 0;
    uint64_t open_start_ms_ = 0;
    uint64_t last_yawn_ms_  = 0;

    std::deque<uint64_t> blink_events_;                              // 眨眼时刻
    std::deque<std::pair<uint64_t, uint64_t>> closed_events_;        // (结束时刻, 时长)
    std::deque<uint64_t> yawn_events_;                               // 哈欠时刻
};

} // namespace drv
