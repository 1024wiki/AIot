/**
 * @file fusion.cc  融合决策引擎实现
 */
#include "fusion.h"
#include <algorithm>
#include <cmath>
#include <stdio.h>

namespace drv {

/* ======================= 疲劳策略 ====================== */
/**
 * 视觉权重 0.5、生理权重 0.5；视觉未激活时降级为纯生理通道
 * 分数组成：
 *  1. 生理得分 bio (满分 1.0)：
 *     - RMSSD 下降比例 (scale < 0.75 得 0.5分，< 0.6 再得 0.2分)
 *     - 坐姿持续异常 > 20s 得 0.3分
 *  2. 视觉得分 vision (满分 1.0)：
 *     - PERCLOS 超过 0.15 得 0.4分，> 0.3 再得 0.2分
 *     - 眨眼频率 < 12 次/分 得 0.2分
 *     - 15分钟哈欠 >= 2次 得 0.2分
 * 严重分档：
 *  - 综合 score >= 0.8：重度疲劳（强蜂鸣 + 3级振动，持续10s）
 *  - 0.5 <= score < 0.8：轻度疲劳（警告振动2级，持续5s）
 */
Decision FatigueStrategy::evaluate(const BioState &st)
{
    Decision d{WARN_NONE,0,0,0,0};

    double bio = 0.0;
    double scale = st.rmssd_scale.load();
    if (scale < 0.75) bio += 0.5;
    if (scale < 0.60) bio += 0.2;
    if (st.posture_bad_ms.load() > 20*1000) bio += 0.3;
    bio = std::min(1.0, bio);

    double vision = 0.0;
    if (st.vision_active.load()) {
        // 视觉判据（对应面试稿 Q4/Q8）：PERCLOS / 眨眼频率 / 哈欠 / 单秒闭眼时长
        double pc = st.perclos.load();
        if (pc > 0.15) vision += 0.4;
        if (pc > 0.30) vision += 0.2;
        if (st.blink_per_min.load() < 12) vision += 0.2;
        if (st.yawn_count.load() >= 2) vision += 0.2;
        // 60s 内闭眼累计 >9s 等价于平均每秒闭眼 >150ms
        if (st.eye_closed_ms.load() > 9000) vision += 0.2;
        vision = std::min(1.0, vision);

        // 视觉与生理各占 50%
        d.score = 0.5 * vision + 0.5 * bio;
    } else {
        // 视觉未启用或失效：纯生理通道降级
        d.score = bio;
    }

    // 分级输出
    if (d.score >= 0.8) {
        d.warn_id = WARN_FATIGUE_L2; d.motor_lvl = 3;
        d.buzzer_lvl = 3; d.duration_s = 10;
    } else if (d.score >= 0.5) {
        d.warn_id = WARN_FATIGUE_L1; d.motor_lvl = 2;
        d.buzzer_lvl = 0; d.duration_s = 5;
    } else if (!st.vision_active.load() && d.score >= 0.7) {
        // 降级路径：视觉不可用（遮挡/强光）时，纯生理得分仍超0.7 → 轻度振动
        d.warn_id = WARN_FATIGUE_L1; d.motor_lvl = 1;
        d.buzzer_lvl = 0; d.duration_s = 3;
    }
    return d;
}

/* ======================= 坐姿策略 ======================
 * 不良坐姿持续 60s 后轻微振动提醒（1级振动、无蜂鸣），不打扰司机听音乐/导航
 */
Decision PostureStrategy::evaluate(const BioState &st)
{
    Decision d{WARN_NONE,0,0,0,0};
    int p = st.posture_id.load();
    int bad_ms = st.posture_bad_ms.load();
    if (p > POSTURE_NORMAL && bad_ms > 60 * 1000) {
        double intensity = std::min(1.0, bad_ms / 120000.0);
        d.score = 0.4 + 0.2 * intensity;   // 0.4 ~ 0.6（低于空气0.7，仲裁自然排后）
        d.warn_id = WARN_POSTURE;
        d.motor_lvl = 1;
        d.buzzer_lvl = 0;
        d.duration_s = 3;
    }
    return d;
}

/* ======================= 环境策略 ======================
 * 环境特征来自查表标定（EnvWorker）：
 *   酒精：alc_mgl >= 0.09 mg/L（饮酒驾车线）→ 最高优先级（score=1.0 阻断级）
 *   空气：air_bad（等效300ppm）→ 低档蜂鸣+振动10s（score=0.7）
 */
Decision EnvironmentStrategy::evaluate(const BioState &st)
{
    Decision d{WARN_NONE,0,0,0,0};
    if (st.alcohol_detected.load()) {
        d.warn_id = WARN_ALCOHOL; d.motor_lvl = 3;
        d.buzzer_lvl = 3; d.duration_s = 15; d.score = 1.0;
    } else if (st.air_bad.load()) {
        d.warn_id = WARN_AIR; d.motor_lvl = 1;
        d.buzzer_lvl = 1; d.duration_s = 10; d.score = 0.7;
    }
    return d;
}

/* ==================== 融合引擎 ======================
 * 仲裁顺序（与面试稿报警矩阵一致）：
 *   酒精阻断(1.0) > 空气异常(0.7) > 坐姿不良(<=0.6) > 疲劳得分
 */
void FusionEngine::setup_default_strategies()
{
    add_strategy(new EnvironmentStrategy());   // 必须先跑：含酒精阻断短路
    add_strategy(new PostureStrategy());
    add_strategy(new FatigueStrategy());
}

Decision FusionEngine::decide(const BioState &st, uint32_t now_ms, uint32_t warn_gap_ms)
{
    last_details_.clear();
    last_details_.reserve(strategies_.size());
    Decision best{WARN_NONE,0,0,0,0};
    for (auto *s : strategies_) {
        Decision d = s->evaluate(st);
        last_details_.push_back(d);
        if (d.warn_id == WARN_ALCOHOL) return d;          // 酒精阻断级：立即仲裁
        if (d.score > best.score) best = d;
    }

    // 告警去抖：同类告警静默期内不上报，异常消失即重置计时
    if (best.warn_id == WARN_NONE) { last_cmd_ms_ = 0; last_warn_ = WARN_NONE; return best; }
    if (last_warn_ == best.warn_id && (now_ms - last_cmd_ms_) < warn_gap_ms)
        return Decision{WARN_NONE,0,0,0,0};
    last_cmd_ms_ = now_ms; last_warn_ = best.warn_id;
    return best;
}

/* ==================== NbIotReporter 实现 ==================== */

bool NbIotReporter::atf(const std::string &cmd, const char *expect, int timeout_ms)
{
    uart_->write_str(cmd + "\r\n");
    return at_cmd_wait(expect, timeout_ms);
}

bool NbIotReporter::at(const char *cmd, const char *expect, int timeout_ms)
{
    uart_->write_str(std::string(cmd) + "\r\n");
    return at_cmd_wait(expect, timeout_ms);
}

static bool wait_for(Uart *u, const char *expect, int timeout_ms)
{
    char buf[1024]; size_t idx = 0;
    long long deadline = timeout_ms;
    while (deadline > 0) {
        ssize_t n = u->read_bytes((uint8_t*)buf + idx, sizeof(buf)-idx-1);
        if (n > 0) { idx += (size_t)n; buf[idx] = 0;
            if (strstr(buf, expect)) return true;
            if (strstr(buf, "ERROR")) return false;
        }
        usleep(10000); deadline -= 10;
    }
    return false;
}

bool NbIotReporter::at_cmd_wait(const char *expect, int timeout_ms)
{
    return wait_for(uart_, expect, timeout_ms);
}

bool NbIotReporter::init_at_mqtt(const std::string &broker, int port,
                                 const std::string &client,
                                 const std::string &user,
                                 const std::string &pass)
{
    if (!at("AT")) return false;
    at("AT+CSQ");
    if (!at("AT+CGATT=1")) return false;

    char cmd[256];
    snprintf(cmd, sizeof cmd, "AT+MQTTCONN=\"%s\",%d,\"%s\",\"%s\",\"%s\"",
             broker.c_str(), port, client.c_str(), user.c_str(), pass.c_str());
    if (!atf(std::string(cmd), "+MQTTCONN")) return false;
    ok_ = true;
    return true;
}

bool NbIotReporter::publish_json(const std::string &topic, const std::string &json)
{
    std::string esc;
    for (char c : json) { if (c == '"') esc += "\\\""; else esc += c; }
    char cmd[768];
    snprintf(cmd, sizeof cmd, "AT+MQTTPUB=\"%s\",1,0,0,\"%s\"", topic.c_str(), esc.c_str());
    return atf(cmd, "OK", 5000);
}

void NbIotReporter::tick_heartbeat(const BioState &st)
{
    std::string json = "{";
    json += "\"hr\":" + std::to_string(st.hr.load());
    json += ",\"spo2\":" + std::to_string(st.spo2.load());
    json += ",\"rmssd\":" + std::to_string(st.rmssd.load());
    json += ",\"posture\":" + std::to_string(st.posture_id.load());
    json += ",\"warn\":0";
    json += "}";
    if (!ok_) {
        init_at_mqtt("mqtt.ThingsCloud.cn", 1883, "rk3576", "user", "pass");
    }
    publish_json("device/aiot_drv/status", json);
}

} // namespace drv
