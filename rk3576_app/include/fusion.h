/**
 * @file fusion.h  融合决策引擎 + 下行命令生成 + NB-IoT 上报接口
 *        决策策略用“策略模式”挂插，便于切换加权/规则/模板匹配策略。
 */
#pragma once
#include <atomic>
#include <cstdint>
#include <functional>
#include <vector>
#include <string>
#include "proto.h"
#include "workers.h"
#include "uart.h"

namespace drv {

/* ==================== 融合决策策略接口 ==================== */
struct Decision {
    WarnId warn_id;
    uint8_t motor_lvl;
    uint8_t buzzer_lvl;
    uint8_t duration_s;
    double score;
};

/**
 * 策略模式： IStrategy 只有一个方法 evaluate(state,&Decision)
 * FuseEngine 持有 vector<IStrategy*>，运行所有策略取最高分
 */
class IStrategy {
public:
    virtual ~IStrategy() {}
    virtual Decision evaluate(const BioState &st) = 0;
    virtual const char* name() const = 0;
};

/// 疲劳策略（生理门控→视觉确认→视觉得分0.5 + 生理得分0.5 等权融合）
class FatigueStrategy : public IStrategy {
public:
    Decision evaluate(const BioState &st) override;
    const char* name() const override { return "fatigue"; }
};

/// 坐姿不良策略（持续 60s 后轻微振动提醒，不打扰听音乐）
class PostureStrategy : public IStrategy {
public:
    Decision evaluate(const BioState &st) override;
    const char* name() const override { return "posture"; }
};

/// 环境异常策略（查表标定浓度：酒精超标最高优先，空气浑浊低档提醒）
class EnvironmentStrategy : public IStrategy {
public:
    Decision evaluate(const BioState &st) override;
    const char* name() const override { return "environment"; }
};

/* ==================== 融合引擎 ==================== */
class FusionEngine {
public:
    FusionEngine() { setup_default_strategies(); }

    void add_strategy(IStrategy *s) { strategies_.push_back(s); }
    void setup_default_strategies();

    /// 每 30ms 调用一次；st 为最新 BioState，返回判定后的顶层决策
    /// 仲裁优先级：酒精阻断(1.0) > 空气异常(0.7) > 坐姿不良(<=0.6) > 疲劳得分
    /// 命令去抖：上一次告警后静默 warn_gap_ms 内不重复下发（WARN_NONE 期间清零）
    Decision decide(const BioState &st, uint32_t now_ms, uint32_t warn_gap_ms);

    /// 按分层报告：每个策略的独立得分（用于 UI 或供上位机诊断）
    const std::vector<Decision>& details() const { return last_details_; }

private:
    std::vector<IStrategy*> strategies_;
    std::vector<Decision>   last_details_;
    uint32_t last_cmd_ms_{0};   // 上次命令下发时刻（去抖基准）
    uint8_t  last_warn_{0};     // 上次告警类型（去抖基准）
};

/* ==================== 下行命令构造器 ==================== */
inline size_t make_warn_frame(uint8_t warn_id, uint8_t motor_lvl, uint8_t buzzer_lvl,
                              uint8_t dur_s, uint32_t tick_ms, uint8_t *out)
{
    WarnCmdPayload p{};
    p.warn_id = warn_id; p.motor_lvl = motor_lvl;
    p.buzzer_lvl = buzzer_lvl; p.duration_s = dur_s;
    return frame_encode(DOWN_WARN_CMD, &p, sizeof p, tick_ms, out);
}

inline size_t make_sample_cfg_frame(uint8_t sensor_id, uint8_t new_hz,
                                    uint32_t tick_ms, uint8_t *out)
{
    SampleCfgPayload p{sensor_id, new_hz};
    return frame_encode(DOWN_SAMPLE_CFG, &p, sizeof p, tick_ms, out);
}

inline size_t make_time_sync_frame(uint32_t rk_ms, uint8_t *out) {
    TimeSyncPayload p{rk_ms};
    return frame_encode(DOWN_TIME_SYNC, &p, sizeof p, rk_ms, out);
}

/* ==================== NB-IoT 上报接口（AT-MQTT 方案 A / PPP+paho 方案 B） ==================== */
class NbIotReporter {
public:
    NbIotReporter(Uart *uart) : uart_(uart), ok_(false) {}
    /// 方案A：纯 AT-MQTT 透传模组，如 YNH-M5311
    bool init_at_mqtt(const std::string &broker, int port,
                      const std::string &client, const std::string &user,
                      const std::string &pass);
    /// JSON 发布： AT+MQTTPUB
    bool publish_json(const std::string &topic, const std::string &json);
    /// 每 60s 心跳与状态上报
    void tick_heartbeat(const BioState &st);

    bool online() const { return ok_; }

private:
    bool at(const char *cmd, const char *expect = "OK", int timeout_ms = 3000);
    bool atf(const std::string &cmd, const char *expect = "OK", int timeout_ms = 3000);
    bool at_cmd_wait(const char *expect, int timeout_ms);

    Uart *uart_;
    bool ok_;
    std::string topic_;
};

} // namespace drv
