/**
 * @file proto.h
 * @brief STM32 <---> RK3576 自定义 UART 协议定义
 *        HEAD(2B,0x55AA小端) | VER(1B) | TYPE(1B) | LEN(2B) | TICK(4B) | PAYLOAD | CRC16(2B)
 *        CRC16-CCITT poly=0x1021 init=0xFFFF, 覆盖 VER..PAYLOAD
 * @note  与 `基于AIoT的健康驾驶辅助系统-设计文档-v2.md` §5 对应
 */
#pragma once
#include <cstdint>
#include <cstddef>

namespace drv {

/* ============================== 帧格式常量 ============================== */
constexpr uint8_t  PROTO_VER        = 0x01;
constexpr uint8_t  FRAME_HEAD0      = 0x55;   // 小端先发低字节
constexpr uint8_t  FRAME_HEAD1      = 0xAA;
constexpr size_t   FRAME_HDR_SIZE   = 10;     // 2+1+1+2+4
constexpr size_t   FRAME_MAX_PAYLOAD = 512;
constexpr size_t   FRAME_MAX_SIZE   = FRAME_HDR_SIZE + FRAME_MAX_PAYLOAD + 2;

/* ============================== 消息类型枚举 ============================== */
/* 上行 STM32 -> RK3576 */
enum UpType : uint8_t {
    UP_HEART      = 0x01,   // 心率血氧 IR/RED 原始数据
    UP_PRESSURE   = 0x02,   // FSR402×4 原始 ADC
    UP_ENV        = 0x03,   // MQ-3 + MQ-135
    UP_HEARTBEAT  = 0x04,   // 节点自检心搏
};

/* 下行 RK3576 -> STM32 */
enum DownType : uint8_t {
    DOWN_WARN_CMD   = 0x81, // 预警执行命令（马达/蜂鸣器）
    DOWN_SAMPLE_CFG = 0x82, // 采样频率下发配置
    DOWN_TIME_SYNC  = 0x83, // 时间基准下发
    DOWN_MODE       = 0x84, // 工作模式（0=normal,1=report_only)
    DOWN_RESET      = 0x85, // 链路复位请求
};

/* ============================== Payload 结构 ============================== */
#pragma pack(push, 1)

struct HeartPayload {            // TYPE=0x01
    uint8_t  fifo_len;           // 本帧包含的 IR/RED 对数 (<=32)
    struct { uint32_t ir; uint32_t red; } samp[32];
};

struct PressurePayload {         // TYPE=0x02
    uint16_t ch[4];              // 4 路 FSR402 原始 ADC 值
};

struct EnvPayload {              // TYPE=0x03
    uint16_t mq3;
    uint16_t mq135;
};

struct NodeHeartbeatPayload {    // TYPE=0x04
    uint8_t  battery_pct;
    uint8_t  soc_state;
    uint32_t uptime_ms;
};

struct WarnCmdPayload {          // TYPE=0x81
    uint8_t  warn_id;            // 0 无 1 疲劳轻度 2 疲劳重度 3 坐姿不良 4 酒精 5 空气
    uint8_t  motor_lvl;          // 0-3 振动强度
    uint8_t  buzzer_lvl;         // 0-3 蜂鸣强度
    uint8_t  duration_s;         // 持续秒数
};

struct SampleCfgPayload {        // TYPE=0x82
    uint8_t  sensor_id;          // 1=MQ3 2=MQ135 3=FSR 4=MAX30102
    uint8_t  new_hz;             // 例 10 = 10*0.1Hz => 1Hz; 1 = 0.1Hz
};

struct TimeSyncPayload {         // TYPE=0x83
    uint32_t rk_ms;
};

struct ModePayload {             // TYPE=0x84
    uint8_t  mode;               // 0 normal, 1 report_only
};

struct ResetPayload {            // TYPE=0x85
    uint8_t  reason;
};

#pragma pack(pop)

/* ============================== 业务/预警枚举 ============================== */
enum WarnId : uint8_t {
    WARN_NONE      = 0,
    WARN_FATIGUE_L1 = 1,
    WARN_FATIGUE_L2 = 2,
    WARN_POSTURE   = 3,
    WARN_ALCOHOL   = 4,
    WARN_AIR       = 5,
};

enum PostureId : int {
    POSTURE_NORMAL = 0,
    POSTURE_LEFT   = 1,
    POSTURE_RIGHT  = 2,
    POSTURE_FORWARD= 3,
    POSTURE_RECLINE= 4,
    POSTURE_UNKNOWN= -1,
};

/* ============================== CRC16 ============================== */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/* ============================== 帧解码状态机 ============================== */
/**
 * @brief 增量字节流解析机：支持跨 read() 半帧、坏帧/Garbage 自动重同步。
 *        用法：每读到一字节 feed(byte)，返回 true 时调用 take() 提取完整帧。
 */
class FrameParser {
public:
    FrameParser() { reset(); }

    void reset() {
        state_ = S_HEAD0; idx_ = 0; want_ = 0; have_frame_ = false;
    }

    bool feed(uint8_t b) {
        switch (state_) {
        case S_HEAD0:
            if (b == FRAME_HEAD0) { buf_[0]=b; idx_=1; state_=S_HEAD1; }
            break;
        case S_HEAD1:
            if (b == FRAME_HEAD1) { buf_[idx_++]=b; state_=S_HDR; }
            else if (b == FRAME_HEAD0) { idx_=1; /* 连续0x55保持 */ }
            else { reset(); }
            break;
        case S_HDR: { // 收 VER..TICK 共8字节，凑满 FRAME_HDR_SIZE
            buf_[idx_++] = b;
            if (idx_ == FRAME_HDR_SIZE) {
                want_ = buf_[4] | (buf_[5] << 8);           // LEN(2B LE)
                if (want_ > FRAME_MAX_PAYLOAD) { reset(); } // 异常长度重同步
                else state_ = (want_ ? S_BODY : S_TAIL);
            }
            break; }
        case S_BODY:
            buf_[idx_++] = b;
            if (idx_ == FRAME_HDR_SIZE + want_) state_ = S_TAIL;
            break;
        case S_TAIL: // CRC 低字节
            buf_[idx_++] = b; state_ = S_CRC_HI; break;
        case S_CRC_HI: {
            buf_[idx_++] = b;
            uint16_t got = buf_[idx_-2] | (buf_[idx_-1] << 8);
            uint16_t exp = crc16_ccitt(buf_ + 2, idx_ - 4); // VER..PAYLOAD
            if (got == exp) have_frame_ = true;
            state_ = S_HEAD0; // 不 reset()，保留 buf 供 take()
            break; }
        }
        return have_frame_;
    }

    bool has_frame() const { return have_frame_; }

    /// 提取帧（含头含CRC），返回帧长
    size_t take(uint8_t *out) {
        if (!have_frame_) return 0;
        size_t n = idx_;
        for (size_t i=0;i<n;i++) out[i]=buf_[i];
        have_frame_ = false; idx_ = 0; want_ = 0; state_ = S_HEAD0;
        return n;
    }

private:
    enum : int { S_HEAD0, S_HEAD1, S_HDR, S_BODY, S_TAIL, S_CRC_HI };
    int     state_;
    uint8_t buf_[FRAME_MAX_SIZE];
    uint16_t idx_;
    uint16_t want_;
    bool     have_frame_;
};

/* ============================== 帧编码 ============================== */
/// 组一帧：向 out 写完整字节序列，返回总长度；失败返回0
inline size_t frame_encode(uint8_t type, const void *payload, uint16_t plen,
                           uint32_t tick_ms, uint8_t *out)
{
    if (plen > FRAME_MAX_PAYLOAD) return 0;
    out[0] = FRAME_HEAD0; out[1] = FRAME_HEAD1;
    out[2] = PROTO_VER;   out[3] = type;
    out[4] = plen & 0xFF; out[5] = (plen >> 8) & 0xFF;
    out[6] = tick_ms & 0xFF; out[7] = (tick_ms>>8)&0xFF;
    out[8] = (tick_ms>>16)&0xFF; out[9] = (tick_ms>>24)&0xFF;
    for (uint16_t i=0;i<plen;i++) out[FRAME_HDR_SIZE+i] = ((const uint8_t*)payload)[i];
    uint16_t crc = crc16_ccitt(out + 2, 8 + plen);
    out[FRAME_HDR_SIZE+plen]   = crc & 0xFF;
    out[FRAME_HDR_SIZE+plen+1] = (crc >> 8) & 0xFF;
    return FRAME_HDR_SIZE + plen + 2;
}

} // namespace drv
