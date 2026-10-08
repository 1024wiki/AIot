/**
 * @file main.cc
 * @brief RK3576 守护主进程：三层架构
 *         接入层(UART epoll) -> 队列 -> 算法Workers -> 共享BioState
 *         -> FuseThread(门控+策略) -> 0x81命令下行 & MQTT上行
 *
 *         平台：Linux (RK3576 主机 或 socat 虚拟串口模拟测试)
 *         作者：项目主责 RK3576 侧
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <stdint.h>
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <sys/timerfd.h>
#include <sys/select.h>

#include "proto.h"
#include "spsc_queue.h"
#include "uart.h"
#include "reactor.h"
#include "workers.h"
#include "fusion.h"
#include "visionpipeline.h"
#include "config.h"

using namespace drv;

static std::atomic<bool> g_run{true};
static void on_sigint(int) { g_run.store(false); }

/* ------------------------- 数据流线 ------------------------- */
static SpscQueue<HeartPayload,    128> q_heart;
static SpscQueue<PressurePayload,  32> q_press;
static SpscQueue<EnvPayload,       32> q_env;
static SpscQueue<NodeHeartbeatPayload, 16> q_hb;

static BioState     g_state;
static FusionEngine g_fusion;
static IVision     *g_vision = nullptr;

static Uart  uart_stm;
static Uart  uart_nb;
static NbIotReporter g_nb(&uart_nb);

static Config g_cfg;
static EnvCfg g_envcfg;

/* ------------------------- UART RX 分发 ------------------------- */
static void on_stm32_readable(int fd, uint32_t ev)
{
    (void)ev;
    uint8_t buf[256];
    ssize_t n = uart_stm.read_bytes(buf, sizeof buf);
    if (n <= 0) return;

    static FrameParser p;
    for (ssize_t i = 0; i < n; ++i) {
        if (!p.feed(buf[i])) continue;
        uint8_t frm[FRAME_MAX_SIZE];
        size_t flen = p.take(frm);
        if (flen < FRAME_HDR_SIZE + 2) continue;

        uint8_t  type = frm[3];
        uint8_t *pay  = frm + FRAME_HDR_SIZE;
        uint16_t plen = frm[4] | (frm[5] << 8);
        uint32_t tick = frm[6] | (frm[7]<<8) | (frm[8]<<16) | ((uint32_t)frm[9]<<24);

        g_state.last_stm32_tick.store(tick);

        // 工厂模式分发：按帧 TYPE 取对应队列，统一入口处理三类上行数据
        // （心率血氧 0x01 / 4路压力 0x02 / 空气酒精 0x03）
        switch (type) {
        case UP_HEART: {
            if (plen >= 1) {
                HeartPayload h{};
                memcpy(&h, pay, std::min<size_t>(plen, sizeof h));
                q_heart.push(h);
            }
            break; }
        case UP_PRESSURE: {
            if (plen >= sizeof(PressurePayload)) {
                PressurePayload pp{}; memcpy(&pp, pay, sizeof pp);
                q_press.push(pp);
            }
            break; }
        case UP_ENV: {
            if (plen >= sizeof(EnvPayload)) {
                EnvPayload e{}; memcpy(&e, pay, sizeof e);
                q_env.push(e);
            }
            break; }
        case UP_HEARTBEAT: {
            if (plen >= sizeof(NodeHeartbeatPayload)) {
                NodeHeartbeatPayload hb{}; memcpy(&hb, pay, sizeof hb);
                q_hb.push(hb);
            }
            break; }
        default: break;
        }
    }
}

/* ------------------------- Worker Threads ------------------------- */
static void heart_worker()
{
    HrvWorker w;
    while (g_run) {
        HeartPayload h;
        while (q_heart.pop(h)) {
            for (int i = 0; i < h.fifo_len && i < 32; ++i)
                w.push_sample(h.samp[i].ir, h.samp[i].red);
        }
        w.analyze(g_state);
        usleep(5000); // 5ms
    }
}

static void posture_worker()
{
    PostureKnn knn;
    if (!knn.load(g_cfg.knn_path.c_str()))
        fprintf(stderr, "[WARN] KNN 样本库 %s 加载失败，坐姿判定将不可用\n",
                g_cfg.knn_path.c_str());
    while (g_run) {
        PressurePayload p;
        if (q_press.pop(p)) {
            int id = knn.classify(p.ch);
            g_state.posture_raw.store(id);
            static int cur_good = 0, cur_bad_ms = 0;
            if (id == POSTURE_NORMAL || id == POSTURE_UNKNOWN) {
                cur_bad_ms = 0;
            } else if (id > POSTURE_NORMAL) {
                cur_bad_ms += 200;  // 每0.2s一帧
            }
            g_state.posture_id.store(id);
            g_state.posture_bad_ms.store(cur_bad_ms);
            (void)cur_good;
        }
        usleep(5000);
    }
}

static void env_worker()
{
    EnvWorker w; w.configure(g_envcfg);
    while (g_run) {
        EnvPayload e;
        while (q_env.pop(e)) w.push_event(e.mq3, e.mq135, g_state);
        usleep(20000); // 20ms
    }
}

/* ------------------------- 融合与执行 ------------------------- */
static uint32_t now_ms()
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

static void fuse_and_act()
{
    // 周期 30ms：融合策略在引擎内部完成仲裁 + 同类告警 warn_gap_ms 静默去抖
    Decision d = g_fusion.decide(g_state, now_ms(), (uint32_t)g_cfg.warn_gap_ms);

    if (d.warn_id != WARN_NONE) {
        uint8_t out[64];
        size_t n = make_warn_frame(d.warn_id, d.motor_lvl, d.buzzer_lvl,
                                   d.duration_s, now_ms(), out);
        uart_stm.write_all(out, n);
        fprintf(stdout, "[ALARM] warn=%d lvl=%d/%d %ds score=%.2f\n",
                d.warn_id, d.motor_lvl, d.buzzer_lvl,
                d.duration_s, d.score);

        // 重度疲劳或酒精告警时上报云端事件
        if (d.warn_id >= WARN_FATIGUE_L2 || d.warn_id == WARN_ALCOHOL) {
            std::string js = "{\"warn\":" + std::to_string(d.warn_id) +
                             ",\"score\":" + std::to_string(d.score) + "}";
            if (g_nb.online()) g_nb.publish_json("device/aiot_drv/event", js);
        }
    }
}

/* ------------------------- 定时资源管理 ------------------------- */
static void vision_gate_tick()
{
    // 门控线程：RMSSD 异常或坐姿持续异常 -> 启动 VisionStub
    bool want = (g_state.rmssd_scale.load() < g_cfg.vision_gate_rmssd_scale) ||
                (g_state.posture_bad_ms.load() > g_cfg.vision_gate_posture_ms);
    if (want && !g_vision->running()) {
        g_vision->start();
        // 真实视觉管线：启动一次完整会话（采集+推理+指标累计），
        // 会话结束后自动下电，避免 NPU/相机常驻耗电
        if (auto *vp = dynamic_cast<VisionPipeline *>(g_vision)) {
            std::thread([vp]() {
                vp->run_session(g_cfg.vision_session_s, g_state,
                                1000 / std::max(1, g_cfg.vision_fps));
                vp->shutdown();                       // 会话结束恢复下电
                g_state.vision_active.store(false);
            }).detach();
        }
        return;
    }
    if (!want && g_vision->running()) {
        g_vision->shutdown();
        g_state.vision_active.store(false);
    }

    if (g_vision->running()) g_vision->tick_once(g_state);
}

static void mqtt_tick()
{
    g_nb.tick_heartbeat(g_state);
}

static void sync_tick()
{
    uint8_t out[16];
    size_t n = make_time_sync_frame(now_ms(), out);
    uart_stm.write_all(out, n);
}

int main(int argc, char *argv[])
{
    signal(SIGINT,  on_sigint);
    signal(SIGTERM, on_sigint);

    g_cfg.load(argc, argv);
    fprintf(stdout, "== 启动 RK3576 守护进程 ==\n");
    fprintf(stdout, "STM32 UART : %s (%d)\n", g_cfg.uart_stm_dev.c_str(), g_cfg.baud_stm);
    fprintf(stdout, "NB-IoT UART: %s (%d)  enabled=%d\n",
            g_cfg.uart_nb_dev.c_str(), g_cfg.baud_nb, g_cfg.nb_enable);
    fprintf(stdout, "KNN 样本库 : %s\n", g_cfg.knn_path.c_str());

    /* ---- UART 打开 ---- */
    if (!uart_stm.open_dev(g_cfg.uart_stm_dev.c_str(), g_cfg.baud_stm)) {
        fprintf(stderr, "无法打开 STM32 串口 %s\n", g_cfg.uart_stm_dev.c_str());
        return 1;
    }
    bool nb_ok = false;
    if (g_cfg.nb_enable) {
        nb_ok = uart_nb.open_dev(g_cfg.uart_nb_dev.c_str(), g_cfg.baud_nb);
        if (!nb_ok) fprintf(stderr, "[WARN] NB GPS 串口打开失败，MQTT停用\n");
    }

    /* ---- 视觉/Stub ---- */
    VisionPipeline *real_vision = nullptr;
    if (g_cfg.vision_stub) {
        g_vision = new VisionStub();          // 开发/PC 联调桩
    } else {
        real_vision = new VisionPipeline();
        if (real_vision->init(g_cfg.cam_rgb_dev.c_str(),
                              g_cfg.cam_nir_dev.c_str(),
                              g_cfg.rknn_model.c_str())) {
            g_vision = real_vision;
            fprintf(stdout, "视觉管线: 真实模式 RGB=%s NIR=%s model=%s\n",
                    g_cfg.cam_rgb_dev.c_str(), g_cfg.cam_nir_dev.c_str(),
                    g_cfg.rknn_model.c_str());
        } else {
            fprintf(stderr, "[WARN] 真实视觉管线初始化失败，回退到 Stub\n");
            delete real_vision;
            g_vision = new VisionStub();
        }
    }
    g_vision->shutdown();

    /* ---- Reactor ---- */
    Reactor rc;
    if (!rc.init(32)) return 2;

    rc.add(uart_stm.fd(), EPOLLIN, on_stm32_readable);
    if (nb_ok) {
        rc.add(uart_nb.fd(), EPOLLIN, [](int fd, uint32_t){
            uint8_t b[256]; ssize_t ignored = uart_nb.read_bytes(b, sizeof b);
            (void)ignored; // AT 回音通过 NbIotReporter::at() 同步读
        });
    }
    if (g_cfg.vision_enable)
        rc.add_timer_task(100, [](int, uint32_t e){ vision_gate_tick(); });
    rc.add_timer_task(g_cfg.fuse_ms,        [](int, uint32_t){ fuse_and_act(); });
    rc.add_timer_task(g_cfg.mqtt_ms,        [](int, uint32_t){ mqtt_tick(); });
    rc.add_timer_task(g_cfg.time_sync_ms,   [](int, uint32_t){ sync_tick(); });

    std::thread th_heart(heart_worker);
    std::thread th_post(posture_worker);
    std::thread th_env(env_worker);

    fprintf(stdout, "Reactor started.  fuse=%dms mqtt=%dms\n",
            g_cfg.fuse_ms, g_cfg.mqtt_ms);
    rc.run();

    g_run = false;
    th_heart.join(); th_post.join(); th_env.join();
    if (g_vision && g_vision->running()) g_vision->shutdown();
    printf("== 守护进程退出 ==\n");
    return 0;
}
