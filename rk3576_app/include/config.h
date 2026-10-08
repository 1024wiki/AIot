/**
 * @file config.h  命令行/配置参数结构
 */
#pragma once
#include <string>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <getopt.h>

namespace drv {

struct Config {
    /* ---- 设备路径 ---- */
    std::string uart_stm_dev{"/dev/ttyS1"};
    std::string uart_nb_dev {"/dev/ttyS2"};
    std::string knn_path    {"/usr/share/posture_knn.bin"};

    /* ---- 波特率 ---- */
    int baud_stm{115200};
    int baud_nb {115200};

    /* ---- 功能开关 ---- */
    bool nb_enable     {false};
    bool vision_stub   {true};   // 默认Stub；-V 显式指定。板端用 --real-vision 切真管线
    bool vision_enable {true};

    /* ---- 视觉管线设备与模型 ---- */
    std::string cam_rgb_dev {"/dev/video0"};   // RGB 流（YUV422）
    std::string cam_nir_dev {"/dev/video1"};   // NIR 流（GREY）
    std::string rknn_model  {"/usr/share/fused.rknn"};
    int    vision_session_s {60};              // 单次视觉会话时长（秒）
    int    vision_fps       {15};              // 视觉采样帧率上限（≥15FPS 足够捕捉眨眼）

    /* ---- 门控阈值 ---- */
    double vision_gate_rmssd_scale {0.75};
    int    vision_gate_posture_ms  {60*1000};

    /* ---- 节拍周期 ---- */
    int fuse_ms        {30};
    int mqtt_ms        {60000};
    int time_sync_ms   {30000};
    int warn_gap_ms    {30000};   // 同一等级告警去重间隔

    void load(int argc, char *argv[]) {
        static struct option o[] = {
            {"uart",     required_argument, 0, 'u'},
            {"nbuart",   required_argument, 0, 'n'},
            {"baud",     required_argument, 0, 'b'},
            {"nbbaud",   required_argument, 0, 'B'},
            {"knn",      required_argument, 0, 'k'},
            {"enable-nb",no_argument,       0, 'N'},
            {"vision-stub",no_argument,     0, 'V'},
            {"real-vision",no_argument,     0, 'R'},
            {"rgb",      required_argument, 0, '1'},
            {"nir",      required_argument, 0, '2'},
            {"model",    required_argument, 0, 'M'},
            {"novision", no_argument,       0, 'q'},
            {"fms",      required_argument, 0, 'f'},
            {"help",     no_argument,       0, 'h'},
            {0,0,0,0}
        };
        int c;
        while ((c = getopt_long(argc, argv, "u:n:b:B:k:NVqf:h", o, nullptr)) != -1) {
            switch (c) {
            case 'u': uart_stm_dev = optarg; break;
            case 'n': uart_nb_dev  = optarg; break;
            case 'b': baud_stm = atoi(optarg); break;
            case 'B': baud_nb  = atoi(optarg); break;
            case 'k': knn_path  = optarg; break;
            case 'N': nb_enable = true; break;
            case 'V': vision_stub = true; break;
            case 'R': vision_stub = false; break;
            case '1': cam_rgb_dev = optarg; break;
            case '2': cam_nir_dev = optarg; break;
            case 'M': rknn_model  = optarg; break;
            case 'q': vision_enable = false; break;
            case 'f': fuse_ms = atoi(optarg); break;
            case 'h': default:
                fprintf(stderr,
"用法: rk3576_app [opts]\n"
"  -u --uart STM32串口 (默认 /dev/ttyS1)\n"
"  -n --nbuart NB-IoT 串口 (默认 /dev/ttyS2)\n"
"  -b --baud STM32 波特率\n"
"  -B --nbbaud NB-IoT 波特率\n"
"  -k --knn 坐姿KNN样本文件\n"
"  -N --enable-nb 启用 NB-IoT/MQTT 上报\n"
"  -V --vision-stub 使用视觉Stub(默认)\n"
"  -R --real-vision 使用真实视觉管线(V4L2+RGA+rknn)\n"
"  -1 --rgb  RGB流设备节点\n"
"  -2 --nir  NIR流设备节点\n"
"  -M --model rknn模型路径\n"
"  -q --novision 禁止视觉门控\n"
"  -f --fms 融合决策周期 ms\n"
                );
                exit(0);
            }
        }
    }
};

} // namespace drv
