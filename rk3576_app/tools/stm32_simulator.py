#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""STM32 传感器采集节点模拟器
用 socat 虚拟串口与 RK3576 守护进程联调：
  socat -d -d PTY,raw,echo=0,link=/tmp/vttyM &   <- as STM32 side
  socat -d -d PTY,raw,echo=0,link=/tmp/vttyS &   <- as RK3576 side
然后分别把模拟器和 rk3576_app 开到两个/dev/pts/X 上。
或者直接跑: python3 stm32_simulator.py /tmp/vttyM
"""
import sys, struct, time, os, signal, random, threading

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000: crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:            crc = (crc << 1) & 0xFFFF
    return crc

def f842(tick_ms):
    return bytes([0x55, 0xAA, 0x01]) + tick_ms.to_bytes(4, 'little')

def build_frame(typeb, payload, tick):
    hdr = bytes([0x55, 0xAA, 0x01, typeb]) + len(payload).to_bytes(2,'little') + int(tick).to_bytes(4,'little')
    return hdr + payload + crc16_ccitt(hdr[2:] + payload).to_bytes(2,'little')

def gen_heart(tick):
    # fifo_len=32, (ir,red) 模拟正弦+心跳脉冲
    pl = bytes([32])
    for i in range(32):
        ir = 50000 + int(5000 * ((i % 8) == 0)) + random.randint(-100,100)
        red = 60000 + random.randint(-80,80)
        pl += struct.pack("<II", ir, red)
    return build_frame(0x01, pl, tick)

def gen_press(tick, posture=0):
    # 4路FSR比例随姿态变化：正常均衡 0.25各
    base = [1000, 1000, 1000, 1000]
    if posture==1: base=[600,1400,1000,1000]   # 左倾，左侧低
    if posture==2: base=[1400,600,1000,1000]   # 右倾
    if posture==3: base=[900,900,1400,800]     # 前倾
    if posture==4: base=[900,900,900,1400]     # 后瘫
    v = [min(4095, x + random.randint(-30,30)) for x in base]
    return build_frame(0x02, struct.pack("<4H", *v), tick)

def gen_env(tick, alcohol=False, airbad=False):
    mq3b = 4095*0.6 if alcohol else 4095*0.1
    mq13b = 4095*0.7 if airbad else 4095*0.15
    return build_frame(0x03, struct.pack("<2H", int(mq3b), int(mq13b)), tick)

def gen_hb(tick):
    pl = bytes([85, 0]) + struct.pack("<I", tick & 0xFFFFFFFF)
    return build_frame(0x04, pl, tick)

def parse_down_frames(data: bytes):
    print("[STM32<<RUNTIME] ", data.hex(' '), file=sys.stderr)
    if len(data) >= 10 and data[0]==0x55 and data[1]==0xAA:
        t = data[3]
        warn_names = {0x81:"预警命令",0x82:"采样配置",0x83:"时间同步",0x84:"模式",0x85:"复位"}
        print("  ↳ 收到命令:", warn_names.get(t, hex(t)), "payload:", data[10:-2].hex(' '))

def main():
    dev = sys.argv[1] if len(sys.argv) > 1 else "/dev/pts/2"
    print(f"STM32 模拟器 -> {dev}")
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tick = 0
    stop = [False]
    signal.signal(signal.SIGINT, lambda s,f: stop.__setitem__(0, True))

    last50 = last200 = last1000 = last5s = time.time()
    cur_post = 0
    while not stop[0]:
        now = time.time()
        tick = int(now * 1000) & 0xFFFFFFFF

        # 50Hz 心率流
        if now - last50 > 0.02:
            last50 = now
            os.write(fd, gen_heart(tick))
        # 0.2Hz 压力（5s 一帧；这里改为1Hz方便联调）
        if now - last200 > 1.0:
            last200 = now
            # 每10s 切换一次坐姿，模拟不同的人
            cur_post = (int(now) // 10) % 5
            os.write(fd, gen_press(tick, cur_post))
        # 1Hz 空气/酒精
        if now - last1000 > 1.0:
            last1000 = now
            os.write(fd, gen_env(tick, alcohol=(int(now)//30)%2, airbad=(int(now)//45)%2))
        # 5s 节点心跳
        if now - last5s > 5.0:
            last5s = now
            os.write(fd, gen_hb(tick))

        # 读下行命令帧（非阻塞）
        try:
            d = os.read(fd, 512)
            if d: parse_down_frames(d)
        except BlockingIOError:
            pass

        time.sleep(0.005)
    os.close(fd)

if __name__ == "__main__":
    main()
