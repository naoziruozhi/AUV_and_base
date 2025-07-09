#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CH6 左 / 右 摇摆测试（仅起始抬一下油门）

流程
1. CH3  → 1600 µs 上升 0.8 s
2. CH3  → 1500 µs 保持悬停（无额外升力）
   · CH6 → 1600 µs 右推 2 s
   · CH6 → 1400 µs 左推 2 s
3. 全通道复位 1500 µs

⚠ 请在安全环境下测试！
"""

import time
from pymavlink import mavutil

# ────────── 基本参数 ──────────
PORT, BAUD   = '/dev/ttyACM0', 115200
PWM_MID      = 1500
THR_UP       = PWM_MID + 100      # CH3 抬升
LAT_STEP     = 100                # CH6 ±100
T_UP         = 0.8                # 上升 0.8 s
T_STEP       = 2.0                # 每段 2 s
HZ           = 20                 # 20 Hz 发送

# ────────── 建立连接 ──────────
m = mavutil.mavlink_connection(PORT, baud=BAUD)
m.wait_heartbeat()
m.set_mode(m.mode_mapping()['ALT_HOLD'])
m.arducopter_arm(); m.motors_armed_wait()

def send(ch):
    m.mav.rc_channels_override_send(m.target_system,
                                    m.target_component, *ch)

def hold(ch, sec):
    dt, end = 1.0/HZ, time.time() + sec
    while time.time() < end:
        send(ch); time.sleep(dt)

base  = [PWM_MID]*8               # 所有 1500
up    = base.copy();  up[2]  = THR_UP         # CH3 抬升
right = base.copy(); right[5] = PWM_MID + LAT_STEP  # CH6 右推
left  = base.copy(); left[5]  = PWM_MID - LAT_STEP  # CH6 左推

try:
    print("STEP-1  上升 0.8 s")
    hold(up, T_UP)

    print("STEP-2  CH6 +100 µs 右推 2 s")
    hold(right, T_STEP)

    print("STEP-3  CH6 −100 µs 左推 2 s")
    hold(left, T_STEP)

finally:
    print("复位全部 1500 µs")
    for _ in range(40):
        send(base); time.sleep(1.0/HZ)
