#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mini HSV-tuner   （只调阈值，不做任何图像增强）

窗口：
  ctrl     260×180   —— 6 个滑块 (LH LS LV UH US UV)
  raw      320×180   —— 原始帧缩略图
  mask     320×180   —— HSV 掩码

操作：
  ↑ / ↓ 滑块实时更新掩码
  S  —— 打印当前 6 个 HSV 数值（方便记录）
  Esc —— 退出
"""

import cv2, numpy as np, textwrap

# ────────── 摄像头 GStreamer 管线 ──────────
PIPE = ("v4l2src device=/dev/video0 ! "
        "image/jpeg,width=1280,height=720,framerate=30/1 ! "
        "jpegdec ! videoconvert ! appsink drop=1 max-buffers=1")
cap = cv2.VideoCapture(PIPE, cv2.CAP_GSTREAMER)
assert cap.isOpened(), "❌ 摄像头无法打开"

# ────────── 控制滑块窗口 (超小) ──────────
WIN = "ctrl"
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, 260, 180)
def add(name, init, mx): cv2.createTrackbar(name, WIN, init, mx, lambda _:None)

# 默认阈值——起步抓黄色，可随意调整
for n,v,mx in [("LH",25,179),("LS",60,255),("LV",60,255),
               ("UH",45,179),("US",255,255),("UV",255,255)]:
    add(n,v,mx)

resize=lambda im: cv2.resize(im,(320,180))

def dump_params():
    d={k:cv2.getTrackbarPos(k,WIN) for k in ("LH","LS","LV","UH","US","UV")}
    print("—— HSV 当前值 ——")
    print(textwrap.indent(str(d),"  "))
    print("——————————")

print("▶ 开始调 HSV：S 保存参数，Esc 退出")

while True:
    ok, frame = cap.read()
    if not ok: continue

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lo = np.array([cv2.getTrackbarPos(k,WIN) for k in ("LH","LS","LV")])
    hi = np.array([cv2.getTrackbarPos(k,WIN) for k in ("UH","US","UV")])

    mask = cv2.inRange(hsv, lo, hi)

    cv2.imshow("raw",  resize(frame))
    cv2.imshow("mask", resize(cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)))

    k=cv2.waitKey(1)&0xFF
    if k==27: break           # Esc
    if k in (ord('s'),ord('S')):
        dump_params()

cap.release(); cv2.destroyAllWindows()
