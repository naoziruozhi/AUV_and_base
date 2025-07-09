#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四色灯实时检测（无形态学；红、蓝检测逻辑完全一致）

红 / 蓝： ≥3 块 → y 中位；2 块 → 最下；1 块 → 本块
          （蓝色候选仅取 **上半画面**，下半部分一律忽略）
绿    ： 连通域中过滤面积后取 (y 最大, 若并列 x 最大) 的质心
黄    ： 掩码里 y 最大那一行像素 → 质心

Esc 退出
"""

import cv2
import numpy as np

# ───────── 摄像头管线 ─────────
PIPE = ("v4l2src device=/dev/video0 ! "
        "image/jpeg,width=1280,height=720,framerate=30/1 ! "
        "jpegdec ! videoconvert ! appsink drop=1 max-buffers=1")
cap = cv2.VideoCapture(PIPE, cv2.CAP_GSTREAMER)
assert cap.isOpened(), "❌ 摄像头无法打开"

# ───────── HSV 阈值 ─────────
TUNE = {
    "red1":   {'LH':  0, 'LS':120, 'LV': 60, 'UH': 10, 'US':255, 'UV':255},
    "red2":   {'LH':160, 'LS':120, 'LV': 60, 'UH':179, 'US':255, 'UV':255},
    "green":  {'LH': 42, 'LS': 77, 'LV':189, 'UH': 78, 'US':255, 'UV':255},
    "yellow": {'LH': 10, 'LS': 63, 'LV':123, 'UH': 21, 'US':255, 'UV':255},
    "blue":   {'LH': 97, 'LS':105, 'LV':184, 'UH':120, 'US':255, 'UV':255},
}
COLORS = {
    "red":    ([TUNE["red1"], TUNE["red2"]], (  0,  0,255)),
    "green":  ([TUNE["green"]],              (  0,255,  0)),
    "yellow": ([TUNE["yellow"]],             (  0,255,255)),
    "blue":   ([TUNE["blue"]],               (255,  0,  0)),
}

# ───────── 阈值 / 常量 ─────────
MIN_PIX      = 30      # 红、蓝：整幅图的像素计数阈值
MIN_GREEN_A  = 180       # 绿：单连通域面积阈值
WIN_H, WIN_W = 360, 640

# ───────── 辅助函数 ─────────
def centroid(cnt):
    m = cv2.moments(cnt)
    a = m["m00"]
    return None if a == 0 else (int(m["m10"] / a), int(m["m01"] / a))

def bottom_right_centroid(stats, cents):
    """绿：取 (y 最大, 若并列 x 最大) 的质心"""
    idx = max(range(len(cents)), key=lambda i: (cents[i][1], cents[i][0]))
    return tuple(map(int, cents[idx]))

# ───────── 窗口 ─────────
cv2.namedWindow("view", cv2.WINDOW_NORMAL)
cv2.resizeWindow("view", WIN_W, WIN_H)

print("▶ 运行中，Esc 退出")

while True:
    ok, frame = cap.read()
    if not ok:
        continue

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]
    masks, centers = {}, {}

    # ─────── 遍历四色 ───────
    for name, (ranges, bgr) in COLORS.items():
        mask = np.zeros((h, w), np.uint8)
        for d in ranges:
            lo = np.array([d["LH"], d["LS"], d["LV"]])
            hi = np.array([d["UH"], d["US"], d["UV"]])
            mask |= cv2.inRange(hsv, lo, hi)
        masks[name] = mask.copy()

        # ---- 黄：y 最大行 ----
        if name == "yellow":
            ys, xs = np.where(mask > 0)
            if ys.size:
                y_max  = ys.max()
                x_mean = int(xs[ys == y_max].mean())
                centers[name] = (x_mean, int(y_max))
                cv2.circle(frame, centers[name], 5, bgr, -1)
            continue

        # ---- 红 / 蓝：与 Apriltag-PID 代码同一逻辑 ----
        if name in ("red", "blue"):
            # 像素数量阈值
            if np.count_nonzero(mask) < MIN_PIX:
                continue

            # 蓝：只保留上半画面的像素
            if name == "blue":
                mask[int(h * 0.5):, :] = 0
                if np.count_nonzero(mask) < MIN_PIX:
                    continue

            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cen = [centroid(c) for c in cnts if centroid(c)]
            if not cen:
                continue
            cen = sorted(cen, key=lambda p: p[1])       # 按 y 从小到大
            n = len(cen)
            centers[name] = cen[n // 2] if n >= 3 else cen[-1] if n == 2 else cen[0]

        # ---- 绿：连通域 + 几何规则 ----
        elif name == "green":
            nlab, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
            cand = [(i, stats[i, cv2.CC_STAT_AREA]) for i in range(1, nlab)
                    if stats[i, cv2.CC_STAT_AREA] >= MIN_GREEN_A]
            if not cand:
                continue
            cents = [tuple(map(int, centroids[i])) for i, _ in cand]
            centers[name] = bottom_right_centroid(stats, cents)

        # ---- 画点 & 标签 ----
        if name in centers:
            cv2.circle(frame, centers[name], 5, bgr, -1)
            cv2.putText(frame, name,
                        (centers[name][0] + 6, centers[name][1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 2)

    # ─────── 四点齐 → 画矩形 ───────
    if len(centers) == 4:
        pr, pg, py, pb = (centers[c] for c in ("red", "green", "yellow", "blue"))
        cv2.polylines(frame, [np.array([pr, pb, pg, py], np.int32)],
                      True, (255, 255, 255), 2)
        cx = (pr[0] + pg[0] + pb[0] + py[0]) // 4
        cy = (pr[1] + pg[1] + pb[1] + py[1]) // 4
        cv2.circle(frame, (cx, cy), 6, (255, 255, 255), -1)
        cv2.putText(frame, "center", (cx + 8, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # ─────── 显示 ───────
    cv2.imshow("view", frame)
    for cname in ("red", "green", "yellow", "blue"):
        cv2.imshow(f"mask_{cname}", masks[cname])

    if cv2.waitKey(1) == 27:   # Esc
        break

cap.release()
cv2.destroyAllWindows()
