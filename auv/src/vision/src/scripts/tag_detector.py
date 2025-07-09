#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AprilTag-0 监测（极简处理链：GRAY → CONTRAST/BRIGHT → THRESH）
增加逻辑：测量 tag0 的几何面积（像素²）并在窗口中显示，窗口尺寸缩小为 640×360
"""
import os, sys, cv2, numpy as np
from pupil_apriltags import Detector

os.environ["NO_AT_BRIDGE"] = "1"

# ── 摄像头 GStreamer 管线 ──────────────────────────────────────
PIPE = (
    "v4l2src device=/dev/video0 ! "
    "image/jpeg,width=1280,height=720,framerate=30/1 ! "
    "jpegdec ! videoconvert ! appsink drop=1 max-buffers=1"
)
cap = cv2.VideoCapture(PIPE, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    sys.exit("❌ 摄像头打开失败")

# ── AprilTag 检测器（最大容错） ────────────────────────────────
detector = Detector(
    families="tag36h11",
    nthreads=2,
    quad_decimate=1.0,
    decode_sharpening=0.4,
    refine_edges=0        # 禁用边缘优化
)

# ── 与 Binary Preview 一致的固定参数 ──────────────────────────
THRESH_VAL   = 139                # cv2.threshold 阈值
THRESH_TYPE  = cv2.THRESH_BINARY  # 与滑块 TYPE=0 相同
CONTRAST_A   = 92 / 100.0         # alpha
BRIGHT_B     = 0                  # beta

# 创建可调整大小的窗口，然后缩小到 640×360
cv2.namedWindow("Binary View",    cv2.WINDOW_NORMAL)
cv2.namedWindow("Detection View", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Binary View", 640, 360)
cv2.resizeWindow("Detection View", 640, 360)

print("📸 极简模式：只输出 tag_id=0 | 按 ESC 退出")
while True:
    ok, frame = cap.read()
    if not ok:
        continue

    # 1) 灰度
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 2) 对比度/亮度调整
    adjusted = cv2.convertScaleAbs(gray, alpha=CONTRAST_A, beta=BRIGHT_B)
    # 3) 二值化
    _, binary = cv2.threshold(adjusted, THRESH_VAL, 255, THRESH_TYPE)
    # 4) AprilTag 检测
    detections = detector.detect(binary, estimate_tag_pose=False)

    found = False
    for d in detections:
        if d.tag_id == 0:
            pts = d.corners.astype(int)
            cx, cy = d.center.astype(int)
            # 计算几何面积（像素²）
            area = int(cv2.contourArea(pts))

            # 在 Detection View 上显示
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(frame,
                        f"Tag0 area={area} px^2",
                        (cx + 6, cy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 0), 2)
            # 在 Binary View 上也显示面积
            cv2.putText(binary,
                        f"Area={area} px^2",
                        (cx + 6, cy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        255, 1)
            # 控制台打印
            print(f"Tag0 geometric area: {area} pixel^2")
            found = True
            break  # 只关心第一个 tag0

    # 状态提示
    cv2.putText(frame,
                "Tag0: FOUND" if found else "Tag0: NONE",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 0) if found else (0, 0, 255), 2)

    # 显示
    cv2.imshow("Binary View", binary)
    cv2.imshow("Detection View", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
