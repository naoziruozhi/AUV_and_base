#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np

cv2.namedWindow("HSV Mask", cv2.WINDOW_NORMAL)
cv2.namedWindow("Pipe View", cv2.WINDOW_NORMAL)
cv2.namedWindow("Pipe Region", cv2.WINDOW_NORMAL)
cv2.namedWindow("Final Binary", cv2.WINDOW_NORMAL)  # 新增窗口：合成二值图

def nothing(x): pass

# HSV 滑块
cv2.createTrackbar("H_MIN", "HSV Mask", 0,   179, nothing)
cv2.createTrackbar("H_MAX", "HSV Mask", 84,  179, nothing)
cv2.createTrackbar("S_MIN", "HSV Mask", 0,   255, nothing)
cv2.createTrackbar("S_MAX", "HSV Mask", 255, 255, nothing)
cv2.createTrackbar("V_MIN", "HSV Mask", 154, 255, nothing)
cv2.createTrackbar("V_MAX", "HSV Mask", 255, 255, nothing)

# 摄像头 GStreamer 管线
PIPE = (
    "v4l2src device=/dev/video0 ! "
    "image/jpeg,width=1280,height=720,framerate=30/1 ! "
    "jpegdec ! videoconvert ! appsink drop=1 max-buffers=1"
)
cap = cv2.VideoCapture(PIPE, cv2.CAP_GSTREAMER)
assert cap.isOpened(), "❌ 摄像头无法打开"

# 窗口位置与大小
cv2.moveWindow("Pipe View", 100, 100)
cv2.resizeWindow("Pipe View", 640, 360)
cv2.moveWindow("HSV Mask", 800, 100)
cv2.resizeWindow("HSV Mask", 640, 360)
cv2.moveWindow("Pipe Region", 100, 500)
cv2.resizeWindow("Pipe Region", 640, 360)
cv2.moveWindow("Final Binary", 800, 500)
cv2.resizeWindow("Final Binary", 640, 360)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # ==== 图像增强 ====
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_eq = cv2.equalizeHist(gray)  # 均衡直方图提高对比度

    # ==== HSV 掩码 ====
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hmin = cv2.getTrackbarPos("H_MIN", "HSV Mask")
    hmax = cv2.getTrackbarPos("H_MAX", "HSV Mask")
    smin = cv2.getTrackbarPos("S_MIN", "HSV Mask")
    smax = cv2.getTrackbarPos("S_MAX", "HSV Mask")
    vmin = cv2.getTrackbarPos("V_MIN", "HSV Mask")
    vmax = cv2.getTrackbarPos("V_MAX", "HSV Mask")
    lower = np.array([hmin, smin, vmin])
    upper = np.array([hmax, smax, vmax])
    mask_hsv = cv2.inRange(hsv, lower, upper)
    mask_hsv = cv2.medianBlur(mask_hsv, 5)

    # ==== 合成黑白图 ====
    combined = cv2.bitwise_and(gray_eq, gray_eq, mask=mask_hsv)
    _, final_mask = cv2.threshold(combined, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Pipe Region 可视图
    region = np.zeros_like(frame)
    region[final_mask > 0] = (255, 255, 255)

    # ==== 寻找轮廓 ====
    contours, _ = cv2.findContours(final_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500:
            continue
        cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
        M = cv2.moments(cnt)
        if M["m00"] == 0: continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        centers.append((cx, cy))
        cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)

    if len(centers) >= 2:
        centers_sorted = sorted(centers, key=lambda p: p[1])
        for i in range(len(centers_sorted) - 1):
            cv2.line(frame, centers_sorted[i], centers_sorted[i+1], (0, 255, 255), 2)
        cv2.putText(frame, f"{len(centers_sorted)} segment centers",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # 显示
    cv2.imshow("Pipe View", frame)
    cv2.imshow("HSV Mask", mask_hsv)
    cv2.imshow("Pipe Region", region)
    cv2.imshow("Final Binary", final_mask)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
