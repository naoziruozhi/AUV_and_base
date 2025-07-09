#!/usr/bin/env python3

import cv2
from ultralytics import YOLO

# ✅ 加载你的 RKNN YOLO 模型
model = YOLO('/mnt/ros1/src/yolo/model/yolo11n_rknn_model')

# ✅ 打开摄像头（默认是 /dev/video0）
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ 无法打开摄像头")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ 图像读取失败")
        break

    # ✅ 推理：不保存、不打印、直接画框
    results = model.predict(source=frame, stream=False, show=False, save=False, verbose=False)

    # ✅ 画框：获取一个新图像
    annotated_frame = results[0].plot()

    # ✅ 显示图像窗口
    cv2.imshow('YOLO Detection', annotated_frame)

    # 按下 q 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
