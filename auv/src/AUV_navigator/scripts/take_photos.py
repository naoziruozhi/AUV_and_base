#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
最简 OpenCV 视频流显示
Esc 键退出
"""

import cv2

def main():
    cap = cv2.VideoCapture(0)          # 0 = 默认摄像头
    if not cap.isOpened():
        raise RuntimeError("无法打开摄像头")

    try:
        while True:
            ok, frame = cap.read()      # 读取一帧
            if not ok:
                print("⚠️  读取帧失败")
                break

            cv2.imshow("Webcam", frame) # 显示
            if cv2.waitKey(1) == 27:    # Esc 键 ASCII 码 27
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
