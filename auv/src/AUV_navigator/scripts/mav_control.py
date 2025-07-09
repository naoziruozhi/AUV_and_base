#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, cv2, time, threading, numpy as np
import socket
from pymavlink import mavutil
from pupil_apriltags import Detector
from ultralytics import YOLO   # ← 用于YOLO模型

os.environ["NO_AT_BRIDGE"] = "1"

# ─── 光通信基础 ───────────────────────────────────────────────
BASENAME = os.path.basename
SERVER_IP = "192.168.1.100"   # 基站静态 IP
SERVER_PORT = 5000            # 通信端口（上位机必须监听此端口）

FILE_PATH = "/mnt/ros1/src/AUV_navigator/sample_output/pipe_seg.mp4"  # 替换成实际路径
CHUNK_SIZE = 1024 * 1024  # 1MB 每块

# ─── MAVLink 基础 ───────────────────────────────────────────────
PORT, BAUD = '/dev/ttyACM0', 115200
MODE = 'ALT_HOLD'; SEND_HZ = 20
PWM_MID, PWM_MIN, PWM_MAX = 1500, 1100, 1900
THRO_START_BOOST, THRO_BOOST_DUR = 100, 1.2
clamp = lambda v: int(max(PWM_MIN, min(PWM_MAX, v)))

# ─── PID 参数 ──────────────────────────────────────────────────
Kp_x,Ki_x,Kd_x = 0.68, 0.009, 0.04; MAX_DX   = 80
Kp_y,Ki_y,Kd_y = 0.80, 0.004, 0.06; MAX_DY   = 80
Kp_f,Ki_f,Kd_f = 0.25, 0.000, 0.00; MAX_DF   = 40
Kp_yaw,Ki_yaw,Kd_yaw = 0.45, 0.007, 0.07; MAX_DYAW = 50
YAW_DB = 0.3

# ─── 滑动平均 ─────────────────────────────────────────────────
SMOOTH_ALPHA = 0.4

# ─── 灰度增强参数 ──────────────────────────────────────────────
USE_BINARIZE = False
BIN_THRESH   = 128
BIN_TYPE     = 0
BIN_CONTRAST = 1.00
BIN_BRIGHT   = 16

# ─── 阶段目标 ─────────────────────────────────────────────────
FRAME_OK_NEED = 3
STAGE1_TARGET, STAGE1_TOL = 12500, 1000
S1_RANGE = (STAGE1_TARGET-STAGE1_TOL, STAGE1_TARGET+STAGE1_TOL)
STAGE2_TARGET, STAGE2_TOL = 255000, 10000
S2_RANGE = (STAGE2_TARGET-STAGE2_TOL, STAGE2_TARGET+STAGE2_TOL)
POS_TOL = 50   # 位置死区

# ─── 状态机码表 ───────────────────────────────────────────────
ST_APPROACH, ST_HOLD, ST_FORWARD = range(3)
state = ST_APPROACH
ok_cnt = 0

# ─── 绘色 ─────────────────────────────────────────────────────
CLR = {"red":(0,0,255),"blue":(255,0,0),"green":(0,255,0),"yellow":(0,255,255)}

# ─── YOLO相关初始化 ───────────────────────────────────────────
yolo_model = YOLO('/mnt/ros1/src/yolo/model/yolo11n_rknn_model')
YOLO_COLOR_NAMES = {0: "red", 1: "yellow", 2: "blue", 3: "green"}

# ─── YOLO识别灯的函数 ─────────────────────────────────────────
def detect_rectangle(frame, hsv=None):
    """
    用yolo检测灯的位置，并返回四个灯中心点和偏航
    :param frame: BGR图像
    :return: (ok, cx, cy, yaw)
    """
    results = yolo_model(frame)
    boxes = results[0].boxes if hasattr(results[0], 'boxes') else []
    c = {}

    # 遍历检测到的box
    for box in boxes:
        cls_id = int(box.cls)
        if cls_id in YOLO_COLOR_NAMES:
            color = YOLO_COLOR_NAMES[cls_id]
            x1, y1, x2, y2 = map(int, box.xyxy)
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            c[color] = (cx, cy)
            cv2.circle(frame, (cx, cy), 4, CLR[color], -1)

    # 必须red, blue, green, yellow都检测到才算ok
    if all(k in c for k in ["red", "blue", "green", "yellow"]):
        pr, pb, pg, py = (c[k] for k in ("red", "blue", "green", "yellow"))
        pts = np.int32([pr, pb, pg, py])
        cv2.polylines(frame, [pts], True, (200,200,200), 1)
        cx = (pr[0]+pb[0]+pg[0]+py[0])//4
        cy = (pr[1]+pb[1]+pg[1]+py[1])//4
        cv2.circle(frame, (cx, cy), 5, (255,255,255), -1)
        # 方向（yaw）：保持原计算方式
        yaw = np.degrees(np.arctan2(pg[1]-pr[1], pg[0]-pr[0]))
        return True, cx, cy, yaw
    return False, None, None, None

# ─── 光通信函数封装  ─────────────────────────────────────────
def send_file(filepath):
    if not os.path.exists(filepath):
        print("❌ 文件不存在:", filepath)
        return

    file_size = os.path.getsize(filepath)
    filename = BASENAME(filepath)

    try:
        print(f"🔌 正在连接基站 {SERVER_IP}:{SERVER_PORT} ...")
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((SERVER_IP, SERVER_PORT))
        print("✅ 已连接，开始发送文件...")

        # 第一步：发送文件名和大小
        header = f"{filename}|{file_size}".encode()
        s.sendall(header.ljust(256, b'\0'))  # 固定长度 256 字节，方便接收端解析

        # 第二步：发送文件内容
        with open(filepath, 'rb') as f:
            sent = 0
            while chunk := f.read(CHUNK_SIZE):
                s.sendall(chunk)
                sent += len(chunk)
                percent = sent / file_size * 100
                print(f"📤 发送进度：{percent:.2f}%", end="\r")
                time.sleep(0.1)  # 模拟光通信稍慢的速率

        print("\n✅ 文件发送完成！")

    except Exception as e:
        print("❌ 发送失败:", e)
    finally:
        s.close()

# ─── MAVLink 封装 ──────────────────────────────────────────────
def mav_setup():
    m=mavutil.mavlink_connection(PORT,baud=BAUD)
    m.wait_heartbeat(); m.set_mode(m.mode_mapping()[MODE])
    m.arducopter_arm(); m.motors_armed_wait(); return m
def startup_boost(mav,pwm):
    tgt=clamp(PWM_MID+THRO_START_BOOST)
    end=time.time()+THRO_BOOST_DUR; dt=1/SEND_HZ
    while time.time()<end:
        pwm[2]=tgt
        mav.mav.rc_channels_override_send(mav.target_system,mav.target_component,*pwm)
        time.sleep(dt)
    pwm[2]=PWM_MID
def send_loop(mav,pwm,alive):
    dt=1/SEND_HZ
    while alive():
        mav.mav.rc_channels_override_send(mav.target_system,mav.target_component,*pwm)
        time.sleep(dt)

# ─── 主程序 ───────────────────────────────────────────────────
def main():
    global state, ok_cnt
    mav=mav_setup(); pwm=[PWM_MID]*8; startup_boost(mav,pwm)
    alive=True
    threading.Thread(target=lambda:send_loop(mav,pwm,lambda:alive),daemon=True).start()

    cap=cv2.VideoCapture(0); assert cap.isOpened()
    det=Detector("tag36h11")

    int_x=int_y=int_f=int_yaw=0.0
    prev_x=prev_y=prev_f=prev_yaw=0.0
    fdx=fdy=feyaw=feA=0.0; first_sample=True
    dt=1/SEND_HZ
    hold_dx=hold_dy=hold_yaw=0.0

    push_log=[]                # ← 记录 FORWARD 阶段 pwm[4] 偏移

    try:
        while True:
            ok,frame=cap.read(); h,w=frame.shape[:2]
            if not ok: continue
            cx_f,cy_f=w//2,h//2
            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

            # 灰度增强
            if USE_BINARIZE:
                adjusted = cv2.convertScaleAbs(gray, alpha=BIN_CONTRAST, beta=BIN_BRIGHT)
                _, gray_tag = cv2.threshold(adjusted, BIN_THRESH, 255, BIN_TYPE)
            else:
                gray_tag = cv2.convertScaleAbs(gray, alpha=BIN_CONTRAST, beta=BIN_BRIGHT)

            # ─── Tag 检测 ──────────────────────────────────
            dets=det.detect(gray_tag, False)
            tag=next((d for d in dets if d.tag_id==0), None) if dets else None

            if tag is not None:
                for i in range(4):
                    p1,p2=tag.corners[i].astype(int),tag.corners[(i+1)%4].astype(int)
                    cv2.line(frame,tuple(p1),tuple(p2),(150,150,150),1)
                cx_t,cy_t=tag.center.astype(int)
                cv2.circle(frame,(cx_t,cy_t),5,(150,150,150),-1)
                dx_raw,dy_raw=cx_t-cx_f,cy_f-cy_t
                area=cv2.contourArea(tag.corners.astype(int))
                vtop,vbot=tag.corners[1]-tag.corners[0],tag.corners[2]-tag.corners[3]
                eyaw_raw=(np.degrees(np.arctan2(vtop[1],vtop[0]))-
                          np.degrees(np.arctan2(vbot[1],vbot[0])))/2
                hold_dx,hold_dy,hold_yaw=dx_raw,dy_raw,eyaw_raw
            else:
                # ★ 已进入 FORWARD → 执行撤退序列并退出
                if state == ST_FORWARD:
                    print("### Tag0 丢失，开始撤退序列")

                    # 1) 下降 0.3 s
                    pwm[:] = [PWM_MID]*8
                    pwm[2] = clamp(PWM_MID - 150)    # CH3 向下
                    pwm[3] = clamp(PWM_MID + 40)
                    time.sleep(0.5)
                    pwm[:] = [PWM_MID]*8

                    # 2) 进行数据上传 额外等待3分钟进行无线充电
                    send_file(FILE_PATH)
                    time.sleep(180)
                    
                    # 3) 垂直上升 0.8 s
                    pwm[2] = clamp(PWM_MID + 150)
                    time.sleep(0.4)
                    pwm[2] = PWM_MID

                    # 4) 时间反放撤退
                    dt_back = 1 / SEND_HZ
                    for offset in reversed(push_log):
                        pwm[4] = clamp(PWM_MID - offset)   # 反向同幅
                        time.sleep(dt_back)
                    pwm[4] = PWM_MID

                    # 5) 就地解锁并退出
                    mav.arducopter_disarm()
                    break

                ok_rect,cx,cy,eyaw_raw=detect_rectangle(frame,hsv)
                if ok_rect:
                    dx_raw,dy_raw=cx-cx_f,cy_f-cy
                else:
                    dx_raw,dy_raw,eyaw_raw=hold_dx,hold_dy,hold_yaw
                area = STAGE1_TARGET
                state = ST_APPROACH

            # ─── 指数滑动平均 ──────────────────────────────
            if first_sample:
                fdx,fdy,feyaw,feA = dx_raw,dy_raw,eyaw_raw,STAGE1_TARGET-area; first_sample=False
            else:
                fdx   = SMOOTH_ALPHA*fdx   + (1-SMOOTH_ALPHA)*dx_raw
                fdy   = SMOOTH_ALPHA*fdy   + (1-SMOOTH_ALPHA)*dy_raw
                feyaw = SMOOTH_ALPHA*feyaw + (1-SMOOTH_ALPHA)*eyaw_raw
                feA   = SMOOTH_ALPHA*feA   + (1-SMOOTH_ALPHA)*(STAGE1_TARGET-area)

            dx,dy,eyaw = fdx,fdy,feyaw
            eA          = feA

            # ─── 状态机 ──────────────────────────────────
            if tag is not None:
                # ① APPROACH
                if state == ST_APPROACH:
                    int_f += eA*dt
                    out_f = np.clip(Kp_f*eA+Ki_f*int_f+Kd_f*(eA-prev_f)/dt,-MAX_DF,MAX_DF); prev_f=eA
                    pwm[4]=clamp(PWM_MID+int(out_f))
                    cond_area=S1_RANGE[0]<=area<=S1_RANGE[1]
                    cond_pos =abs(dx)<=POS_TOL and abs(dy)<=POS_TOL
                    ok_cnt   = ok_cnt+1 if (cond_area and cond_pos) else 0
                    if ok_cnt>=FRAME_OK_NEED:
                        ok_cnt=0; state=ST_HOLD; int_f=0; prev_f=0
                        print(">> APPROACH → HOLD")
                # ② HOLD
                elif state == ST_HOLD:
                    pwm[4]=PWM_MID
                    cond_hold=(abs(dx)<=POS_TOL and abs(dy)<=POS_TOL and
                               abs(eyaw)<=YAW_DB and S1_RANGE[0]<=area<=S1_RANGE[1])
                    ok_cnt = ok_cnt+1 if cond_hold else 0
                    if ok_cnt>=FRAME_OK_NEED:
                        ok_cnt=0; state=ST_FORWARD; int_f=0; prev_f=0
                        print(">> HOLD → FORWARD")
                    if not(S1_RANGE[0]<=area<=S1_RANGE[1]):
                        ok_cnt=0; state=ST_APPROACH; print("<< HOLD 失败 → APPROACH")
                # ③ FORWARD
                elif state == ST_FORWARD:
                    eA2 = STAGE2_TARGET - area
                    int_f += eA2*dt
                    out_f = np.clip(Kp_f*eA2+Ki_f*int_f+Kd_f*(eA2-prev_f)/dt,
                                    -MAX_DF,MAX_DF); prev_f=eA2
                    out_f = np.clip(out_f, 0, 20)
                    pwm[4] = clamp(PWM_MID + int(out_f))

                    # 记录推进量（偏移值）
                    push_log.append(pwm[4] - PWM_MID)

                    cond_area2=S2_RANGE[0]<=area<=S2_RANGE[1]
                    cond_pos2 =abs(dx)<=POS_TOL and abs(dy)<=POS_TOL
                    cond_yaw2 =abs(eyaw)<=YAW_DB
                    ok_cnt    = ok_cnt+1 if (cond_area2 and cond_pos2 and cond_yaw2) else 0
                    if ok_cnt>=FRAME_OK_NEED:
                        print("### FORWARD 阶段完成，脚本退出")
                        break

            # ─── Roll / Pitch / Yaw PID ────────────────────
            int_x += dx*dt
            out_x  = np.clip(Kp_x*dx+Ki_x*int_x+Kd_x*(dx-prev_x)/dt,-MAX_DX,MAX_DX); prev_x=dx
            pwm[5] = clamp(PWM_MID+int(out_x))

            int_y += dy*dt
            out_y  = np.clip(Kp_y*dy+Ki_y*int_y+Kd_y*(dy-prev_y)/dt,-MAX_DY,MAX_DY); prev_y=dy
            pwm[2] = clamp(PWM_MID+int(out_y))

            if abs(eyaw)<YAW_DB: eyaw=0
            int_yaw += eyaw*dt
            out_yaw = np.clip(Kp_yaw*eyaw+Ki_yaw*int_yaw+Kd_yaw*(eyaw-prev_yaw)/dt,
                              -MAX_DYAW,MAX_DYAW); prev_yaw=eyaw
            pwm[3] = clamp(PWM_MID+int(out_yaw))

            # ─── 发送串口 & 画面 ───────────────────────────
            cv2.putText(frame,
                        f"dx:{dx:.0f} dy:{dy:.0f} yaw:{eyaw:.1f} st:{state}",
                        (10,25),0,0.6,(0,255,255),2)
            cv2.imshow("view",frame)
            if cv2.waitKey(1)==27: break
    finally:
        alive=False; pwm[:]=[PWM_MID]*8
        for _ in range(5):
            mav.mav.rc_channels_override_send(mav.target_system,
                                              mav.target_component,*pwm)
            time.sleep(0.1)
        cap.release(); cv2.destroyAllWindows()

if __name__ == "__main__":
    while True:
        time.sleep(100)
        main()
