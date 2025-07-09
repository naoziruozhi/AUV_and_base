#!/usr/bin/env python3
import time
from pymavlink import mavutil
import threading

class MAVController:
    def __init__(self, port='/dev/ttyACM0', baud=115200, pwm_mid=1500):
        self.port = port
        self.baud = baud
        self.pwm_mid = pwm_mid
        self.mav = None
        self.pwm = [pwm_mid] * 8

    def connect(self):
        print("连接飞控...")
        self.mav = mavutil.mavlink_connection(self.port, baud=self.baud)
        self.mav.wait_heartbeat()
        print(f"连接成功：系统 ID = {self.mav.target_system}, 组件 ID = {self.mav.target_component}")
        self.set_mode("ALT_HOLD")

    def set_mode(self, mode):
        mode_map = {
        "MANUAL": 0,
        "STABILIZE": 2,
        "DEPTH_HOLD": 3,
        "ALT_HOLD": 7,
        "POS_HOLD": 16,
        "AUTO": 10,
        "CIRCLE": 17
        }
        mode_id = mode_map.get(mode.upper())
        if mode_id is None:
            raise ValueError(f"模式 {mode} 不支持")
    
        self.mav.set_mode(mode_id)
        print(f"设置模式为 {mode}")

    def arm(self):
        self.mav.mav.command_long_send(
        self.mav.target_system,
        self.mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

        self.mav.motors_armed_wait()
        print("电机已解锁")

    def disarm(self):
        print("发送锁定指令...")
        self.mav.mav.command_long_send(
        self.mav.target_system,
        self.mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,  # 0: disarm
        0, 0, 0, 0, 0, 0
        )
        # 等待锁定状态
        while self.mav.motors_armed():
            time.sleep(0.1)
        print("✅ 电机已锁定")

    def send_rc(self):
        print('开始发送RC')
        while(1) :
            if len(self.pwm) != 8:
                raise ValueError("PWM 数组长度必须为8")
            self.mav.mav.rc_channels_override_send(
                self.mav.target_system,
                self.mav.target_component,
                *self.pwm
            )
            time.sleep(0.05)
        print('终止发送')

    def stop(self):
        self.pwm = [self.pwm_mid] * 8
        
        print("归中（停止）指令已发送")


    def get_status(self):
 
        # 获取最新的心跳包
        hb = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb:
            mode_id = hb.custom_mode
            base_mode = hb.base_mode
            armed = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

            # 获取模式名称
            mode_map = self.mav.mode_mapping()
            mode_name = next((k for k, v in mode_map.items() if v == mode_id), "未知模式")

            print(f"飞控当前模式: {mode_name}")
            print(f"是否已解锁: {'是' if armed else '否'}")
        else:
            print("未收到心跳，无法获取飞控状态")

        # 获取姿态信息（俯仰、横滚、偏航角）
        att = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=3)
        if att:
            roll = att.roll * 57.3   # 弧度转角度
            pitch = att.pitch * 57.3
            yaw = att.yaw * 57.3
            print(f"姿态角 - 横滚: {roll:.1f}°，俯仰: {pitch:.1f}°，偏航: {yaw:.1f}°")
        else:
            print("未收到姿态信息")

if __name__ == "__main__":
    print("=== MAVController 测试开始 ===")
    controller = MAVController(port='/dev/ttyACM0', baud=115200)
    controller.connect()
    

    try:
        
        a = threading.Thread(target=controller.send_rc)
        a.start()
        time.sleep(2)
        controller.arm()
        time.sleep(2)

        controller.get_status()

        print("测试：发送前进指令（CH5 +200）")
        controller.pwm[4] = controller.pwm_mid + 200  # 通道5：前进
        #controller.send_rc(controller.pwm)
        controller.get_status()
        time.sleep(3)

        print("测试：发送左转指令（CH4 +100）")
        controller.pwm[3] = controller.pwm_mid + 100  # 通道4：左转
        #controller.send_rc(controller.pwm)
        controller.get_status()
        time.sleep(3)

        print("归中所有通道")
        controller.stop()
        time.sleep(2)

        controller.disarm()
    except Exception as e:
        print(f"❌ 出错：{e}")

    print("=== MAVController 测试结束 ===")