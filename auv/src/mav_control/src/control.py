import time
from pymavlink import mavutil

class MAVController:
    def __init__(self, port='/dev/ttyACM0', baud=115200, pwm_mid=1500):
        self.port = port
        self.baud = baud
        self.pwm_mid = pwm_mid
        self.mav = None
        self.pwm = [pwm_mid] * 8

    def connect(self):
        print("连接 ArduSub 飞控...")
        self.mav = mavutil.mavlink_connection(self.port, baud=self.baud)
        self.mav.wait_heartbeat()
        print(f"连接成功：系统 ID = {self.mav.target_system}, 组件 ID = {self.mav.target_component}")

        self.set_mode("ALT_HOLD")  # ArduSub 支持的模式有 MANUAL, STABILIZE, DEPTH, HOLD 等
        print("飞控已就绪")

    def set_mode(self, mode):
        mode_id = self.mav.mode_mapping().get(mode)
        if mode_id is None:
            raise ValueError(f"模式 {mode} 不支持")
        self.mav.set_mode(mode_id)
        print(f"设置飞控模式为 {mode}")

    def arm(self):
        print("解锁电机...")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        self.mav.motors_armed_wait()
        print("✅ 电机已解锁")

    def disarm(self):
        print("发送锁定指令...")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        while self.mav.motors_armed():
            time.sleep(0.1)
        print("✅ 电机已锁定")

    def send_rc(self, pwm):
        if len(pwm) != 8:
            raise ValueError("PWM 数组长度必须为 8")
        self.mav.mav.rc_channels_override_send(
            self.mav.target_system,
            self.mav.target_component,
            *pwm
        )

    def stop(self):
        self.pwm = [self.pwm_mid] * 8
        for _ in range(5):
            self.send_rc(self.pwm)
            time.sleep(0.1)
        print("已发送归中（停止）指令")

    # 推进控制（前后） - 通道 1
    def move_forward(self, power=100):
        self.pwm[0] = self.pwm_mid + power
        print(f"{'前进' if power > 0 else '后退'}")
        self.send_rc(self.pwm)

    # 垂直控制（上下） - 通道 3
    def move_vertical(self, power=100):
        self.pwm[2] = self.pwm_mid + power
        print(f"{'上浮' if power > 0 else '下潜'}")
        self.send_rc(self.pwm)

    # 偏航控制（左/右转） - 通道 4
    def rotate_yaw(self, power=100):
        self.pwm[3] = self.pwm_mid + power
        print(f"{'右转' if power > 0 else '左转'}")
        self.send_rc(self.pwm)

    def get_status(self):
        hb = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if hb:
            mode_id = hb.custom_mode
            base_mode = hb.base_mode
            armed = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0

            mode_map = self.mav.mode_mapping()
            mode_name = next((k for k, v in mode_map.items() if v == mode_id), "未知模式")

            print(f"飞控当前模式: {mode_name}")
            print(f"是否已解锁: {'是' if armed else '否'}")
        else:
            print("⚠️ 未收到心跳包")

        att = self.mav.recv_match(type='ATTITUDE', blocking=True, timeout=3)
        if att:
            roll = att.roll * 57.3
            pitch = att.pitch * 57.3
            yaw = att.yaw * 57.3
            print(f"姿态角 - 横滚: {roll:.1f}°，俯仰: {pitch:.1f}°，偏航: {yaw:.1f}°")
        else:
            print("⚠️ 未收到姿态信息")


if __name__ == "__main__":
    controller = MAVController()
    controller.connect()
    controller.arm()
    controller.get_status()
    time.sleep(2)
    controller.get_status()

    controller.move_forward(power=200)
    controller.get_status()
    time.sleep(1)
    controller.move_forward(power=-200)
    controller.get_status()
    time.sleep(1)
    controller.rotate_yaw(power=150)
    controller.get_status()
    time.sleep(1)
    controller.rotate_yaw(power=-150)
    controller.get_status()
    time.sleep(1)
    controller.move_vertical(power=200)
    controller.get_status()
    time.sleep(1)
    controller.move_vertical(power=-200)
    controller.get_status()
    time.sleep(1)

    controller.stop()
    controller.disarm()
