#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import sys, select, termios, tty
import time

class KeyboardTeleop:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)

        # 发布器
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.heartbeat_pub = rospy.Publisher('/user_heartbeat', Header, queue_size=1)

        self.last_heartbeat_time = time.time()

        # 控制映射
        self.move_bindings = {
            'w': (0, 0, +0.3, 0),   # 上浮
            's': (0, 0, -0.3, 0),   # 下潜
            'a': (0, 0, 0, +0.3),   # 左转
            'd': (0, 0, 0, -0.3),   # 右转
            '8': (+0.3, 0, 0, 0),   # 前进
            '2': (-0.3, 0, 0, 0),   # 后退
            '4': (0, +0.3, 0, 0),   # 左移
            '6': (0, -0.3, 0, 0),   # 右移
            'x': (0, 0, 0, 0),      # 停止
        }

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)  # 更快轮询
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def send_heartbeat(self):
        msg = Header()
        msg.stamp = rospy.Time.now()
        self.heartbeat_pub.publish(msg)

    def run(self):
        print("""
键盘控制机器人：
-------------------------------------------
W/S : 上浮/下潜         （linear.z ±0.3）
A/D : 左右转向         （angular.z ±0.3）
8/2 : 前进/后退         （linear.x ±0.3）
4/6 : 左右横移         （linear.y ±0.3）
X   : 停止（全部设为0）
自动定时发布 /user_heartbeat（0.5秒间隔）
Ctrl+C 退出
""")
        rate = rospy.Rate(20)  # 20Hz 循环，检测键盘+心跳更精确
        try:
            while not rospy.is_shutdown():
                key = self.get_key()

                # 处理键盘
                if key in self.move_bindings:
                    x, y, z, yaw = self.move_bindings[key]
                    twist = Twist()
                    twist.linear.x = x
                    twist.linear.y = y
                    twist.linear.z = z
                    twist.angular.z = yaw
                    self.cmd_pub.publish(twist)
                elif key == '\x03':
                    twist.linear.x = 0
                    twist.linear.y = 0
                    twist.linear.z = 0
                    twist.angular.z = 0
                    self.cmd_pub.publish(twist)
                    break

                # 每隔 0.5 秒发布一次心跳
                if time.time() - self.last_heartbeat_time > 0.5:
                    self.send_heartbeat()
                    self.last_heartbeat_time = time.time()

                rate.sleep()
        finally:
            # 停止机器人
            self.cmd_pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == "__main__":
    rospy.init_node('keyboard_teleop')
    teleop = KeyboardTeleop()
    teleop.run()
