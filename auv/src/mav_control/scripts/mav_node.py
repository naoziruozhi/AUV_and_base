#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import threading
import time
import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from mav_controller import MAVController


class MAVRosNode:
    def __init__(self):
        rospy.init_node("mav_control_node", anonymous=True)

        # 初始化 MAVController
        self.controller = MAVController()
        self.controller.connect()  # 连接飞控 + 设置模式（不 arm）
        self.is_armed = False

        # 状态记录
        self.last_cmd_time = time.time()
        self.last_user_heartbeat = time.time()

        # 订阅控制话题
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("/user_heartbeat", Header, self.user_heartbeat_callback)

        rospy.on_shutdown(self.shutdown)

        # 启动线程：用户心跳检测
        self.heartbeat_thread = threading.Thread(target=self.monitor_user_heartbeat)
        self.keep_send_rc = threading.Thread(target=self.controller.send_rc)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()
        self.keep_send_rc.start()

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_z = msg.linear.z
        angular_z = msg.angular.z

        forward_pwm = int(linear_x * 200)
        vertical_pwm = int(linear_z * 100)
        yaw_pwm = int(angular_z * 100)

        self.controller.pwm[4] = self.controller.pwm_mid + forward_pwm  # 前进/后退
        self.controller.pwm[2] = self.controller.pwm_mid + vertical_pwm  # 上浮/下潜
        self.controller.pwm[3] = self.controller.pwm_mid + yaw_pwm      # 左/右转

        rospy.loginfo("forward_pwm...%d",forward_pwm)
        rospy.loginfo("vertical_pwm...%d",vertical_pwm)
        rospy.loginfo("yaw_pwm...%d",yaw_pwm)

        #self.controller.send_rc(self.controller.pwm)
        self.last_cmd_time = time.time()

    def user_heartbeat_callback(self, msg):
        self.last_user_heartbeat = time.time()
        rospy.logdebug("收到用户心跳")
        #self.arm_motors()
        if self.is_armed == False:
            rospy.loginfo("✅ 收到心跳，解锁电机")
            self.arm_motors()
            self.is_armed = True
            self.controller.get_status()

    def monitor_user_heartbeat(self):
        timeout_sec = rospy.get_param("~user_heartbeat_timeout", 2.0)
        rate = rospy.Rate(1)  # 0.5s 检查一次

        

        time.sleep(2)

        while not rospy.is_shutdown():
            if (time.time() - self.last_user_heartbeat > timeout_sec) and self.is_armed == True:
                rospy.logwarn("⚠️ 用户心跳超时，执行锁定！")
                self.controller.stop()
                self.controller.disarm()
                self.is_armed = False
            
            rate.sleep()

    def arm_motors(self):
        rospy.loginfo("解锁电机...")
        self.controller.arm()
        rospy.loginfo("解锁成功...")

    def shutdown(self):
        rospy.loginfo("节点关闭，归中并锁定电机")
        self.controller.stop()
        self.controller.disarm()

    def run(self):
        rate = rospy.Rate(10)
        timeout = rospy.get_param("~cmd_vel_timeout", 1.5)

        while not rospy.is_shutdown():
            #if time.time() - self.last_cmd_time > timeout:
                #self.controller.stop()
            rate.sleep()

if __name__ == "__main__":
    node = MAVRosNode()
    node.run()

