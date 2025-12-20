#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/23
# @author:aiden
# 颜色跟踪(color tracking)
import os
import time
import rclpy
import signal
import threading
import sdk.pid as pid
from rclpy.node import Node
from kinematics import transform
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from kinematics_msgs.srv import SetRobotPose
from rclpy.executors import MultiThreadedExecutor
from interfaces.msg import ColorsInfo, ColorDetect
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.srv import SetColorDetectParam, SetString
from ros_robot_controller_msgs.msg import PWMServoState,SetPWMServoState
from kinematics.kinematics_control import set_pose_target
from servo_controller.bus_servo_control import set_servo_position

class ColorTrackNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.z_dis = 0.20
        self.y_dis = 500
        self.center = None
        self.running = True
        self.start = False
        self.name = name
        self.pid_z = pid.PID(0.00001, 0.0, 0.0)
        self.pid_y = pid.PID(0.02, 0.0, 0.0)
        
        self.machine_type = os.environ.get('MACHINE_TYPE')
        signal.signal(signal.SIGINT, self.shutdown)
        self.pwm_pub = self.create_publisher(SetPWMServoState,'ros_robot_controller/pwm_servo/set_state',10)
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 底盘控制(chassis control)

        self.create_subscription(ColorsInfo, '/color_detect/color_info', self.get_color_callback, 1)

        timer_cb_group = ReentrantCallbackGroup()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Kinematic init Start')
        self.x_init = transform.link3 + transform.tool_link
        self.client = self.create_client(Trigger, '/kinematics/init_finish')
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target', callback_group=timer_cb_group)
        self.kinematics_client.wait_for_service()


        self.get_logger().info('\033[1;32m%s\033[0m' % 'Kinematic is Finish')
        self.create_service(Trigger, '~/start', self.start_srv_callback) # 进入玩法(enter the game)
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group) # 退出玩法(exit the game)
        self.create_service(SetString, '~/set_color', self.set_color_srv_callback, callback_group=timer_cb_group) # 设置颜色(set color)
        self.set_color_client = self.create_client(SetColorDetectParam, '/color_detect/set_param', callback_group=timer_cb_group)
        self.set_color_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)
        
    def pwm_controller(self,position_data):
        pwm_list = []
        msg = SetPWMServoState()
        msg.duration = 0.2
        for i in range(len(position_data)):
            pos = PWMServoState()
            pos.id = [i+1]
            pos.position = [int(position_data[i])]
            pwm_list.append(pos)
        msg.state = pwm_list
        self.pwm_pub.publish(msg)

    def init_process(self):
        self.timer.cancel()
        self.init_action()
        if self.get_parameter('start').value:
            self.start_srv_callback(Trigger.Request(), Trigger.Response())
            request = SetString.Request()
            request.data = 'red'
            self.set_color_srv_callback(request, SetString.Response())
        threading.Thread(target=self.run, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False

    def init_action(self):
        self.pwm_controller([1500,1500])
        msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
        res = self.send_request(self.kinematics_client, msg)
        if res.pulse:
            servo_data = res.pulse
            set_servo_position(self.joints_pub, 1.5, ((10, 200), (5, 500), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, servo_data[0])))
            time.sleep(1.8)
        self.mecanum_pub.publish(Twist())

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def set_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_color")
        msg = SetColorDetectParam.Request()
        msg_red = ColorDetect()
        msg_red.color_name = request.data
        msg_red.detect_type = 'circle'
        msg.data = [msg_red]
        res = self.send_request(self.set_color_client, msg)
        if res.success:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'start_track_%s'%msg_red.color_name)
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'track_fail')
        response.success = True
        response.message = "set_color"
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start color track")
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop color track")
        self.start = False
        res = self.send_request(ColorDetect.Request())
        if res.success:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set color success')
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'set color fail')
        response.success = True
        response.message = "stop"
        return response

    def get_color_callback(self, msg):
        if msg.data != []:
            if msg.data[0].radius > 10:
                self.center = msg.data[0]
            else:
                self.center = None 
        else:
            self.center = None

    def run(self):
        while self.running:
            if self.center is not None and self.start:
                t1 = time.time()
                center = self.center

                self.pid_y.SetPoint = center.width/2 
                self.pid_y.update(self.center.x)
                self.y_dis += self.pid_y.output
                if self.y_dis < 200:
                    self.y_dis = 200
                if self.y_dis > 800:
                    self.y_dis = 80

                self.pid_z.SetPoint = center.height/2 
                self.pid_z.update(center.y)
                self.z_dis += self.pid_z.output
                if self.z_dis > 0.25:
                    self.z_dis = 0.25
                if self.z_dis < 0.16:
                    self.z_dis = 0.16
                msg = set_pose_target([self.x_init, 0.0, self.z_dis], 0.0, [-180.0, 180.0], 1.0)
                res = self.send_request(self.kinematics_client, msg)
                t2 = time.time()
                t = t2 - t1
                if t < 0.02:
                    time.sleep(0.02 - t)
                if res.pulse:
                    servo_data = res.pulse
                    set_servo_position(self.joints_pub, 0.02, ((10, 200), (5, 500), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, int(self.y_dis))))
                else:
                    
                    set_servo_position(self.joints_pub, 0.02, ((1, int(self.y_dis)), ))
            else:
                time.sleep(0.01)
        self.init_action()
        rclpy.shutdown()


def main():
    node = ColorTrackNode('color_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
