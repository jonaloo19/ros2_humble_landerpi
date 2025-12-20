#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/18
# @author:aiden
# 追踪拾取（Tracking and picking）
import os
import ast
import cv2
import time
import math
import queue
import rclpy
import threading
import numpy as np
from sdk import common
from sdk.pid import PID
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from interfaces.msg import Pose2D
from geometry_msgs.msg import Twist
from interfaces.srv import SetPose2D
from rclpy.parameter import Parameter
if os.path.exists('/dev/ring_mic'):
    from xf_mic_asr_offline import voice_play
from rcl_interfaces.msg import SetParametersResult
from servo_controller_msgs.msg import ServosPosition
from rcl_interfaces.srv import SetParametersAtomically
from servo_controller.bus_servo_control import set_servo_position
from servo_controller.action_group_controller import ActionGroupController

class AutomaticPickNode(Node):
    
    config_path = '/home/ubuntu/ros2_ws/src/example/config/automatic_pick_roi.yaml'

    lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        # 颜色识别
        self.image_proc_size = (320, 240)

        self.running = True
        self.detect_count = 0
        self.start_pick = False
        self.start_place = False
        self.target_color = ""
        self.linear_base_speed = 0.007
        self.angular_base_speed = 0.03

        self.yaw_pid = PID(P=0.015, I=0, D=0.000)
        # self.linear_pid = PID(P=0, I=0, D=0)
        self.linear_pid = PID(P=0.0015, I=0, D=0)
        # self.angular_pid = PID(P=0, I=0, D=0)
        self.angular_pid = PID(P=0.003, I=0, D=0)

        self.linear_speed = 0
        self.angular_speed = 0
        self.yaw_angle = 90

        self.pick_stop_x = 320
        self.pick_stop_y = 388
        self.place_stop_x = 320
        self.place_stop_y = 388
        self.stop = True

        self.d_y = 10
        self.d_x = 10

        self.pick = False
        self.place = False

        self.broadcast_status = ''
        self.status = "approach"
        self.count_stop = 0
        self.count_turn = 0

        self.declare_parameter('status', 'start')
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        
        self.start = self.get_parameter('start').value
        self.broadcast = self.get_parameter('broadcast').value
        self.enable_display = self.get_parameter('enable_display').value
        self.place_position = self.get_parameter('place_position').value
        self.place_without_color = self.get_parameter('place_without_color').value
        self.debug = self.get_parameter('debug').value
        
        self.language = os.environ['ASR_LANGUAGE']
        self.machine_type = os.environ['MACHINE_TYPE']
        self.camera_type = os.environ['DEPTH_CAMERA_TYPE']
        

        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.image_pub = self.create_publisher(Image, '~/image_result', 1)

        image_topic = "/ascamera/camera_publisher/rgb0/image"
        self.create_subscription(Image, image_topic, self.image_callback, 1)
        
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.parameter_client = self.create_client(SetParametersAtomically, name + '/set_parameters_atomically')

        self.create_service(Trigger, '~/pick', self.start_pick_callback)
        self.create_service(Trigger, '~/place', self.start_place_callback) 

        self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/software/arm_pc/ActionGroups')
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.transport_client = self.create_client(SetPose2D, '/navigation_transport/place')

        self.mecanum_pub.publish(Twist())
        self.controller.run_action('navigation_pick_init')
        # set_servo_position(self.joints_pub, 2, ((1, 500), (2, 650), (3, 60), (4, 185), (5, 500), (10, 200)))
    
        time.sleep(2)
        if self.debug:
            self.controller.run_action('navigation_pick_debug')
            time.sleep(5)

            self.controller.run_action('navigation_pick_init')
            # set_servo_position(self.joints_pub, 2, ((1, 500), (2, 650), (3, 60), (4, 185), (5, 500), (10, 200)))
            time.sleep(2)
            msg = Trigger.Request()
            self.start_pick_callback(msg, Trigger.Response())

        threading.Thread(target=self.action_thread, daemon=True).start()
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'status' and param.type_ == Parameter.Type.STRING:
                self.get_logger().info('status parameter change to %s' % param.value)
        return SetParametersResult(successful=True)

    def set_parameter(self, client, name, value):
        # Parameter.Type.INTEGER、Parameter.Type.DOUBLE、Parameter.Type.STRING、Parameter.Type.BOOLEAN、Parameter.Type.BYTE_ARRA
        req = SetParametersAtomically.Request()
        req.parameters = [Parameter(name, Parameter.Type.STRING, value).to_parameter_msg()]
        client.call_async(req)


    def start_pick_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start pick")
        self.set_parameter(self.parameter_client, 'status', 'start_pick')
        
        self.linear_speed = 0
        self.angular_speed = 0
        self.yaw_angle = 90

        param = self.get_parameter('pick_stop_pixel_coordinate').value
        self.get_logger().info('\033[1;32mget pick stop pixel coordinate: %s\033[0m' % str(param))
        self.pick_stop_x = param[0]
        self.pick_stop_y = param[1]
        self.get_logger().info('\033[1;32mget [self.pick_stop_x] %s\033[0m' % str(self.pick_stop_x))
        self.get_logger().info('\033[1;32mget [self.pick_stop_y] %s\033[0m' % str(self.pick_stop_y))
        self.stop = True

        self.d_y = 10
        self.d_x = 10

        self.pick = False
        self.place = False

        self.status = "approach"
        self.target_color = 'red'
        self.broadcast_status = 'find_target'
        self.count_stop = 0
        self.count_turn = 0

        self.linear_pid.clear()
        self.angular_pid.clear()
        self.start_pick = True

        response.success = True
        response.message = "start_pick"
        return response 


    def start_place_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start place")
        self.set_parameter(self.parameter_client, 'status', 'start_place')

        self.linear_speed = 0
        self.angular_speed = 0
        self.d_y = 30
        self.d_x = 30
        
        param = self.get_parameter('place_stop_pixel_coordinate').value
        self.get_logger().info('\033[1;32mget place stop pixel coordinate: %s\033[0m' % str(param))
        self.place_stop_x = param[0]
        self.place_stop_y = param[1]
        self.stop = True
        self.pick = False
        self.place = False
        self.target_color = 'red'
        self.broadcast_status = 'mission_completed'

        self.linear_pid.clear()
        self.angular_pid.clear()
        self.start_place = True

        response.success = True
        response.message = "start_place"
        return response 

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()


    def color_detect(self, img):
        img_h, img_w = img.shape[:2]
        frame_resize = cv2.resize(img, self.image_proc_size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(Convert the image to LAB color space)
        # self.get_logger().info(str(self.lab_data['lab']['Stereo'][self.target_color]['min']))
        # self.get_logger().info(str(self.lab_data['lab']['Stereo'][self.target_color]['max']))
        frame_mask = cv2.inRange(frame_lab, tuple(self.lab_data['lab']['Stereo'][self.target_color]['min']),
                                 tuple(self.lab_data['lab']['Stereo'][self.target_color]['max']))  # 对原图像和掩模进行位运算(Perform bitwise operations on the original image and the mask)

        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(Erosion)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(Dilation)

        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(Find the contour)
        # cv2.imshow('image', dilated)
        center_x, center_y, angle = -1, -1, -1
        if len(contours) != 0:
            areaMaxContour, area_max = common.get_area_max_contour(contours, 10)  # 找出最大轮廓(Locate the maximum contour)
            if areaMaxContour is not None:
                #print(111111111111, area_max)
                if 10 < area_max:  # 有找到最大面积(The maximum contour is found)
                    rect = cv2.minAreaRect(areaMaxContour)  # 最小外接矩形(Minimum bounding rectangle)
                    angle = rect[2]
                    box = np.intp(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点(The four vertices of the minimum bounding rectangle)
                    for j in range(4):
                        box[j, 0] = int(common.val_map(box[j, 0], 0, self.image_proc_size[0], 0, img_w))
                        box[j, 1] = int(common.val_map(box[j, 1], 0, self.image_proc_size[1], 0, img_h))

                    cv2.drawContours(img, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形(Draw the rectangle formed by the four points)
                    # 获取矩形的对角点
                    ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
                    pt3_x, pt3_y = box[2, 0], box[2, 1]
                    radius = abs(ptime_start_x - pt3_x)
                    center_x, center_y = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2)  # 中心点(Center point)
                    cv2.circle(img, (center_x, center_y), 5, (0, 255, 255), -1)  # 画出中心点(Draw the center)

        return center_x, center_y, angle

    def action_thread(self):
        while True:
            if self.pick:
                self.start_pick = False
                self.mecanum_pub.publish(Twist())
                

                self.controller.run_action('navigation_pick')

                if self.broadcast and self.broadcast_status == 'crawl_succeeded':
                    self.broadcast_status = 'mission_completed'
                    voice_play.play('crawl_succeeded', language=self.language)
                
                self.pick = False
                self.set_parameter(self.parameter_client, 'status', 'pick_finish')
                self.get_logger().info('pick finish')
                if self.broadcast:
                    self.transport_client.wait_for_service()
                    pose = SetPose2D.Request()
                    pose.data.x = self.place_position[0]
                    pose.data.y = self.place_position[1]
                    pose.data.roll = self.place_position[2]
                    pose.data.pitch = self.place_position[3]
                    pose.data.yaw = self.place_position[4]
                    res = self.send_request(self.transport_client, pose) 
                    if res.success:
                        self.get_logger().info('set place position success')
                    else:
                        self.get_logger().info('set place position failed')
            elif self.place:
                self.start_place = False
                self.mecanum_pub.publish(Twist())
                time.sleep(1)

                self.controller.run_action('navigation_place')

                self.set_parameter(self.parameter_client, 'status', 'place_finish')
                self.get_logger().info('place finish')
                if self.broadcast and self.broadcast_status == 'mission_completed':
                    self.broadcast_status = ''
                    voice_play.play('mission_completed', language=self.language)
                self.place = False
            else:
                time.sleep(0.01)

    def pick_handle(self, image):
        twist = Twist()
        if not self.pick or self.debug:
            object_center_x, object_center_y, object_angle = self.color_detect(image)  # 获取物体颜色的中心和角度(Obtain the center and angle of the object's color)
            if self.debug:
                self.detect_count += 1
                if self.detect_count > 10:
                    self.detect_count = 0
                    self.pick_stop_y = object_center_y
                    self.pick_stop_x = object_center_x
                    data = common.get_yaml_data(self.config_path)
                    data['/**']['ros__parameters']['pick_stop_pixel_coordinate'] = [self.pick_stop_x, self.pick_stop_y]
                    common.save_yaml_data(data, self.config_path)
                    self.debug = False
                self.get_logger().info('x_y: ' + str([object_center_x, object_center_y]))  # 打印当前物体中心的像素(Print the pixel coordinates of the current object center)
            elif object_center_x > 0:
                if self.broadcast and self.broadcast_status == 'find_target':
                    self.broadcast_status = 'crawl_succeeded'
                    voice_play.play('find_target', language=self.language)
                ########电机pid处理(motor pid processing)#########
                # 以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入(Use the x and y coordinates of the image's center point as the set values, and the current x and y coordinates as inputs.)#
                self.linear_pid.SetPoint = self.pick_stop_y
                if abs(object_center_y - self.pick_stop_y) <= self.d_y:
                    object_center_y = self.pick_stop_y
                if self.status != "align":
                    self.linear_pid.update(object_center_y)  # 更新pid(Update pid)
                    tmp = self.linear_base_speed + self.linear_pid.output

                    self.linear_speed = tmp
                    if tmp > 0.15:
                        self.linear_speed = 0.1
                    if tmp < -0.15:
                        self.linear_speed = -0.1
                    if abs(tmp) <= 0.0075:
                        self.linear_speed = 0

                self.angular_pid.SetPoint = self.pick_stop_x
                if abs(object_center_x - self.pick_stop_x) <= self.d_x:
                    object_center_x = self.pick_stop_x
                if self.status != "align":
                    self.angular_pid.update(object_center_x)  # 更新pid(Update pid)
                    tmp = self.angular_base_speed + self.angular_pid.output

                    self.angular_speed = tmp
                    if tmp > 1.2:
                        self.angular_speed = 1.2
                    if tmp < -1.2:
                        self.angular_speed = -1.2
                    if abs(tmp) <= 0.038:
                        self.angular_speed = 0
                if abs(self.linear_speed) == 0 and abs(self.angular_speed) == 0:
                    if 'Mecanum' in self.machine_type:
                        self.count_turn += 1
                        if self.count_turn > 5:
                            self.count_turn = 5
                            self.status = "align"
                            if self.count_stop < 10:  # 连续10次都没在移动(Not moving for 10 consecutive times)
                                if object_angle < 40: # 不取45，因为如果在45时值的不稳定会导致反复移动(Do not use 45, because instability at 45 can cause repeated movements)
                                    object_angle += 90
                                self.yaw_pid.SetPoint = 90
                                if abs(object_angle - 90) <= 1:
                                    object_angle = 90
                                self.yaw_pid.update(object_angle)  # 更新pid(Update pid)
                                self.yaw_angle = self.yaw_pid.output
                                if object_angle != 90:
                                    if abs(self.yaw_angle) <=0.038:
                                        self.count_stop += 1
                                    else:
                                        self.count_stop = 0
                                    twist.linear.y = float(-2 * 0.3 * math.sin(self.yaw_angle / 2))
                                    twist.angular.z = float(self.yaw_angle)
                                else:
                                    self.count_stop += 1
                            elif self.count_stop <= 20:
                                self.d_x = 5
                                self.d_y = 5
                                self.count_stop += 1
                                self.status = "adjust"
                            else:
                                self.count_stop = 0
                                self.pick = True
                    else:
                        self.count_stop += 1
                        if self.count_stop > 15:
                            self.count_stop = 0
                            self.pick = True
                else:
                    if self.count_stop >= 10:
                        self.count_stop = 10
                    self.count_turn = 0
                    if self.status != 'align':
                        twist.linear.x = float(self.linear_speed)
                        twist.angular.z = float(self.angular_speed)

        self.mecanum_pub.publish(twist)

        return image


    def place_handle(self, image):
        twist = Twist()
        if not self.place or self.debug:
            object_center_x, object_center_y, object_angle = self.color_detect(image)  # 获取物体颜色的中心和角度(Obtain the center and angle of the object's color)
            if self.debug:
                self.get_logger().info('x_y: %s'%str(object_center_x, object_center_y))  # 打印当前物体离中心的像素距离(Print the current pixel distance of the object from the center)
            elif object_center_x > 0:
                ########电机pid处理(motor pid processing)#########
                # 以图像的中心点的x，y坐标作为设定的值，以当前x，y坐标作为输入(Use the x and y coordinates of the image's center point as the reference values, and the current x and y coordinates as the inputs)#
                self.linear_pid.SetPoint = self.place_stop_y
                if abs(object_center_y - self.place_stop_y) <= self.d_y:
                    object_center_y = self.place_stop_y
                self.linear_pid.update(object_center_y)  # 更新pid(Update pid)
                tmp = self.linear_base_speed + self.linear_pid.output

                self.linear_speed = tmp
                if tmp > 0.15:
                    self.linear_speed = 0.15
                if tmp < -0.15:
                    self.linear_speed = -0.15
                if abs(tmp) <= 0.0075:
                    self.linear_speed = 0

                self.angular_pid.SetPoint = self.place_stop_x
                if abs(object_center_x - self.place_stop_x) <= self.d_x:
                    object_center_x = self.place_stop_x

                self.angular_pid.update(object_center_x)  # 更新pid(Update pid)
                tmp = self.angular_base_speed + self.angular_pid.output

                self.angular_speed = tmp
                if tmp > 1.2:
                    self.angular_speed = 1.2
                if tmp < -1.2:
                    self.angular_speed = -1.2
                if abs(tmp) <= 0.035:
                    self.angular_speed = 0

                if abs(self.linear_speed) == 0 and abs(self.angular_speed) == 0:
                    self.place = True
                else:
                    twist.linear.x = float(self.linear_speed)
                    twist.angular.z = float(self.angular_speed)

        self.mecanum_pub.publish(twist)

        return image


    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(If the queue is full, discard the oldest image)
            self.image_queue.get()
            # 将图像放入队列(Add the image into the queue)
        self.image_queue.put(rgb_image)

    def main(self):
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            if self.start_pick:
                self.stop = True
                result_image = self.pick_handle(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            elif self.start_place:
                self.stop = True
                if self.place_without_color:
                    self.place = True
                    result_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                else:
                    result_image = self.place_handle(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            else:
                if self.stop:
                    self.stop = False
                    self.mecanum_pub.publish(Twist())
                result_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            cv2.line(result_image, (self.pick_stop_x, 0), (self.pick_stop_x, 480), (0, 255, 255), 2)
            cv2.line(result_image, (0, self.pick_stop_y), (640, self.pick_stop_y), (0, 255, 255), 2)
            if self.enable_display:
                cv2.imshow(self.name, result_image)
                cv2.waitKey(1)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
        
        self.controller.run_action('navigation_pick_init')
        # set_servo_position(self.joints_pub, 2, ((1, 500), (2, 700), (3, 15), (4, 215), (5, 500), (10, 200)))
        self.mecanum_pub.publish(Twist())
        rclpy.shutdown()

def main():
    node = AutomaticPickNode('automatic_pick')
    rclpy.spin(node)
    node.destroy_node()
 
if __name__ == "__main__":
    main()

