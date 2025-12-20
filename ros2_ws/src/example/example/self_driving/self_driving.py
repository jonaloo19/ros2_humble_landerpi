#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/28
# @author:aiden
# autonomous driving
import os
import cv2
import math
import time
import queue
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import sdk.fps as fps
from rclpy.node import Node
import sdk.common as common
# from app.common import Heart
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo
from std_srvs.srv import SetBool, Trigger
from sdk.common import colors, plot_one_box
from example.self_driving import lane_detect
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_robot_controller_msgs.msg import BuzzerState, SetPWMServoState, PWMServoState
from servo_controller.bus_servo_control import set_servo_position

class SelfDrivingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.is_running = True
        self.pid = pid.PID(0.5, 0.0, 0.05)
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.param_init()

        self.fps = fps.FPS()  
        self.image_queue = queue.Queue(maxsize=2)
        self.classes = ['go', 'right', 'park', 'red', 'green', 'crosswalk']
        self.display = True
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.colors = common.Colors()
        # signal.signal(signal.SIGINT, self.shutdown)
        self.lane_detect = lane_detect.LaneDetector("yellow")

        self.mecanum_pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 1)

        self.create_service(Trigger, '~/enter', self.enter_srv_callback) # enter the game
        self.create_service(Trigger, '~/exit', self.exit_srv_callback) # exit the game
        self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)

        self.start = self.get_parameter('start').get_parameter_value().bool_value

        # self.heart = Heart(self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(None))
        timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/yolo_node/init_finish')
        self.client.wait_for_service()
        self.start_yolo_client = self.create_client(Trigger, '/yolo_node/start', callback_group=timer_cb_group)
        self.start_yolo_client.wait_for_service()
        self.stop_yolo_client = self.create_client(Trigger, '/yolo_node/stop', callback_group=timer_cb_group)
        self.stop_yolo_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()

        self.mecanum_pub.publish(Twist())
        set_servo_position(self.joints_pub, 1.5, ((10, 300), (5, 500), (4, 150), (3, 210), (2, 650), (1, 500)))
        if not self.get_parameter('only_line_follow').value:
            self.send_request(self.start_yolo_client, Trigger.Request())
        time.sleep(1)
        
        self.debug_park = self.declare_parameter('debug_park', False).value
        
        if self.debug_park:
            self.get_logger().info('\033[1;32m%s\033[0m' % "调试模式：立即执行停车动作")
            threading.Thread(target=self.park_action, daemon=True).start()
        
        if self.start:
            self.display = True
            self.enter_srv_callback(Trigger.Request(), Trigger.Response())
            request = SetBool.Request()
            request.data = True
            self.set_running_srv_callback(request, SetBool.Response())

        #self.park_action() 
        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def param_init(self):
        self.start = False
        self.enter = False
        self.right = True

        self.have_turn_right = False
        self.detect_turn_right = False
        self.detect_far_lane = False
        self.park_x = -1  # 获取停车标志的x像素坐标
        self.park_cross_dist = 190
        if 'Pro' in self.machine_type:
            self.park_cross_dist = 70
            # self.detect_right_speed = 0.05
        self.start_turn_time_stamp = 0
        self.count_turn = 0
        self.start_turn = False  # start to turn

        self.count_right = 0
        self.count_right_miss = 0
        self.turn_right = False  # right turning sign

        self.last_park_detect = False
        self.count_park = 0  
        self.stop = False  # 停止标志
        self.start_park = False  # 开始停车标志

        self.count_crosswalk = 0
        self.crosswalk_distance = 0  # distance to the zebra crossing
        self.crosswalk_length = 0.1 + 0.3  # the length of zebra crossing and the robot

        self.start_slow_down = False  # slowing down sign
        self.normal_speed = 0.11  # normal driving speed
        self.slow_down_speed = 0.11  # slowing down speed

        self.traffic_signs_status = None  # record the state of the traffic lights
        self.red_loss_count = 0

        self.object_sub = None
        self.image_sub = None
        self.objects_info = []

        # 添加保存近距离补线结果的变量，便于后续一直沿着这条线前进
        self.saved_park_line = None

    def get_node_state(self, request, response):
        response.success = True
        return response

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "self driving enter")
        with self.lock:
            self.start = False
            camera = 'ascamera'#self.get_parameter('depth_camera_name').value
            self.create_subscription(Image, '/ascamera/camera_publisher/rgb0/image' , self.image_callback, 1)
            self.create_subscription(ObjectsInfo, '/yolo_node/object_detect', self.get_object_callback, 1)
            self.mecanum_pub.publish(Twist())
            self.enter = True
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "self driving exit")
        with self.lock:
            try:
                if self.image_sub is not None:
                    self.image_sub.unregister()
                if self.object_sub is not None:
                    self.object_sub.unregister()
            except Exception as e:
                self.get_logger().info('\033[1;32m%s\033[0m' % str(e))
            self.mecanum_pub.publish(Twist())
        self.param_init()
        response.success = True
        response.message = "exit"
        return response

    def set_running_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_running")
        with self.lock:
            self.start = request.data
            if not self.start:
                self.mecanum_pub.publish(Twist())
        response.success = True
        response.message = "set_running"
        return response

    def shutdown(self, signum, frame):  # press 'ctrl+c' to close the program
        self.is_running = False

    def image_callback(self, ros_image):  # callback target checking
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # if the queue is full, remove the oldest image
            self.image_queue.get()
        # put the image into the queue
        self.image_queue.put(rgb_image)
    
    # parking processing
    def park_action(self):
        # if self.machine_type == 'MentorPi_Mecanum': 
        if 'Mecanum' in self.machine_type: 
            twist = Twist()
            twist.linear.y = -0.2
            self.mecanum_pub.publish(twist)
            time.sleep(0.38/0.2)
        # elif self.machine_type == 'MentorPi_Acker':
        elif 'Acker' in self.machine_type:
            twist = Twist()
            twist.linear.x = 0.15
            twist.angular.z = twist.linear.x*math.tan(-0.5061)/0.145
            self.mecanum_pub.publish(twist)
            time.sleep(3)

            twist = Twist()
            twist.linear.x = 0.15
            twist.angular.z = -twist.linear.x*math.tan(-0.5061)/0.145
            self.mecanum_pub.publish(twist)
            time.sleep(2)

            twist = Twist()
            twist.linear.x = -0.15
            twist.angular.z = twist.linear.x*math.tan(-0.5061)/0.145
            self.mecanum_pub.publish(twist)
            time.sleep(1.5)

        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % "执行默认停车模式")
            # 添加每个动作的详细日志
            self.get_logger().info('\033[1;32m%s\033[0m' % "步骤1: 左转")
            twist = Twist()
            time.sleep(0.5)
            twist.angular.z = -1.0
            self.mecanum_pub.publish(twist)
            time.sleep(1.1)
            self.mecanum_pub.publish(Twist())
            
            self.get_logger().info('\033[1;32m%s\033[0m' % "步骤2: 前进")
            twist = Twist()
            twist.linear.x = 0.2
            self.mecanum_pub.publish(twist)
            time.sleep(0.40/0.2)
            self.mecanum_pub.publish(Twist())
            
            self.get_logger().info('\033[1;32m%s\033[0m' % "步骤3: 右转")
            twist = Twist()
            twist.angular.z = 1.0
            self.mecanum_pub.publish(twist)
            time.sleep(1.1)
        
        self.get_logger().info('\033[1;32m%s\033[0m' % "停车动作完成")
        self.mecanum_pub.publish(Twist())

    def main(self):
        while self.is_running:
            time_start = time.time()
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                self.get_logger().info('图像队列为空')
                if not self.is_running:
                    break
                else:
                    continue

            result_image = image.copy()
            if self.start:
                h, w = image.shape[:2]

                # obtain the binary image of the lane
                binary_image = self.lane_detect.get_binary(image)

                twist = Twist()

                # 始终打印 crosswalk_distance 的值
                self.get_logger().info('\033[1;33m crosswalk_distance: %s\033[0m' % self.crosswalk_distance)
 
                # if detecting the zebra crossing, start to slow down
                if 70 < self.crosswalk_distance and not self.start_slow_down:  # The robot starts to slow down only when it is close enough to the zebra crossing
                    self.count_crosswalk += 3
                    if self.count_crosswalk == 3:  # judge multiple times to prevent false detection
                        self.count_crosswalk = 0
                        self.start_slow_down = True  # sign for slowing down
                        self.count_slow_down = time.time()  # fixing time for slowing down
                else:  # need to detect continuously, otherwise reset
                    self.count_crosswalk = 0

                # deceleration processing
                if self.start_slow_down:
                    if self.traffic_signs_status is not None:
                        area = abs(self.traffic_signs_status.box[0] - self.traffic_signs_status.box[2]) * abs(self.traffic_signs_status.box[1] - self.traffic_signs_status.box[3])
                        if self.traffic_signs_status.class_name == 'red' and area < 1000:  # If the robot detects a red traffic light, it will stop
                            self.mecanum_pub.publish(Twist())
                            self.stop = True
                        elif self.traffic_signs_status.class_name == 'green':  # If the traffic light is green, the robot will slow down and pass through
                            twist.linear.x = self.slow_down_speed
                            self.stop = False
                    if not self.stop:  # In other cases where the robot is not stopped, slow down the speed and calculate the time needed to pass through the crosswalk. The time needed is equal to the length of the crosswalk divided by the driving speed
                        twist.linear.x = self.slow_down_speed
                        if time.time() - self.count_slow_down > self.crosswalk_length / twist.linear.x:
                            self.start_slow_down = False
                else:
                    twist.linear.x = self.normal_speed  # go straight with normal speed

                # If the robot detects a stop sign and a crosswalk, it will slow down to ensure stable recognition
                if 0 < self.park_x and 130 < self.crosswalk_distance:
                    twist.linear.x = self.slow_down_speed
                    if not self.start_park and 130 < self.crosswalk_distance:  # 当机器人足够接近人行横道时，它将开始停车
                            self.count_park += 1 
                            if self.count_park >= 15:
                                self.mecanum_pub.publish(Twist()) 
                                self.start_park = True
                                self.stop = True
                                threading.Thread(target=self.park_action).start()
                    else:
                        self.count_park = 0  

 
                # 右转及停车补线策略(right turn and stop line filling strategy)
                if self.turn_right:
                        y = self.lane_detect.add_horizontal_line(binary_image)
                        if 0 < y < 400:
                            roi = [(0, y), (w, y), (w, 0), (0, 0)]
                            cv2.fillPoly(binary_image, [np.array(roi)], [0, 0, 0])  # 将上面填充为黑色，防干扰(fill the above area with black to prevent interference)
                            min_x = cv2.minMaxLoc(binary_image)[-1][0]
                            cv2.line(binary_image, (min_x, y), (w, y), (255, 255, 255), 40)  # 画虚拟线来驱使转弯(draw virtual lines to guide the turn)
                            
                            # 在结果图像上也显示这条水平线，便于观察
                            # cv2.line(result_image, (min_x, y), (w, y), (0, 0, 255), 5)  # 红色线，更容易看到
                            
                            # 显示二值图像中的水平线
                            # if self.display:
                            #     cv2.imshow("右转补线二值图像", binary_image)
                                
                            self.get_logger().info('\033[1;34m执行右转补线: y=%s, min_x=%s\033[0m' % (y, min_x))
                elif 0 < self.park_x and not self.start_turn:   # and 190 > self.crosswalk_distance  and not self.start_park检测到停车标识需要填补线，使其保持直走(if a stop sign is detected, fill in the lines to keep the vehicle going straight)                    
                    if not self.detect_far_lane:
                    # if self.saved_park_line is None: 
                        up, down, center = self.lane_detect.add_vertical_line_near(binary_image)
                        # self.saved_park_line = (up, down, center)
                        self.get_logger().info('\033[1;35m近距离补线: up=%s, down=%s, center=%s\033[0m' % (up, down, center))
                        self.get_logger().info('\033[1;33m当前 center 值: %s (阈值范围: 60-100)\033[0m' % center)
                        binary_image[:, :] = 0  # 全置黑，防止干扰(set all to black to prevent interference)
                        if 5 < center < 130:  #可调参数，先调小当将要看不到车道线时切换到识别较远车道线(switch to recognizing farther lane lines when lane lines are about to become invisible)
                            self.detect_far_lane = True
                            self.get_logger().info('\033[1;35m切换到远距离补线模式:')               
                    else:
                        up, down = self.lane_detect.add_vertical_line_far(binary_image)
                        # binary_image[:, :] = 0
                    # 不清空整个二值图像，直接绘制补线
                    if up != down:
                        cv2.line(binary_image, up, down, (255, 255, 255), 20)
                            
                        # cv2.line(result_image, up, down, (0, 255, 0), 5)
                        self.get_logger().info('\033[1;35m成功绘制补线\033[0m')

                else:
                    # 不满足补线条件时，清除保存的补线数据，恢复正常巡线
                    if self.saved_park_line is not None:
                        self.get_logger().info('\033[1;36m条件不满足，清除保存的补线数据，恢复正常巡线\033[0m')
                        self.saved_park_line = None

                # line following processing
                result_image, lane_angle, lane_x = self.lane_detect(binary_image, image.copy())  # the coordinate of the line while the robot is in the middle of the lane 
                if lane_x >= 0 and not self.stop:  
                    if lane_x > 150:  
                        if self.turn_right:  # 如果是检测到右转标识的转弯(if it's a right turn sign detected, initiate the turn)
                            self.count_right_miss += 1
                            if self.count_right_miss >= 10:  # 可调节参数，如果右转标识连续检测到10次，则认为右转标识消失，停止转弯(if the right turn sign is detected 10 times in a row, it is considered that the right turn sign has disappeared, and the turn is stopped)
                                self.count_right_miss = 0
                                self.turn_right = False
                                self.normal_speed = 0.1

                        self.count_turn += 1
                        if self.count_turn > 5 and not self.start_turn:
                            self.start_turn = True
                            self.count_turn = 0
                            self.start_turn_time_stamp = time.time()
                        if 'Acker' not in self.machine_type:
                            twist.angular.z = -0.45  # turning speed
                        elif 'Tank' in self.machine_type: 
                            twist.angular.z = twist.linear.x * math.tan(-0.5061) / 0.157
                        else:
                            twist.angular.z = twist.linear.x * math.tan(-0.5061) / 0.145
                    
                    else:  # use PID algorithm to correct turns on a straight road
                        self.count_turn = 0
                        if time.time() - self.start_turn_time_stamp > 2 and self.start_turn:
                            self.start_turn = False
                        if not self.start_turn:
                            self.pid.SetPoint = 110  # the coordinate of the line while the robot is in the middle of the lane
                            self.pid.update(lane_x)
                            if 'Acker' not in self.machine_type:
                                twist.angular.z = common.set_range(self.pid.output, -0.1, 0.1)
                            else:
                                twist.angular.z = twist.linear.x * math.tan(common.set_range(self.pid.output, -0.1, 0.1)) / 0.145
                        else:
                            if 'Acker' in self.machine_type:
                                twist.angular.z = 0.15 * math.tan(-0.5061) / 0.145
                    self.mecanum_pub.publish(twist) 
                else:
                    self.pid.clear()

             
                if self.objects_info:
                    for i in self.objects_info:
                        box = i.box
                        class_name = i.class_name
                        cls_conf = i.score
                        cls_id = self.classes.index(class_name)
                        color = colors(cls_id, True)
                        plot_one_box(
                            box,
                            result_image,
                            color=color,
                            label="{}:{:.2f}".format(class_name, cls_conf),
                        )

            else:
                self.get_logger().info('self.start 为 False，机器人不会移动')
                time.sleep(0.01)

            
            bgr_image = cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR)
            if self.display:
                self.fps.update()
                bgr_image = self.fps.show_fps(bgr_image)
                
                # 显示二值图像，便于调试
                # if self.turn_right or (0 < self.park_x and not self.start_turn and 200 > self.crosswalk_distance):
                #     binary_display = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
                #     cv2.imshow("二值图像", binary_display)
                
                # Display the image in a popup window
                cv2.imshow("Self Driving", bgr_image)
                cv2.waitKey(1)  # Allow window to refresh
 
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(bgr_image, "bgr8"))

           
            time_d = 0.03 - (time.time() - time_start)
            if time_d > 0:
                time.sleep(time_d)

            # 在打印 crosswalk_distance 后添加
            # self.get_logger().info('\033[1;36m park_x: %s\033[0m' % self.park_x)

        self.mecanum_pub.publish(Twist())
        rclpy.shutdown()


    # Obtain the target detection result
    def get_object_callback(self, msg):
        self.objects_info = msg.objects
        self.get_logger().info('\033[1;32m [Get Object Callback] \033[0m')
        if self.objects_info == []:  # If it is not recognized, reset the variable
            self.traffic_signs_status = None
            self.crosswalk_distance = 0
            self.get_logger().info('\033[1;32m [Detect Noting] %s\033[0m')
        else:
            min_distance = 0
            for i in self.objects_info:
                class_name = i.class_name
                center = (int((i.box[0] + i.box[2])/2), int((i.box[1] + i.box[3])/2)) 
                if class_name == 'crosswalk': 
                    if center[1] > min_distance:  # Obtain recent y-axis pixel coordinate of the crosswalk
                        min_distance = center[1]
                elif class_name == 'right':  # obtain the right turning sign
                    self.count_right += 1
                    self.count_right_miss = 0
                    if self.count_right >= 5:  # If it is detected multiple times, take the right turning sign to true
                        self.turn_right = True
                        # self.normal_speed = self.detect_right_speed
                        self.count_right = 0
                elif class_name == 'park':  # obtain the center coordinate of the parking sign
                    self.park_x = center[0]
                elif class_name == 'red' or class_name == 'green':  # obtain the status of the traffic light
                    self.traffic_signs_status = i
               

            self.get_logger().info('\033[1;32m%s\033[0m' % class_name)
            self.crosswalk_distance = min_distance

def main():
    node = SelfDrivingNode('self_driving')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()

    
