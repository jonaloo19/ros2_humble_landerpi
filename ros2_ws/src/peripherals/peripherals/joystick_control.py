#!/usr/bin/env python3
# coding=utf8

import os
import math
import time
import rclpy
import sys
import threading
import numpy as np
import pygame as pg
from enum import Enum
from rclpy.node import Node
from sdk.common import val_map
from std_srvs.srv import Trigger
from sensor_msgs.msg import Joy
from functools import partial
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import BuzzerState
from ros_robot_controller_msgs.msg import BuzzerState, SetPWMServoState, PWMServoState

###############################################################################
AXES_MAP =  ('0', '1', '2', '3', 'hat_x', 'hat_y')


BUTTONS = [("cross", "circle", "", "square","triangle", "", "l1",
           "R1", "L2", "R2", "select", "start", "mode","lc","rc"),
           
           ("triangle", "circle", "cross",  "square","l1", "r1", "l2", "r2", 
            "select", "start",  "lc","rc","mode","","")]

class JoystickController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        os.environ["SDL_VIDEODRIVER"] = "dummy"  # For use PyGame without opening a visible display
        pg.display.init()
        
        self.min_value = 0.1
        self.declare_parameter('max_linear', 0.7)
        self.declare_parameter('max_angular', 3.0)
        self.declare_parameter('disable_servo_control', True)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.disable_servo_control = self.get_parameter('disable_servo_control').value
        self.machine = os.environ['MACHINE_TYPE']
        self.get_logger().info('\033[1;32m%s\033[0m' % self.max_linear)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.mecanum_pub = self.create_publisher(Twist, 'controller/cmd_vel', 1)
        

        self.js = None
        self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))        
        self.last_buttons = [0] * len(BUTTONS[0])
        self.lock = threading.Lock()
        
        self.create_timer(0.1, self.update_buttons) 
        threading.Thread(target=self.connect, daemon=True).start()

    def get_button_state(self, button):
        if button in self.BUTTONS:
            return self.last_buttons[self.BUTTONS.index(button)]
        return 0

    def connect(self):
        while True:
            if os.path.exists("/dev/input/js0"):
                with self.lock:
                    if self.js is None:
                        pg.joystick.init()
                        try:
                            self.js = pg.joystick.Joystick(0)
                            self.js.init()
                            # old device
                            if(self.js.get_name() == 'SHANWAN Android Gamepad'):
                                self.BUTTONS = BUTTONS[0]
                                self.last_buttons = [0] * len(self.BUTTONS)
                            
                            # new device
                            elif(self.js.get_name() == 'USB WirelessGamepad'):
                                self.BUTTONS = BUTTONS[1]        
                        except Exception as e:
                            print(e)
                            self.js = None
            else:
                with self.lock:
                    if self.js is not None:
                        self.js.quit()
                        self.js = None
            pg.time.delay(200)       
            

    def axes_callback(self, axes):
        twist = Twist()        
        if abs(axes['0']) < self.min_value:
            axes['0'] = 0
        if abs(axes['1']) < self.min_value:
            axes['1'] = 0
        if abs(axes['2']) < self.min_value:
            axes['2'] = 0
        if abs(axes['3']) < self.min_value:
            axes['3'] = 0

        if 'Mecanum' in self.machine:
            twist.linear.y = val_map(axes['0'], 1, -1, -self.max_linear, self.max_linear) 
            twist.linear.x = val_map(axes['1'], 1, -1, -self.max_linear, self.max_linear)
            twist.angular.z = val_map(axes['2'], 1, -1, -self.max_angular, self.max_angular)
        elif 'Tank' in self.machine:
            twist.linear.x = val_map(axes['1'], 0.8, -0.8, -self.max_linear, self.max_linear)
            twist.angular.z = val_map(axes['2'], 2.2, -2.2, -self.max_angular, self.max_angular)
        elif 'Acker' in self.machine:
            twist.linear.x = val_map(axes['1'], 1, -1, -self.max_linear, self.max_linear)
            steering_angle = val_map(axes['2'], 1, -1, -math.radians(40), math.radians(40))
            if twist.linear.x == 0:
                twist.linear.z = 1.0
                servo_state = PWMServoState()
                servo_state.id = [3]
                servo_state.position = [1500 + int(math.degrees(-steering_angle)/180*2000)]
                self.get_logger().info('\033[1;32m speed acker %f\033[0m' % (1500 + int(math.degrees(-steering_angle)/180*2000)))
                data = SetPWMServoState()
                data.state = [servo_state]
                data.duration = 0.02
                self.servo_state_pub.publish(data)
            else:
                angle = steering_angle
                if angle != 0:
                    R = 0.145/math.tan(angle)
                    twist.angular.z =  float(twist.linear.x/R)
  
            
        self.mecanum_pub.publish(twist)
   
    def handle_button_event(self, button_state, pressed):
        """
        处理按键事件：按下和释放
        """
        callback = "".join([self.BUTTONS[button_state], '_callback'])
        if hasattr(self, callback):
            try:
                getattr(self, callback)(pressed)
            except Exception as e:
                self.get_logger().error(str(e))
    
        
        
    def select_callback(self, new_state):
        pass

    def l1_callback(self, new_state):
        pass

    def l2_callback(self, new_state):
        pass

    def r1_callback(self, new_state):
        pass

    def r2_callback(self, new_state):
        pass

    def square_callback(self, new_state):
        pass

    def cross_callback(self, new_state):
        pass

    def circle_callback(self, new_state):
        pass

    def triangle_callback(self, new_state):
        pass

    def start_callback(self, new_state):
        if new_state:
            msg = BuzzerState()
            msg.freq = 2500
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)
    
    
    def update_buttons(self):
        try:
            while True:
                for event in pg.event.get():
                    if event.type == pg.QUIT:

                        sys.exit(0)

                    # 处理手柄摇杆事件
                    elif event.type == pg.JOYAXISMOTION:
                        axis_index = event.axis  # 获取当前轴的索引
                        axis_value = event.value
                        axis_key = AXES_MAP[axis_index]
                        self.last_axes[axis_key] = axis_value
                    # 处理十字键    
                    elif event.type == pg.JOYHATMOTION:
                        hat_y, hat_x = event.value                                              
                        self.last_axes['hat_x'] = hat_x
                        self.last_axes['hat_y'] = hat_y    
                    # 处理按键                                                
                    elif event.type == pg.JOYBUTTONDOWN:                        
                        self.handle_button_event(event.button, True)

                    elif event.type == pg.JOYBUTTONUP:
                        self.handle_button_event(event.button, False)

                    
                    self.axes_callback(self.last_axes)

        except KeyboardInterrupt:
            print("\n程序已手动退出")
            pg.quit()
            sys.exit(0)
            
        

def main():
    node = JoystickController('joystick_control')
    rclpy.spin(node)  

if __name__ == "__main__":
    main()
