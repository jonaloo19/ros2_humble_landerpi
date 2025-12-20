#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/21
# @author:aiden
# 机械臂运动学调用(Kinematics call for the robotic arm)
from kinematics_msgs.srv import SetRobotPose, SetJointValue

def set_pose_target(position, pitch, pitch_range=[-180.0, 180.0], resolution=1.0):
    '''
    给定坐标和俯仰角，返回逆运动学解(Given the coordinates and pitch angle, return the inverse kinematics solution)
    position: 目标位置，列表形式[x, y, z]，单位m(Target position, in the form of a list [x, y, z], in meters)
    pitch: 目标俯仰角，单位度，范围-180~180(Target pitch angle, in degrees, range -180 to 180)
    pitch_range: 如果在目标俯仰角找不到解，则在这个范围内寻找解(If no solution is found at the target pitch angle, search for a solution within this range)
    resolution: pitch_range范围角度的分辨率(Resolution of the pitch_range angle)
    return: 调用是否成功， 舵机的目标位置， 当前舵机的位置， 机械臂的目标姿态， 最优解所有舵机转动的变化量(Whether the call was successful, the target position of the servo, the current position of the servo, the target pose of the robotic arm, and the change in rotation of all servomotors in the optimal solution)
    '''
    msg = SetRobotPose.Request()
    msg.position = [float(i) for i in position]
    msg.pitch = float(pitch)
    msg.pitch_range = [float(i) for i in pitch_range]
    msg.resolution = float(resolution)
    return msg

def set_joint_value_target(joint_value):
    '''
    给定每个舵机的转动角度，返回机械臂到达的目标位置姿态(Given the rotation angles of each servomotor, return the target position and pose of the robotic arm)
    joint_value: 每个舵机转动的角度，列表形式[joint1, joint2, joint3, joint4, joint5]，单位脉宽(The rotation angle of each servomotor, in the form of a list [joint1, joint2, joint3, joint4, joint5], in pulse width units)
    return: 目标位置的3D坐标和位姿，格式geometry_msgs/Pose(3D coordinates and pose of the target position, in the format geometry_msgs/Pose)
    '''
    msg = SetJointValue.Request()
    msg.joint_value = [float(i) for i in joint_value]
    return msg
    
if __name__ == "__main__":
    import time
    import rclpy
    from rclpy.node import Node
    import kinematics.transform as transform
    # 初始化节点
    rclpy.init()
    client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
    while True:
        t = time.time()
        res = node.set_pose_target([transform.link3 + transform.tool_link, 0.0, 0.36], 0.0, [-180.0, 180.0], 1.0)
        print(time.time() - t)
    rclpy.logging.get_logger('p2').info(str(res[1]))
    # print('ik', res)
    # if res[1] != []:
        # res = set_joint_value_target(res[1])
        # print('fk', res)
    node.destroy_node()
    rclpy.shutdown()
