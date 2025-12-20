#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo


class DepthDebugNode(Node):
    def __init__(self):
        super().__init__('track_and_grab_moveit')

        self.camera_info = None
        self.depth_msg_count = 0

        # 直接订阅仿真中的真实深度图话题
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_cam/depth_image',          # ★ 一定是这个
            #'/depth_cam/depth_cam',    
            self.on_depth_image,
            10
        )

        # 直接订阅真实的 camera_info 话题
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/depth_cam/rgb/camera_info',    # ★ 一定是这个
            self.on_camera_info,
            10
        )

        self.get_logger().info('DepthDebugNode started (track_and_grab_moveit).')

    def on_camera_info(self, msg: CameraInfo):
        if self.camera_info is None:
            self.get_logger().info(
                f'Received CameraInfo: width={msg.width}, height={msg.height}, '
                f'fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}'
            )
        self.camera_info = msg

    def on_depth_image(self, msg: Image):
        self.depth_msg_count += 1
        if self.depth_msg_count <= 5:
            self.get_logger().info(
                f'Received depth Image #{self.depth_msg_count}: '
                f'{msg.width}x{msg.height}, encoding={msg.encoding}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = DepthDebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

