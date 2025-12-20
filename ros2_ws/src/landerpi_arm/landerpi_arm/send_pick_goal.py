#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Command-line tool to send pick goals to grasp action server
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PointStamped
from landerpi_interfaces.action import PickPose
import argparse
import sys


class PickGoalSender(Node):
    """Node to send pick goals via command line"""
    
    def __init__(self):
        super().__init__('pick_goal_sender')
        self.action_client = ActionClient(self, PickPose, '/arm/pick_pose')
        self.get_logger().info("Pick goal sender initialized")
    
    def send_goal(self, x, y, z, frame_id='base_footprint', object_id=''):
        """
        Send pick goal to action server
        
        Args:
            x, y, z: Target position coordinates
            frame_id: Reference frame
            object_id: Optional object identifier
        """
        # Wait for server
        self.get_logger().info("Waiting for action server...")
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available!")
            return False
        
        # Create goal message
        goal_msg = PickPose.Goal()
        
        # Set target pose
        goal_msg.target_pose = PoseStamped()
        goal_msg.target_pose.header.frame_id = frame_id
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.target_pose.pose.position.z = z
        
        # Orientation will be set by server based on grasp_orientation config
        goal_msg.target_pose.pose.orientation.w = 1.0
        
        goal_msg.object_id = object_id
        
        # Send goal
        self.get_logger().info(
            f"Sending pick goal: frame='{frame_id}', "
            f"position=[{x:.3f}, {y:.3f}, {z:.3f}], "
            f"object_id='{object_id}'"
        )
        
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected!")
            return False
        
        self.get_logger().info("Goal accepted, waiting for result...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        
        # Print result
        if result.success:
            self.get_logger().info(
                f"✓ SUCCESS: {result.message}"
            )
        else:
            self.get_logger().error(
                f"✗ FAILED: {result.message} (error_code={result.error_code})"
            )
        
        return result.success
    
    def feedback_callback(self, feedback_msg):
        """Handle action feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"[{feedback.stage}] Progress: {feedback.progress*100:.0f}%"
        )


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Send pick goal to LanderPi arm grasp action server'
    )
    parser.add_argument('--x', type=float, required=True,
                        help='Target X position')
    parser.add_argument('--y', type=float, required=True,
                        help='Target Y position')
    parser.add_argument('--z', type=float, required=True,
                        help='Target Z position')
    parser.add_argument('--frame', type=str, default='base_footprint',
                        help='Reference frame (default: base_footprint)')
    parser.add_argument('--object-id', type=str, default='',
                        help='Object identifier (optional)')
    parser.add_argument('--point-topic', type=str, default='',
                        help='If set, subscribe to geometry_msgs/PointStamped and send first point as goal')
    
    # Parse only known args (let ROS parse its own args)
    parsed_args, unknown = parser.parse_known_args()
    
    # Initialize ROS
    rclpy.init(args=args)
    
    sender = PickGoalSender()

    def point_cb(msg: PointStamped):
        """Callback to handle a point message and send pick goal."""
        sender.get_logger().info(
            f"Received point from topic '{parsed_args.point_topic}': "
            f"[{msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}] in frame '{msg.header.frame_id}'"
        )
        success = sender.send_goal(
            msg.point.x,
            msg.point.y,
            msg.point.z,
            msg.header.frame_id if msg.header.frame_id else parsed_args.frame,
            parsed_args.object_id
        )
        sys.exit(0 if success else 1)

    try:
        if parsed_args.point_topic:
            sender.get_logger().info(f"Subscribing to point topic '{parsed_args.point_topic}' ...")
            sender.create_subscription(PointStamped, parsed_args.point_topic, point_cb, 10)
            rclpy.spin(sender)
            sys.exit(1)  # should exit in callback
        else:
            success = sender.send_goal(
                parsed_args.x,
                parsed_args.y,
                parsed_args.z,
                parsed_args.frame,
                parsed_args.object_id
            )
            sys.exit(0 if success else 1)

    except KeyboardInterrupt:
        sender.get_logger().info("Interrupted by user")
        sys.exit(1)
    finally:
        sender.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

