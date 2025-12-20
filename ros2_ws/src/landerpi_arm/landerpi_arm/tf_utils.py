#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TF2 transformation utilities for pose conversions
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs  # Required for PoseStamped transform


class TFUtils:
    """Utility class for TF2 transformations"""
    
    def __init__(self, node: Node):
        """
        Initialize TF utilities
        
        Args:
            node: ROS2 node instance
        """
        self.node = node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.node.get_logger().info("TF utilities initialized")
    
    def transform_pose(self, pose_stamped: PoseStamped, target_frame: str, timeout: float = 1.0) -> PoseStamped:
        """
        Transform a PoseStamped to target frame
        
        Args:
            pose_stamped: Input pose with frame_id
            target_frame: Target coordinate frame
            timeout: Maximum wait time for transform (seconds)
            
        Returns:
            Transformed PoseStamped in target frame
            
        Raises:
            LookupException: If transform not available
            ConnectivityException: If TF tree connectivity issue
            ExtrapolationException: If transform timestamp issue
        """
        try:
            # Wait for transform to be available
            self.node.get_logger().debug(
                f"Transforming pose from '{pose_stamped.header.frame_id}' to '{target_frame}'"
            )
            
            # Check if transform exists
            if not self.tf_buffer.can_transform(
                target_frame,
                pose_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout)
            ):
                raise LookupException(
                    f"Cannot find transform from '{pose_stamped.header.frame_id}' to '{target_frame}'"
                )
            
            # Perform transformation
            transformed_pose = self.tf_buffer.transform(pose_stamped, target_frame)
            
            self.node.get_logger().debug(
                f"Transformed pose: [{transformed_pose.pose.position.x:.3f}, "
                f"{transformed_pose.pose.position.y:.3f}, "
                f"{transformed_pose.pose.position.z:.3f}]"
            )
            
            return transformed_pose
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().error(f"TF transform failed: {str(e)}")
            raise
    
    def lookup_transform(self, target_frame: str, source_frame: str, timeout: float = 1.0) -> TransformStamped:
        """
        Look up transform between two frames
        
        Args:
            target_frame: Target frame
            source_frame: Source frame
            timeout: Maximum wait time (seconds)
            
        Returns:
            TransformStamped between frames
            
        Raises:
            LookupException: If transform not available
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout)
            )
            return transform
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.node.get_logger().error(
                f"Failed to lookup transform from '{source_frame}' to '{target_frame}': {str(e)}"
            )
            raise

