#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Auto pick executor: move arm to cruise pose, listen to color detections,
convert camera-frame 3D points to base frame, and call /arm/pick_pose.
Designed as an offline script that can also be called by a navigation module.
"""

import json
import math
from typing import List, Optional, Deque
from collections import deque
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from rclpy.exceptions import ParameterAlreadyDeclaredException
import tf2_geometry_msgs  # noqa: F401  Needed for geometry_msgs TF conversions

from landerpi_interfaces.action import PickPose
from landerpi_arm.moveit2_client import MoveIt2Client


class ColorPickExecutor(Node):
    """Subscribe detections, transform to base frame, and send pick goals."""

    def __init__(self):
        super().__init__("color_pick_executor")

        # Parameters (can be overridden by navigation module)
        try:
            self.declare_parameter("use_sim_time", False)
        except ParameterAlreadyDeclaredException:
            pass
        self.declare_parameter(
            "color_priority", ["GREEN", "RED", "BLUE", "YELLOW"]
        )  # ordered list
        self.declare_parameter(
            "cruise_joints",
            [0.0, -0.5, 1.0, -0.5, 0.0],  # example cruise pose with joint1=0
        )
        self.declare_parameter("skip_home", False)  # navigation module can set True
        # Camera intrinsics (used to recompute 3D from u,v,depth if provided)
        self.declare_parameter("fx", 554.25)
        self.declare_parameter("fy", 554.25)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("depth_scale", 1.0)  # keep 1.0 if depth already in meters
        # Temporal smoothing window (number of detections to median)
        self.declare_parameter("median_window", 3)
        # Grasp pose tuning
        self.declare_parameter("z_min", 0.035)       # minimum target z (m)
        self.declare_parameter("dz_grasp", 0.00)   # delta z applied to target (m)
        self.declare_parameter("dx_grasp", -0.02)     # delta x applied to target (m)
        self.declare_parameter("dy_grasp", -0.01)       # delta y applied to target (m)
        self.declare_parameter("stabilize_sec", 5.0)  # wait after cruise (s)
        # Frame IDs (override if sim publishes different camera/base frames)
        self.declare_parameter("camera_frame", "depth_cam_link")
        self.declare_parameter("base_frame", "base_footprint")

        # Apply use_sim_time
        # use_sim_time is declared above; no need to set again. Just read when needed.

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._tf_ready = False
        self._tf_wait_start = time.monotonic()
        self._tf_wait_timer = self.create_timer(0.5, self._check_tf_ready)

        # Action client for grasp
        self.pick_client = ActionClient(self, PickPose, "/arm/pick_pose")

        # MoveIt2 client for joint motions (HOME -> cruise)
        self.moveit_client = MoveIt2Client(self)

        # Internal state
        self._target_ready = False
        self._target_pose_base: Optional[PoseStamped] = None
        self._busy = False
        self._done = False
        self._recent_targets: Deque[PoseStamped] = deque(maxlen=int(self.get_parameter("median_window").get_parameter_value().integer_value or 3))

        # Subscribe detection info
        self.create_subscription(
            String, "/color_detection/detection_info", self.detection_callback, 10
        )

        self.get_logger().info("ColorPickExecutor initialized")

    def wait_for_servers(self):
        """Wait for MoveIt2 and pick action servers."""
        self.moveit_client.wait_for_action_server(timeout=15.0)
        self.get_logger().info("Waiting for pick action server...")
        if not self.pick_client.wait_for_server(timeout_sec=15.0):
            raise RuntimeError("Pick action server not available")
        self.get_logger().info("All servers ready")

    def move_to_home_and_cruise(self):
        """Optionally go HOME then cruise pose."""
        skip_home = self.get_parameter("skip_home").get_parameter_value().bool_value
        cruise_joints = [
            float(v) for v in self.get_parameter("cruise_joints").get_parameter_value().double_array_value
        ]

        if not skip_home:
            self.get_logger().info("Moving to HOME [0,0,0,0,0] before cruise")
            success, err = self.moveit_client.plan_and_execute_joints(
                [0.0, 0.0, 0.0, 0.0, 0.0], velocity_scaling=0.25, acceleration_scaling=0.20, planning_time=6.0
            )
            if not success:
                raise RuntimeError(f"Failed to move HOME: {err}")

        self.get_logger().info(f"Moving to cruise pose: {cruise_joints}")
        success, err = self.moveit_client.plan_and_execute_joints(
            cruise_joints, velocity_scaling=0.25, acceleration_scaling=0.20, planning_time=6.0
        )
        if not success:
            raise RuntimeError(f"Failed to move cruise: {err}")
        # Pause to mimic navigation settling and allow detection to stabilize
        stabilize_sec = self.get_parameter("stabilize_sec").get_parameter_value().double_value
        self.get_logger().info(f"Cruise pose reached, waiting {stabilize_sec:.1f}s for stabilization...")
        time.sleep(stabilize_sec)
        self.get_logger().info("Stabilization done, waiting for detections...")

    def detection_callback(self, msg: String):
        """Handle detection_info JSON; pick first valid target by color priority."""
        if self._busy or self._done:
            return
        if not self._tf_ready:
            return

        try:
            data = json.loads(msg.data)
            detections = data.get("detections", [])
        except Exception as e:
            self.get_logger().warn(f"Failed to parse detection_info: {e}")
            return

        if not detections:
            return

        # Build priority list
        color_priority: List[str] = [
            c.upper() for c in self.get_parameter("color_priority").get_parameter_value().string_array_value
        ]

        # Find first matching detection with 3D info
        chosen = None
        for color in color_priority:
            for det in detections:
                if str(det.get("color_name", "")).upper() == color:
                    chosen = det
                    break
            if chosen:
                break

        if not chosen:
            return

        # Prefer recomputing 3D using u,v,depth to reduce upstream noise
        u = chosen.get("center_x")
        v = chosen.get("center_y")
        depth_val = chosen.get("depth_value")
        fx = self.get_parameter("fx").get_parameter_value().double_value
        fy = self.get_parameter("fy").get_parameter_value().double_value
        cx = self.get_parameter("cx").get_parameter_value().double_value
        cy = self.get_parameter("cy").get_parameter_value().double_value
        depth_scale = self.get_parameter("depth_scale").get_parameter_value().double_value or 1.0

        if depth_val is None or u is None or v is None:
            return

        z_opt = float(depth_val) * depth_scale
        x_opt = (float(u) - cx) * z_opt / fx
        y_opt = (float(v) - cy) * z_opt / fy

        # Convert from optical frame (x right, y down, z forward) to link frame (x forward, y left, z up)
        # Here we map: x_link = z_opt, y_link = -x_opt, z_link = -y_opt
        x_link = float(z_opt)
        y_link = float(-x_opt)
        z_link = float(-y_opt)

        camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        base_frame = self.get_parameter("base_frame").get_parameter_value().string_value

        # Build PointStamped in camera frame
        pt = PointStamped()
        pt.header.frame_id = camera_frame
        # Use latest TF by stamping at time 0 to avoid future extrapolation in sim time.
        pt.header.stamp = rclpy.time.Time().to_msg()
        pt.point.x = x_link
        pt.point.y = y_link
        pt.point.z = z_link

        try:
            if not self.tf_buffer.can_transform(
                base_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            ):
                self.get_logger().warn(
                    f"TF not ready for {camera_frame} -> {base_frame}",
                    throttle_duration_sec=2.0,
                )
                return
            transformed = self.tf_buffer.transform(
                pt, base_frame, timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        pose = PoseStamped()
        pose.header = transformed.header
        # Lift target in Z to account for finger offset and avoid touching ground,
        # and apply small offsets to better align the finger tips with the object center.
        z_min = self.get_parameter("z_min").get_parameter_value().double_value
        dz_grasp = self.get_parameter("dz_grasp").get_parameter_value().double_value
        dx_grasp = self.get_parameter("dx_grasp").get_parameter_value().double_value
        dy_grasp = self.get_parameter("dy_grasp").get_parameter_value().double_value
        pose.pose.position.x = transformed.point.x + dx_grasp
        pose.pose.position.y = transformed.point.y + dy_grasp
        pose.pose.position.z = max(transformed.point.z + dz_grasp, z_min)
        pose.pose.orientation.w = 1.0  # orientation handled by grasp server

        # Temporal median smoothing
        self._recent_targets.append(pose)
        xs = sorted(p.pose.position.x for p in self._recent_targets)
        ys = sorted(p.pose.position.y for p in self._recent_targets)
        zs = sorted(p.pose.position.z for p in self._recent_targets)
        mid = len(self._recent_targets) // 2
        pose_smoothed = PoseStamped()
        pose_smoothed.header = pose.header
        pose_smoothed.pose.position.x = xs[mid]
        pose_smoothed.pose.position.y = ys[mid]
        pose_smoothed.pose.position.z = zs[mid]
        pose_smoothed.pose.orientation.w = 1.0

        self._target_pose_base = pose_smoothed
        self._target_ready = True
        self.get_logger().info(
            f"Target locked (median): [{pose_smoothed.pose.position.x:.3f}, {pose_smoothed.pose.position.y:.3f}, {pose_smoothed.pose.position.z:.3f}]"
        )

    def send_pick_goal(self):
        """Send pick goal using selected target pose in base frame."""
        if not self._target_ready or self._target_pose_base is None:
            return

        self._busy = True
        goal_msg = PickPose.Goal()
        goal_msg.target_pose = self._target_pose_base
        goal_msg.object_id = ""  # optional

        self.get_logger().info("Sending pick goal to /arm/pick_pose ...")
        send_future = self.pick_client.send_goal_async(goal_msg, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Pick goal rejected")
            self._busy = False
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.success:
            self.get_logger().info(f"Pick success: {result.message}")
        else:
            self.get_logger().error(f"Pick failed: {result.message} (code={result.error_code})")

        self._done = True

    def _check_tf_ready(self):
        camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        if self.tf_buffer.can_transform(
            base_frame,
            camera_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.1),
        ):
            self._tf_ready = True
            self.get_logger().info(f"TF ready for {camera_frame} -> {base_frame}")
            if self._tf_wait_timer is not None:
                self._tf_wait_timer.cancel()
                self.destroy_timer(self._tf_wait_timer)
                self._tf_wait_timer = None
        else:
            if (time.monotonic() - self._tf_wait_start) >= 2.0:
                self.get_logger().warn(
                    f"Waiting for TF {camera_frame} -> {base_frame}",
                    throttle_duration_sec=5.0,
                )
        self._busy = False

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f"[{fb.stage}] {fb.progress*100:.0f}%")

    def run(self):
        """Main loop: wait servers, move, then wait for detection and pick once."""
        try:
            self.wait_for_servers()
            self.move_to_home_and_cruise()

            self.get_logger().info("Executor running. Waiting for one target...")
            while rclpy.ok() and not self._done:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._target_ready and not self._busy:
                    self.send_pick_goal()
        except RuntimeError as e:
            # Allow clean shutdown instead of crash when motion fails (e.g., arm stuck or controller timeout)
            self.get_logger().error(f"Runtime error: {e}")


def main(args=None):
    rclpy.init(args=args)
    executor_node = ColorPickExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(executor_node)
    try:
        executor_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        executor_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
