#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Grasp action server - orchestrates complete pick operation
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from landerpi_interfaces.action import PickPose
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math

from .tf_utils import TFUtils
from .gripper_client import GripperClient
from .moveit2_client import MoveIt2Client


class GraspActionServer(Node):
    """Action server for object grasping using MoveIt2 and gripper control"""
    
    # Error codes
    ERROR_SUCCESS = 0
    ERROR_TF_FAILED = 1
    ERROR_IK_FAILED = 2
    ERROR_PLANNING_FAILED = 3
    ERROR_EXECUTION_FAILED = 4
    ERROR_GRIPPER_FAILED = 5
    
    def __init__(self):
        super().__init__('grasp_action_server')
        
        # Declare parameters with defaults
        self.declare_parameters(
            namespace='',
            parameters=[
                ('planning.planning_frame', 'base_footprint'),
                ('planning.group_name', 'arm'),
                ('planning.ee_link', 'end_effector_link'),
                ('planning.velocity_scaling', 0.25),
                ('planning.acceleration_scaling', 0.20),
                ('planning.planning_timeout', 3.0),
                ('planning.max_planning_retries', 2),
                ('grasp_offsets.approach', [0.0, 0.0, 0.12]),
                ('grasp_offsets.descend', [0.0, 0.0, -0.10]),
                ('grasp_offsets.lift', [0.0, 0.0, 0.15]),
                ('grasp_orientation.roll', 0.0),
                ('grasp_orientation.pitch', 1.57),
                ('grasp_orientation.yaw', 0.0),
                ('gripper.controller_name', 'gripper_controller'),
                ('gripper.joint_name', 'gripper_base_joint'),
                ('gripper.open_position', 0.0),
                ('gripper.close_position', -1.638),
                ('gripper.command_timeout', 2.0),
            ]
        )
        
        # Load parameters
        self._load_parameters()
        
        # Initialize components
        self.get_logger().info("Initializing components...")
        self.tf_utils = TFUtils(self)
        self.gripper_client = GripperClient(
            self,
            controller_name=self.gripper_controller_name,
            joint_name=self.gripper_joint_name,
            timeout=self.gripper_timeout
        )
        self.moveit_client = MoveIt2Client(
            self,
            group_name=self.group_name,
            planning_frame=self.planning_frame,
            ee_link=self.ee_link
        )
        
        # Wait for action servers
        if not self.gripper_client.wait_for_server(timeout=10.0):
            self.get_logger().error("Gripper action server not available!")
        
        if not self.moveit_client.wait_for_action_server(timeout=10.0):
            self.get_logger().error("MoveGroup or arm_controller not available!")
        
        # Create action server
        self.action_server = ActionServer(
            self,
            PickPose,
            '/arm/pick_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.get_logger().info("Grasp action server ready")
        self._log_configuration()
    
    def _load_parameters(self):
        """Load configuration parameters from YAML"""
        self.planning_frame = self.get_parameter('planning.planning_frame').value
        self.group_name = self.get_parameter('planning.group_name').value
        self.ee_link = self.get_parameter('planning.ee_link').value
        self.velocity_scaling = self.get_parameter('planning.velocity_scaling').value
        self.acceleration_scaling = self.get_parameter('planning.acceleration_scaling').value
        self.planning_timeout = self.get_parameter('planning.planning_timeout').value
        self.max_retries = self.get_parameter('planning.max_planning_retries').value
        
        self.approach_offset = self.get_parameter('grasp_offsets.approach').value
        self.descend_offset = self.get_parameter('grasp_offsets.descend').value
        self.lift_offset = self.get_parameter('grasp_offsets.lift').value
        
        self.grasp_roll = self.get_parameter('grasp_orientation.roll').value
        self.grasp_pitch = self.get_parameter('grasp_orientation.pitch').value
        self.grasp_yaw = self.get_parameter('grasp_orientation.yaw').value
        
        self.gripper_controller_name = self.get_parameter('gripper.controller_name').value
        self.gripper_joint_name = self.get_parameter('gripper.joint_name').value
        self.gripper_open_pos = self.get_parameter('gripper.open_position').value
        self.gripper_close_pos = self.get_parameter('gripper.close_position').value
        self.gripper_timeout = self.get_parameter('gripper.command_timeout').value
    
    def _log_configuration(self):
        """Log current configuration"""
        self.get_logger().info("=== Grasp Configuration ===")
        self.get_logger().info(f"Planning frame: {self.planning_frame}")
        self.get_logger().info(f"Group: {self.group_name}, EE: {self.ee_link}")
        self.get_logger().info(f"Velocity scaling: {self.velocity_scaling}")
        self.get_logger().info(f"Approach offset: {self.approach_offset}")
        self.get_logger().info(f"Descend offset: {self.descend_offset}")
        self.get_logger().info(f"Lift offset: {self.lift_offset}")
        self.get_logger().info(f"Grasp orientation (RPY): [{self.grasp_roll}, {self.grasp_pitch}, {self.grasp_yaw}]")
        self.get_logger().info(f"Gripper: open={self.gripper_open_pos}, close={self.gripper_close_pos}")
        self.get_logger().info("==========================")
    
    def goal_callback(self, goal_request):
        """Handle new goal request"""
        self.get_logger().info(f"Received grasp goal for object: '{goal_request.object_id}'")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle goal cancellation"""
        self.get_logger().info("Grasp goal cancelled")
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):
        """Execute grasp operation"""
        self.get_logger().info("=== Starting grasp execution ===")
        
        goal = goal_handle.request
        feedback = PickPose.Feedback()
        result = PickPose.Result()
        
        try:
            # Step 0: Move to home position (vertical up) for consistent starting pose
            self.get_logger().info("=== CRITICAL: Moving to HOME position first ===")
            feedback.stage = "moving_to_home"
            feedback.progress = 0.05
            goal_handle.publish_feedback(feedback)
            
            success, error_msg = self._move_to_home_position()
            if not success:
                # HOME initialization is MANDATORY, cannot proceed without it
                self.get_logger().error(f"CRITICAL: Failed to reach HOME position: {error_msg}")
                result.success = False
                result.error_code = self.ERROR_PLANNING_FAILED
                result.message = f"HOME initialization failed: {error_msg}"
                goal_handle.abort()
                return result
            
            self.get_logger().info("✓ Successfully reached HOME position, stabilizing for 1 second...")
            import time
            time.sleep(1.0)  # Wait 1 second at home position for stability
            
            # Step 1: Transform target pose to planning frame
            feedback.stage = "transforming_pose"
            feedback.progress = 0.1
            goal_handle.publish_feedback(feedback)
            
            target_pose = goal.target_pose
            self.get_logger().info(
                f"Target pose in '{target_pose.header.frame_id}': "
                f"[{target_pose.pose.position.x:.3f}, "
                f"{target_pose.pose.position.y:.3f}, "
                f"{target_pose.pose.position.z:.3f}]"
            )
            
            if target_pose.header.frame_id != self.planning_frame:
                try:
                    target_pose = self.tf_utils.transform_pose(target_pose, self.planning_frame)
                except (LookupException, ConnectivityException, ExtrapolationException) as e:
                    result.success = False
                    result.error_code = self.ERROR_TF_FAILED
                    result.message = f"TF transform failed: {str(e)}"
                    return result
            
            # Step 2: Move to safe approach position (directly above target at high altitude)
            # This ensures we approach from above, not from the side
            safe_approach_pose = self._compute_safe_approach_pose(target_pose)
            
            # Step 3: Compute grasp poses
            pregrasp_pose = self._compute_pregrasp_pose(target_pose)
            grasp_pose = self._compute_grasp_pose(pregrasp_pose)
            lift_pose = self._compute_lift_pose(grasp_pose)
            
            # Step 4: Open gripper
            feedback.stage = "opening_gripper"
            feedback.progress = 0.15
            goal_handle.publish_feedback(feedback)
            
            if not self.gripper_client.open_gripper(self.gripper_open_pos):
                result.success = False
                result.error_code = self.ERROR_GRIPPER_FAILED
                result.message = "Failed to open gripper"
                return result
            
            # Step 5: Move to safe approach position (high above target)
            feedback.stage = "safe_approach"
            feedback.progress = 0.25
            goal_handle.publish_feedback(feedback)
            
            success, error_msg = self.moveit_client.plan_and_execute(
                safe_approach_pose,
                self.velocity_scaling,
                self.acceleration_scaling,
                self.planning_timeout
            )
            
            if not success:
                result.success = False
                result.error_code = self.ERROR_PLANNING_FAILED
                result.message = f"Safe approach failed: {error_msg}"
                return result
            
            # Step 6: Move to pregrasp pose (vertical descent begins)
            feedback.stage = "planning_pregrasp"
            feedback.progress = 0.35
            goal_handle.publish_feedback(feedback)
            
            success, error_msg = self.moveit_client.plan_and_execute(
                pregrasp_pose,
                self.velocity_scaling,
                self.acceleration_scaling,
                self.planning_timeout
            )
            
            if not success:
                result.success = False
                result.error_code = self.ERROR_PLANNING_FAILED
                result.message = f"Pregrasp planning failed: {error_msg}"
                return result
            
            feedback.stage = "approaching"
            feedback.progress = 0.5
            goal_handle.publish_feedback(feedback)
            
            # Step 5: Move to grasp pose (descend)
            feedback.stage = "grasping"
            feedback.progress = 0.6
            goal_handle.publish_feedback(feedback)
            
            success, error_msg = self.moveit_client.plan_and_execute(
                grasp_pose,
                self.velocity_scaling * 0.5,  # Slower for precision
                self.acceleration_scaling * 0.5,
                self.planning_timeout
            )
            
            if not success:
                result.success = False
                result.error_code = self.ERROR_PLANNING_FAILED
                result.message = f"Grasp approach failed: {error_msg}"
                return result
            
            # Step 6: Close gripper
            feedback.stage = "closing_gripper"
            feedback.progress = 0.7
            goal_handle.publish_feedback(feedback)
            
            if not self.gripper_client.close_gripper(self.gripper_close_pos, duration=2.0):
                result.success = False
                result.error_code = self.ERROR_GRIPPER_FAILED
                result.message = "Failed to close gripper"
                return result
            
            # Wait for gripper to fully close and stabilize (critical for grasping)
            self.get_logger().info("Waiting for gripper to stabilize and grip object...")
            import time
            time.sleep(1.0)  # Additional 1.0s wait to ensure firm grip
            
            # Step 7: Lift object
            feedback.stage = "lifting"
            feedback.progress = 0.8
            goal_handle.publish_feedback(feedback)
            
            success, error_msg = self.moveit_client.plan_and_execute(
                lift_pose,
                self.velocity_scaling,
                self.acceleration_scaling,
                self.planning_timeout
            )
            
            if not success:
                result.success = False
                result.error_code = self.ERROR_EXECUTION_FAILED
                result.message = f"Lift failed: {error_msg}"
                return result
            
            # Success!
            feedback.stage = "completed"
            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)
            
            result.success = True
            result.error_code = self.ERROR_SUCCESS
            result.message = f"Successfully grasped object '{goal.object_id}'"
            self.get_logger().info(result.message)
            
            goal_handle.succeed()
            return result
            
        except Exception as e:
            self.get_logger().error(f"Grasp execution exception: {str(e)}")
            result.success = False
            result.error_code = self.ERROR_EXECUTION_FAILED
            result.message = f"Exception: {str(e)}"
            return result
    
    def _compute_safe_approach_pose(self, target_pose: PoseStamped) -> PoseStamped:
        """
        Compute safe approach pose (high altitude directly above target)
        This ensures the arm approaches from above, not from the side
        """
        safe_pose = PoseStamped()
        safe_pose.header = target_pose.header
        # Same X, Y as target, but much higher (25cm above target)
        safe_pose.pose.position.x = target_pose.pose.position.x
        safe_pose.pose.position.y = target_pose.pose.position.y
        safe_pose.pose.position.z = target_pose.pose.position.z + 0.25  # 25cm high
        
        # Set grasp orientation (gripper pointing down)
        safe_pose.pose.orientation = self._rpy_to_quaternion(
            self.grasp_roll, self.grasp_pitch, self.grasp_yaw
        )
        
        self.get_logger().info(
            f"Safe approach pose: [{safe_pose.pose.position.x:.3f}, "
            f"{safe_pose.pose.position.y:.3f}, {safe_pose.pose.position.z:.3f}]"
        )
        return safe_pose
    
    def _compute_pregrasp_pose(self, target_pose: PoseStamped) -> PoseStamped:
        """Compute pregrasp pose (approach position above target)"""
        pregrasp = PoseStamped()
        pregrasp.header = target_pose.header
        pregrasp.pose.position.x = target_pose.pose.position.x + self.approach_offset[0]
        pregrasp.pose.position.y = target_pose.pose.position.y + self.approach_offset[1]
        pregrasp.pose.position.z = target_pose.pose.position.z + self.approach_offset[2]
        
        # Set grasp orientation (gripper pointing down)
        pregrasp.pose.orientation = self._rpy_to_quaternion(
            self.grasp_roll, self.grasp_pitch, self.grasp_yaw
        )
        
        self.get_logger().info(
            f"Pregrasp pose: [{pregrasp.pose.position.x:.3f}, "
            f"{pregrasp.pose.position.y:.3f}, {pregrasp.pose.position.z:.3f}]"
        )
        return pregrasp
    
    def _compute_grasp_pose(self, pregrasp_pose: PoseStamped) -> PoseStamped:
        """Compute grasp pose (descend from pregrasp)"""
        grasp = PoseStamped()
        grasp.header = pregrasp_pose.header
        grasp.pose.position.x = pregrasp_pose.pose.position.x + self.descend_offset[0]
        grasp.pose.position.y = pregrasp_pose.pose.position.y + self.descend_offset[1]
        grasp.pose.position.z = pregrasp_pose.pose.position.z + self.descend_offset[2]
        grasp.pose.orientation = pregrasp_pose.pose.orientation
        
        self.get_logger().info(
            f"Grasp pose: [{grasp.pose.position.x:.3f}, "
            f"{grasp.pose.position.y:.3f}, {grasp.pose.position.z:.3f}]"
        )
        return grasp
    
    def _compute_lift_pose(self, grasp_pose: PoseStamped) -> PoseStamped:
        """Compute lift pose (raise after grasping)"""
        lift = PoseStamped()
        lift.header = grasp_pose.header
        lift.pose.position.x = grasp_pose.pose.position.x + self.lift_offset[0]
        lift.pose.position.y = grasp_pose.pose.position.y + self.lift_offset[1]
        lift.pose.position.z = grasp_pose.pose.position.z + self.lift_offset[2]
        lift.pose.orientation = grasp_pose.pose.orientation
        
        self.get_logger().info(
            f"Lift pose: [{lift.pose.position.x:.3f}, "
            f"{lift.pose.position.y:.3f}, {lift.pose.position.z:.3f}]"
        )
        return lift
    
    def _move_to_home_position(self) -> tuple:
        """
        Move arm to home position (all joints = 0, vertical up)
        This is a CRITICAL operation that ensures consistent starting state.
        Uses a two-step approach if direct planning fails:
        1. Move to intermediate "safe" position
        2. Move to HOME from there
        
        Returns:
            Tuple of (success, error_message)
        """
        try:
            # Home position: all joints at 0 rad (vertical upward)
            home_joints = [0.0, 0.0, 0.0, 0.0, 0.0]
            # Intermediate "safe" position: slightly raised, less extreme
            intermediate_joints = [0.0, -0.5, 1.0, -0.5, 0.0]
            
            self.get_logger().info("=" * 60)
            self.get_logger().info("INITIALIZING: Moving to HOME position [0, 0, 0, 0, 0]")
            self.get_logger().info("This ensures consistent starting state for grasping")
            self.get_logger().info("=" * 60)
            
            # === DIAGNOSTIC: Get current joint state ===
            self.get_logger().info("🔍 DIAGNOSTIC: Reading current joint state...")
            try:
                from sensor_msgs.msg import JointState
                current_joint_msg = None
                
                def joint_state_callback(msg):
                    nonlocal current_joint_msg
                    current_joint_msg = msg
                
                joint_sub = self.create_subscription(
                    JointState,
                    '/joint_states',
                    joint_state_callback,
                    10
                )
                
                # Wait for joint state message
                import time
                timeout = 2.0
                start_time = time.time()
                while current_joint_msg is None and (time.time() - start_time) < timeout:
                    import rclpy
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                if current_joint_msg is not None:
                    # Extract arm joint positions
                    arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
                    current_positions = []
                    for joint_name in arm_joint_names:
                        if joint_name in current_joint_msg.name:
                            idx = current_joint_msg.name.index(joint_name)
                            current_positions.append(current_joint_msg.position[idx])
                    
                    if len(current_positions) == 5:
                        self.get_logger().info(f"📍 CURRENT joint positions:")
                        self.get_logger().info(f"   joint1 = {current_positions[0]:+.3f} rad ({current_positions[0]*57.3:+.1f}°)")
                        self.get_logger().info(f"   joint2 = {current_positions[1]:+.3f} rad ({current_positions[1]*57.3:+.1f}°)")
                        self.get_logger().info(f"   joint3 = {current_positions[2]:+.3f} rad ({current_positions[2]*57.3:+.1f}°)")
                        self.get_logger().info(f"   joint4 = {current_positions[3]:+.3f} rad ({current_positions[3]*57.3:+.1f}°)")
                        self.get_logger().info(f"   joint5 = {current_positions[4]:+.3f} rad ({current_positions[4]*57.3:+.1f}°)")
                    else:
                        self.get_logger().warn(f"⚠️  Could not read all 5 joints (got {len(current_positions)})")
                else:
                    self.get_logger().warn("⚠️  No joint state received within timeout")
                
                self.destroy_subscription(joint_sub)
                
            except Exception as e:
                self.get_logger().warn(f"⚠️  Failed to read current joint state: {e}")
            
            self.get_logger().info("🎯 TARGET positions:")
            self.get_logger().info(f"   Intermediate: {intermediate_joints}")
            self.get_logger().info(f"   HOME:         {home_joints}")
            self.get_logger().info("=" * 60)
            
            # Use longer timeout for HOME movement (initial pose may be far from HOME)
            home_timeout = max(self.planning_timeout * 2, 6.0)  # At least 6 seconds
            
            # Attempt 1: Direct move to HOME
            self.get_logger().info("=" * 60)
            self.get_logger().info("🎯 ATTEMPT 1: Direct planning to HOME [0, 0, 0, 0, 0]")
            self.get_logger().info(f"   Planning timeout: {home_timeout}s")
            self.get_logger().info(f"   Velocity scaling: {self.velocity_scaling * 0.5:.2f}")
            self.get_logger().info(f"   Acceleration scaling: {self.acceleration_scaling * 0.5:.2f}")
            self.get_logger().info("=" * 60)
            
            success, error_msg = self.moveit_client.plan_and_execute_joints(
                home_joints,
                self.velocity_scaling * 0.5,  # Slower velocity for safety
                self.acceleration_scaling * 0.5,  # Slower acceleration
                home_timeout
            )
            
            if success:
                self.get_logger().info("=" * 60)
                self.get_logger().info("✅ HOME position reached successfully (direct)!")
                self.get_logger().info("=" * 60)
                return (True, "")
            
            # Attempt 2: Two-step approach via intermediate position
            self.get_logger().error("=" * 60)
            self.get_logger().error(f"❌ ATTEMPT 1 FAILED: {error_msg}")
            self.get_logger().error("=" * 60)
            self.get_logger().info("=" * 60)
            self.get_logger().info("🎯 ATTEMPT 2: Two-step approach via intermediate position")
            self.get_logger().info(f"   Step 2a: Current → Intermediate {intermediate_joints}")
            self.get_logger().info(f"   Step 2b: Intermediate → HOME {home_joints}")
            self.get_logger().info("=" * 60)
            
            # Step 2a: Move to intermediate position
            self.get_logger().info("─" * 60)
            self.get_logger().info("📍 STEP 2a: Moving to INTERMEDIATE position")
            self.get_logger().info(f"   Target: {intermediate_joints}")
            self.get_logger().info(f"   joint1 = {intermediate_joints[0]:+.3f} rad ({intermediate_joints[0]*57.3:+.1f}°)")
            self.get_logger().info(f"   joint2 = {intermediate_joints[1]:+.3f} rad ({intermediate_joints[1]*57.3:+.1f}°) ←向后仰")
            self.get_logger().info(f"   joint3 = {intermediate_joints[2]:+.3f} rad ({intermediate_joints[2]*57.3:+.1f}°) ← 向上抬")
            self.get_logger().info(f"   joint4 = {intermediate_joints[3]:+.3f} rad ({intermediate_joints[3]*57.3:+.1f}°) ← 补偿")
            self.get_logger().info(f"   joint5 = {intermediate_joints[4]:+.3f} rad ({intermediate_joints[4]*57.3:+.1f}°)")
            self.get_logger().info(f"   Velocity: {self.velocity_scaling * 0.3:.2f}x (VERY SLOW)")
            self.get_logger().info("─" * 60)
            
            success_inter, error_inter = self.moveit_client.plan_and_execute_joints(
                intermediate_joints,
                self.velocity_scaling * 0.3,  # Very slow for safety
                self.acceleration_scaling * 0.3,
                home_timeout
            )
            
            if not success_inter:
                final_error = f"Failed at intermediate position: {error_inter}"
                self.get_logger().error("=" * 60)
                self.get_logger().error(f"❌ STEP 2a FAILED: {final_error}")
                self.get_logger().error("=" * 60)
                self.get_logger().error("🔍 POSSIBLE REASONS:")
                self.get_logger().error("   1. Intermediate joints out of joint limits")
                self.get_logger().error("   2. MoveIt2 cannot find valid path")
                self.get_logger().error("   3. Planning timeout (current: {home_timeout}s)")
                self.get_logger().error("   4. Start state violates collision constraints")
                self.get_logger().error("=" * 60)
                return (False, final_error)
            
            self.get_logger().info("=" * 60)
            self.get_logger().info("✅ STEP 2a SUCCESS: Reached intermediate position!")
            self.get_logger().info("   Pausing for 0.5s to stabilize...")
            self.get_logger().info("=" * 60)
            import time
            time.sleep(0.5)  # Brief pause
            
            # Step 2b: Move from intermediate to HOME
            self.get_logger().info("─" * 60)
            self.get_logger().info("📍 STEP 2b: Moving from INTERMEDIATE to HOME")
            self.get_logger().info(f"   Target: {home_joints} (all zeros - vertical up)")
            self.get_logger().info(f"   Velocity: {self.velocity_scaling * 0.5:.2f}x")
            self.get_logger().info("─" * 60)
            
            success_home, error_home = self.moveit_client.plan_and_execute_joints(
                home_joints,
                self.velocity_scaling * 0.5,
                self.acceleration_scaling * 0.5,
                home_timeout
            )
            
            if success_home:
                self.get_logger().info("=" * 60)
                self.get_logger().info("✅ HOME POSITION REACHED! (two-step approach)")
                self.get_logger().info("   Robot arm is now vertical upward [0, 0, 0, 0, 0]")
                self.get_logger().info("=" * 60)
                return (True, "")
            else:
                final_error = f"Failed at final HOME step: {error_home}"
                self.get_logger().error("=" * 60)
                self.get_logger().error(f"❌ STEP 2b FAILED: {final_error}")
                self.get_logger().error("=" * 60)
                self.get_logger().error("🔍 POSSIBLE REASONS:")
                self.get_logger().error("   1. Path from intermediate to HOME still touches ground")
                self.get_logger().error("   2. MoveIt2 planning timeout")
                self.get_logger().error("   3. Intermediate position not safe enough")
                self.get_logger().error("=" * 60)
                return (False, final_error)
                
        except Exception as e:
            error = f"Exception during HOME movement: {str(e)}"
            self.get_logger().error(f"✗ {error}")
            self.get_logger().error("=" * 60)
            return (False, error)
    
    def _rpy_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        """Convert roll-pitch-yaw to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q


def main(args=None):
    rclpy.init(args=args)
    
    server = GraspActionServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
