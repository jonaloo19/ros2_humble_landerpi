import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # Subscriber to the map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # Subscriber to robot odometry for position updates
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Action client for navigation
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Visited frontiers set
        self.visited_frontiers = set()

        # Map and position data
        self.map_data = None
        self.robot_position = (1, 0)  # Placeholder, update from localization

        # 导航状态标志
        self.is_navigating = False
        self.first_exploration = True  # 首次探索标志

        # Timer for initial exploration - 仅用于首次启动
        self.create_timer(5.0, self.initial_explore)
        
        self.get_logger().info("✅ 智能探索模式已启用 - 到达目标后自动探索下一个前沿点")

    def map_callback(self, msg):
        self.map_data = msg
        self.get_logger().info("Map received")

    def odom_callback(self, msg):
        """更新机器人当前位置"""
        if self.map_data is None:
            return
        
        # 获取机器人在世界坐标系中的位置
        robot_x = msg.pose.pose.position.x
        robot_y = msg.pose.pose.position.y
        
        # 转换为地图栅格坐标
        robot_col = int((robot_x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        robot_row = int((robot_y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
        
        self.robot_position = (robot_row, robot_col)
        # self.get_logger().info(f"机器人位置更新: ({robot_row}, {robot_col})")

    def navigate_to(self, x, y):
        """
        Send navigation goal to Nav2.
        """
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0  # Facing forward

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg

        self.get_logger().info(f"🎯 导航到目标: x={x:.2f}, y={y:.2f}")
        self.is_navigating = True  # 标记为正在导航

        # Wait for the action server
        self.nav_to_pose_client.wait_for_server()

        # Send the goal and register a callback for the result
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the goal response and attach a callback to the result.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        """
        Callback to handle the result of the navigation action.
        """
        self.is_navigating = False  # 导航完成，重置标志
        
        try:
            result = future.result().result
            self.get_logger().info(f"✅ 导航完成！立即寻找下一个前沿点...")
            
            # 到达目标后，立即探索下一个前沿点
            self.explore()
            
        except Exception as e:
            self.get_logger().error(f"❌ 导航失败: {e}")
            # 即使失败也尝试探索下一个点
            self.get_logger().info("尝试寻找新的前沿点...")
            self.explore()

    def find_frontiers(self, map_array):
        """
        Detect frontiers in the occupancy grid map.
        """
        frontiers = []
        rows, cols = map_array.shape

        # Iterate through each cell in the map
        for r in range(1, rows - 1):
            for c in range(1, cols - 1):
                if map_array[r, c] == 0:  # Free cell
                    # Check if any neighbors are unknown
                    neighbors = map_array[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neighbors:
                        frontiers.append((r, c))

        self.get_logger().info(f"Found {len(frontiers)} frontiers")
        return frontiers

    def choose_frontier(self, frontiers):
        """
        Choose the closest frontier to the robot (with minimum distance filter).
        """
        robot_row, robot_col = self.robot_position
        min_distance = float('inf')
        chosen_frontier = None
        
        # 最小距离阈值（栅格单位）- 避免选择太近的前沿点
        MIN_DISTANCE_THRESHOLD = 20  # 约1米（假设分辨率0.05m，20格=1m）

        for frontier in frontiers:
            if frontier in self.visited_frontiers:
                continue

            distance = np.sqrt((robot_row - frontier[0])**2 + (robot_col - frontier[1])**2)
            
            # 过滤掉太近的前沿点
            if distance < MIN_DISTANCE_THRESHOLD:
                continue
            
            if distance < min_distance:
                min_distance = distance
                chosen_frontier = frontier

        if chosen_frontier:
            self.visited_frontiers.add(chosen_frontier)
            distance_meters = min_distance * self.map_data.info.resolution
            self.get_logger().info(f"✅ 选择前沿点: {chosen_frontier}, 距离: {distance_meters:.2f}米")
        else:
            self.get_logger().warning("⚠️  没有找到足够远的前沿点（可能都太近了）")

        return chosen_frontier

    def initial_explore(self):
        """首次探索 - 仅在启动时调用一次"""
        if self.first_exploration:
            self.first_exploration = False
            self.get_logger().info("🚀 开始首次探索...")
            self.explore()

    def explore(self):
        """探索函数 - 寻找并导航到前沿点"""
        # 如果正在导航中，跳过（避免打断）
        if self.is_navigating:
            self.get_logger().info("⏳ 正在导航中，跳过本次探索")
            return

        if self.map_data is None:
            self.get_logger().warning("⚠️  地图数据不可用")
            return

        # Convert map to numpy array
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width))

        # Detect frontiers
        frontiers = self.find_frontiers(map_array)

        if not frontiers:
            self.get_logger().info("🎉 没有发现前沿点 - 探索完成！")
            # 可选：探索完成后的操作
            # self.shutdown_robot()
            return

        # Choose the closest frontier
        chosen_frontier = self.choose_frontier(frontiers)

        if not chosen_frontier:
            self.get_logger().warning("⚠️  没有可用的前沿点")
            return

        # Convert the chosen frontier to world coordinates
        goal_x = chosen_frontier[1] * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = chosen_frontier[0] * self.map_data.info.resolution + self.map_data.info.origin.position.y

        # Navigate to the chosen frontier
        self.navigate_to(goal_x, goal_y)

    # def shudown_robot(self):
    #     
    #
    #
    #     self.get_logger().info("Shutting down robot exploration")


def main(args=None):
    rclpy.init(args=args)
    explorer_node = ExplorerNode()

    try:
        explorer_node.get_logger().info("Starting exploration...")
        rclpy.spin(explorer_node)
    except KeyboardInterrupt:
        explorer_node.get_logger().info("Exploration stopped by user")
    finally:
        explorer_node.destroy_node()
        rclpy.shutdown()
