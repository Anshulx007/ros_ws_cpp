#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import tf2_ros

class NextGoalNode(Node):
    def __init__(self):
        super().__init__('next_goal')
        
        # Parameters
        self.declare_parameter('tolerance_goal', 0.35)  # 0.35m tolerance
        self.tolerance_goal = self.get_parameter('tolerance_goal').value
        
        # TF listener to get robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_active = False
        self.nav_goal_handle = None
        
        # Subscriptions
        self.path_sub = self.create_subscription(Path, '/clean_robot/clean_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Path history & state
        self.planned_path = []
        self.current_idx = 0
        self.robot_pose = None
        self.new_path_received = False
        
        # Timer for goal management at 5Hz
        self.timer = self.create_timer(0.2, self.control_loop)
        
        self.get_logger().info(f"ROS 2 next_goal waypoint tracker initialized. Goal tolerance: {self.tolerance_goal}m")
        
    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def odom_callback(self, msg):
        pass  # We use TF lookup for exact map-frame positioning, but subscribe to odom to ensure compatibility

    def path_callback(self, msg):
        self.get_logger().info(f"Received new coverage path with {len(msg.poses)} waypoints.")
        self.planned_path = []
        for p in msg.poses:
            self.planned_path.append((p.pose.position.x, p.pose.position.y))
            
        self.current_idx = 0
        self.new_path_received = True
        self.nav_active = False
        
        if self.nav_goal_handle is not None:
            try:
                self.nav_goal_handle.cancel_goal_async()
            except Exception:
                pass

    def control_loop(self):
        # Update robot pose
        pose = self.get_robot_pose()
        if pose is not None:
            self.robot_pose = pose
            
        if self.robot_pose is None or len(self.planned_path) == 0:
            return
            
        rx, ry = self.robot_pose
        
        if self.current_idx < len(self.planned_path):
            target_x, target_y = self.planned_path[self.current_idx]
            dist = math.hypot(rx - target_x, ry - target_y)
            
            # Check if waypoint reached (or close enough)
            if dist <= self.tolerance_goal:
                self.get_logger().info(f"Waypoint {self.current_idx+1}/{len(self.planned_path)} reached.")
                self.current_idx += 1
                self.nav_active = False
                
                if self.current_idx >= len(self.planned_path):
                    self.get_logger().info("Coverage path completed successfully!")
                    return
                    
            # Send next goal if not active
            if not self.nav_active:
                # Retrieve current target (since index might have been incremented)
                if self.current_idx < len(self.planned_path):
                    next_x, next_y = self.planned_path[self.current_idx]
                    self.send_nav_goal(next_x, next_y)
        else:
            self.get_logger().info("All waypoints visited.", throttle_duration_sec=10.0)

    def send_nav_goal(self, x, y):
        self.nav_active = True
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f"Navigating to waypoint {self.current_idx+1}/{len(self.planned_path)}: ({x:.2f}, {y:.2f})")
        self.nav_client.wait_for_server()
        
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2. Retrying...")
            self.nav_active = False
            return
            
        self.nav_goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            # Succeeded, wait for control loop to increment waypoint index
            pass
        else:
            # If failed or canceled, we also consider it handled to move on to next waypoint or retry
            self.nav_active = False

def main(args=None):
    rclpy.init(args=args)
    node = NextGoalNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
