#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
from scipy.ndimage import label
import heapq

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')
        
        # Parameters
        self.declare_parameter('size_of_cell', 6)  # 6 grid cells = 0.3m sweep width
        self.size_of_cell = self.get_parameter('size_of_cell').value
        
        # TF listener to get robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/clean_robot/clean_path', 10)
        
        self.get_logger().info(f"ROS 2 CCPP Path Planning Node initialized. Cell size: {self.size_of_cell}")
        
    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def map_callback(self, msg):
        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().info("Waiting for robot pose...", throttle_duration_sec=5.0)
            return
            
        self.get_logger().info("Map received. Starting Complete Coverage Path Planning...")
        
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        width = msg.info.width
        height = msg.info.height
        
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # 1. Subsample map into CCPP cell grid
        D = self.size_of_cell
        sh = height // D
        sw = width // D
        
        cell_grid = np.ones((sh, sw), dtype=np.uint8) # 1 = occupied, 0 = free
        for r in range(sh):
            for c in range(sw):
                sub = grid[r*D:(r+1)*D, c*D:(c+1)*D]
                # Free if no obstacles (value > 50) and no unknown cells (value == -1)
                if sub.size > 0 and np.all(sub == 0):
                    cell_grid[r, c] = 0
                    
        # 2. Setup neural network grid
        neural_grid = np.zeros((sh, sw), dtype=np.float32)
        neural_grid[cell_grid == 1] = -100000.0
        
        # 3. Find robot start cell
        rx, ry = pose
        rcx = int((rx - origin_x) / resolution)
        rcy = int((ry - origin_y) / resolution)
        
        start_r = rcy // D
        start_c = rcx // D
        
        # Keep inside bounds
        start_r = max(0, min(start_r, sh - 1))
        start_c = max(0, min(start_c, sw - 1))
        
        # 4. Neural Network CCPP planning loop
        curr = (start_r, start_c)
        last_theta = 90.0
        path_cells = [curr]
        
        theta_vec = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]
        # Neighbor steps corresponding to theta_vec
        steps = [(0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1), (1, 0), (1, 1)]
        c_0 = 50.0
        
        for loop in range(5000):
            neural_grid[curr] = -250.0
            
            best_val = -float('inf')
            best_step = None
            best_theta = last_theta
            
            for idx, (dr, dc) in enumerate(steps):
                nr, nc = curr[0] + dr, curr[1] + dc
                if 0 <= nr < sh and 0 <= nc < sw:
                    # Angle alignment bonus
                    delta_theta = abs(theta_vec[idx] - last_theta)
                    if delta_theta > 180.0:
                        delta_theta = 360.0 - delta_theta
                    e = 1.0 - delta_theta / 180.0
                    
                    val = neural_grid[nr, nc] + c_0 * e
                    # Diagonal penalty
                    if idx in [1, 3, 5, 7]:
                        val -= 200.0
                        
                    if val > best_val:
                        best_val = val
                        best_step = (nr, nc)
                        best_theta = theta_vec[idx]
                        
            # If we found an unvisited free neighbor
            if best_val > -200.0 and best_step is not None:
                curr = best_step
                last_theta = best_theta
                path_cells.append(curr)
            else:
                # Backtrack using BFS to find closest unvisited free cell
                target = self.find_closest_unvisited(cell_grid, neural_grid, curr)
                if target is not None:
                    # Plan A* path to the target cell
                    astar_path = self.astar_grid(cell_grid, curr, target)
                    if astar_path is not None and len(astar_path) > 1:
                        # Append the intermediate path
                        path_cells.extend(astar_path[1:])
                    else:
                        path_cells.append(target)
                    curr = target
                else:
                    # No more unvisited reachable cells
                    break
                    
        self.get_logger().info(f"CCPP path planning complete. Generated {len(path_cells)} waypoints.")
        
        # 5. Convert cell path back to map world coordinates
        world_path = Path()
        world_path.header.stamp = self.get_clock().now().to_msg()
        world_path.header.frame_id = 'map'
        
        for r, c in path_cells:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = (c * D + D/2) * resolution + origin_x
            pose.pose.position.y = (r * D + D/2) * resolution + origin_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            world_path.poses.append(pose)
            
        self.path_pub.publish(world_path)

    def find_closest_unvisited(self, cell_grid, neural_grid, start):
        sh, sw = cell_grid.shape
        q = [start]
        visited = {start}
        
        while q:
            curr = q.pop(0)
            r, c = curr
            # Target is a cell that is free and has not been visited (activity is 0.0)
            if cell_grid[curr] == 0 and neural_grid[curr] == 0.0:
                return curr
                
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nr, nc = r + dr, c + dc
                if 0 <= nr < sh and 0 <= nc < sw and (nr, nc) not in visited:
                    if cell_grid[nr, nc] == 0:  # Must be traversable
                        visited.add((nr, nc))
                        q.append((nr, nc))
        return None

    def astar_grid(self, cell_grid, start, goal):
        sh, sw = cell_grid.shape
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: math.hypot(goal[0] - start[0], goal[1] - start[1])}
        
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        costs = [1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414]
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path
                
            for (dr, dc), cost in zip(neighbors, costs):
                neighbor = (current[0] + dr, current[1] + dc)
                if 0 <= neighbor[0] < sh and 0 <= neighbor[1] < sw:
                    if cell_grid[neighbor] == 1:
                        continue
                        
                    tentative_g_score = g_score[current] + cost
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        h = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                        f_score[neighbor] = tentative_g_score + h
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        
        return None

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
