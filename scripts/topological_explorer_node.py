#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
import tf2_ros
import cv2
import scipy.ndimage
import itertools
import heapq

class TopologicalExplorerNode(Node):
    def __init__(self):
        super().__init__('topological_explorer_node')
        
        # State definition
        # States: 'INITIALIZING', 'EXPLORING', 'EXPLORATION_COMPLETE', 'COVERAGE_PLANNING', 'COVERAGE_EXECUTION', 'COVERAGE_VERIFICATION', 'MISSION_COMPLETE'
        self.state = 'INITIALIZING'
        self.get_logger().info(f"Topological Explorer Node started. State: {self.state}")
        
        # TF listener to get robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_active = False
        self.nav_goal_handle = None
        self.nav_success = False
        
        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/topological_markers', 10)
        self.path_pub = self.create_publisher(Path, '/exploration_path', 10)
        
        # Grid variables
        self.latest_map = None
        
        # Mission variables
        self.robot_pose = None
        self.graph = {}
        self.target_zone = None
        self.target_frontier = None
        self.stc_path = []
        self.stc_index = 0
        self.blacklist = []
        self.last_map_process_time = self.get_clock().now()
        
        # Main state machine timer at 1Hz
        self.timer = self.create_timer(1.0, self.state_machine_loop)
        
    def get_robot_pose(self):
        try:
            # Lookup transform from map to base_link
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = PoseStamped()
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = 0.0
            pose.pose.orientation = t.transform.rotation
            return pose.pose.position
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return None

    def map_callback(self, msg):
        self.latest_map = msg

    def state_machine_loop(self):
        # 1. Update robot pose
        pose = self.get_robot_pose()
        if pose is not None:
            self.robot_pose = pose
            
        if self.latest_map is None or self.robot_pose is None:
            return
            
        # 2. Run state machine
        if self.state == 'INITIALIZING':
            self.state = 'EXPLORING'
            self.get_logger().info(f"Initialized. Transitioning to state: {self.state}")
            
        elif self.state == 'EXPLORING':
            # Check if Nav2 is currently executing a path/goal
            if self.nav_active:
                self.get_logger().info("Navigating to target...", throttle_duration_sec=5.0)
                if self.target_frontier is not None:
                    fy, fx = self.target_frontier["centroid"]
                    msg = self.latest_map
                    if msg is not None:
                        tx = fx * msg.info.resolution + msg.info.origin.position.x
                        ty = fy * msg.info.resolution + msg.info.origin.position.y
                        dist = math.hypot(self.robot_pose.x - tx, self.robot_pose.y - ty)
                        if dist < 0.45:
                            self.get_logger().info(f"Robot is close to target ({dist:.2f}m < 0.45m). Canceling goal to avoid orientation sticking.")
                            if self.nav_goal_handle is not None:
                                try:
                                    self.nav_goal_handle.cancel_goal_async()
                                except Exception:
                                    pass
                            
                            # Blacklist the current target frontier to avoid loop
                            self.blacklist.append((tx, ty))
                            self.nav_active = False
                return
                
            # Perform Topological Segmentation and Frontier Allocation
            self.process_topological_map()
            
            # Find best target zone and frontier
            target_pose = self.select_best_exploration_target()
            
            if target_pose is not None:
                self.get_logger().info(f"Target found in Zone {self.target_zone}: ({target_pose[0]:.2f}, {target_pose[1]:.2f})")
                # Plan path and execute
                path = self.plan_and_smooth_path(target_pose)
                if path is not None and len(path) > 0:
                    self.publish_path(path)
                    # Send final waypoint to Nav2
                    self.send_nav_goal(path[-1])
                else:
                    self.get_logger().warn("A* planning failed. Navigating directly to frontier centroid.")
                    self.send_nav_goal(target_pose)
            else:
                self.get_logger().info("No more frontiers found. Exploration Complete!")
                self.state = 'EXPLORATION_COMPLETE'
                
        elif self.state == 'EXPLORATION_COMPLETE':
            self.state = 'COVERAGE_PLANNING'
            self.get_logger().info(f"Transitioning to state: {self.state}")
            
        elif self.state == 'COVERAGE_PLANNING':
            self.get_logger().info("Starting STC Coverage Path Planning...")
            self.plan_coverage_path()
            if len(self.stc_path) > 0:
                self.get_logger().info(f"STC coverage path successfully generated with {len(self.stc_path)} waypoints.")
                self.stc_index = 0
                self.state = 'COVERAGE_EXECUTION'
            else:
                self.get_logger().error("STC Planning returned empty path. Skipping to verification.")
                self.state = 'COVERAGE_VERIFICATION'
                
        elif self.state == 'COVERAGE_EXECUTION':
            if self.nav_active:
                if self.stc_index > 0 and self.stc_index - 1 < len(self.stc_path):
                    target_wp = self.stc_path[self.stc_index - 1]
                    dist = math.hypot(self.robot_pose.x - target_wp[0], self.robot_pose.y - target_wp[1])
                    if dist < 0.40:
                        self.get_logger().info(f"Robot is close to coverage waypoint ({dist:.2f}m < 0.40m). Skipping to next.")
                        if self.nav_goal_handle is not None:
                            try:
                                self.nav_goal_handle.cancel_goal_async()
                            except Exception:
                                pass
                        self.nav_active = False
                return
                
            if self.stc_index < len(self.stc_path):
                target_wp = self.stc_path[self.stc_index]
                self.get_logger().info(f"Executing STC Coverage Waypoint {self.stc_index+1}/{len(self.stc_path)}: ({target_wp[0]:.2f}, {target_wp[1]:.2f})")
                self.send_nav_goal(target_wp)
                self.stc_index += 1
            else:
                self.get_logger().info("STC coverage path completed.")
                self.state = 'COVERAGE_VERIFICATION'
                
        elif self.state == 'COVERAGE_VERIFICATION':
            self.get_logger().info("Running Coverage Verification Pass...")
            # We verify coverage by checking the remaining unknown space in the grid
            msg = self.latest_map
            grid = np.array(msg.data, dtype=np.int8)
            total_free = np.sum(grid == 0)
            total_unknown = np.sum(grid == -1)
            coverage_percentage = (total_free / (total_free + total_unknown)) * 100 if (total_free + total_unknown) > 0 else 0.0
            self.get_logger().info(f"Verification Results: {coverage_percentage:.2f}% area explored and covered.")
            self.state = 'MISSION_COMPLETE'
            
        elif self.state == 'MISSION_COMPLETE':
            self.get_logger().info("MISSION ACCOMPLISHED! Topological Exploration and Full Coverage Complete.", throttle_duration_sec=10.0)

    def process_topological_map(self):
        now = self.get_clock().now()
        # Rate limit topological processing to avoid CPU bottleneck
        if (now - self.last_map_process_time).nanoseconds / 1e9 < 4.0:
            return
        self.last_map_process_time = now
        
        msg = self.latest_map
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # 1. Occupancy Grid Map extraction
        free_space = (grid == 0).astype(np.uint8)
        
        # 2. Distance Transform
        dt = scipy.ndimage.distance_transform_edt(free_space)
        dt_smooth = cv2.GaussianBlur(dt, (5, 5), 0)
        
        # 3. GVD (Skeletonization)
        free_bin = (free_space * 255).astype(np.uint8)
        skel = np.zeros_like(free_bin)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        img_temp = free_bin.copy()
        while True:
            eroded = cv2.erode(img_temp, element)
            temp = cv2.dilate(eroded, element)
            temp = cv2.subtract(img_temp, temp)
            skel = cv2.bitwise_or(skel, temp)
            img_temp = eroded.copy()
            if cv2.countNonZero(img_temp) == 0:
                break
                
        # 4. Room / Corridor / Door Detection (Distance-Transform Watershed)
        # Find local maxima as seeds for the room zones
        local_max = (dt_smooth == scipy.ndimage.maximum_filter(dt_smooth, size=19)) & (dt_smooth > 4.0)
        markers, num_features = scipy.ndimage.label(local_max)
        
        if num_features == 0:
            # Fallback if no local maxima found
            self.graph = {}
            return
            
        dt_neg = -dt_smooth
        dt_min, dt_max = dt_neg.min(), dt_neg.max()
        if dt_max > dt_min:
            dt_norm = ((dt_neg - dt_min) / (dt_max - dt_min) * 255).astype(np.uint8)
        else:
            dt_norm = np.zeros_like(dt_neg, dtype=np.uint8)
            
        img_3ch = cv2.merge([dt_norm, dt_norm, dt_norm])
        markers_32s = markers.astype(np.int32)
        cv2.watershed(img_3ch, markers_32s)
        
        # 5. Doorways (where GVD intersects watershed boundaries)
        boundaries = (markers_32s == -1) & (free_space == 1)
        doorway_mask = (skel > 0) & boundaries
        doorway_labels, num_doorways = scipy.ndimage.label(doorway_mask)
        
        # 6. Topological Graph Generation
        self.graph = {zone_id: {
            "centroid": (0.0, 0.0),
            "neighbors": set(),
            "area": 0,
            "cells": [],
            "frontiers": []
        } for zone_id in range(1, num_features + 1)}
        
        # Populate rooms
        for zone_id in range(1, num_features + 1):
            ys, xs = np.where(markers_32s == zone_id)
            if len(ys) > 0:
                self.graph[zone_id]["cells"] = list(zip(ys, xs))
                self.graph[zone_id]["centroid"] = (float(np.mean(ys)), float(np.mean(xs)))
                self.graph[zone_id]["area"] = len(ys)
                
        # Connect neighbors through doorways
        for k in range(1, num_doorways + 1):
            doorway_cell_mask = (doorway_labels == k)
            dilated_doorway = cv2.dilate(doorway_cell_mask.astype(np.uint8), np.ones((3, 3), np.uint8))
            adjacent_labels = np.unique(markers_32s[dilated_doorway > 0])
            adjacent_zones = [int(l) for l in adjacent_labels if 0 < l <= num_features]
            if len(adjacent_zones) >= 2:
                for u, v in itertools.combinations(adjacent_zones, 2):
                    self.graph[u]["neighbors"].add(v)
                    self.graph[v]["neighbors"].add(u)
                    
        # 7. Frontier Detection Inside Zone
        unknown_mask = (grid == -1).astype(np.uint8)
        dilated_unknown = cv2.dilate(unknown_mask, np.ones((3, 3), np.uint8))
        # Keep frontiers away from obstacles
        obstacle_mask = (grid > 50).astype(np.uint8)
        dilated_obstacles = cv2.dilate(obstacle_mask, np.ones((5, 5), np.uint8))
        frontier_mask = (dilated_unknown > 0) & (free_space > 0) & (dilated_obstacles == 0)
        
        frontier_labels, num_frontiers = scipy.ndimage.label(frontier_mask)
        
        # Gather valid frontiers
        valid_frontiers = []
        for f_id in range(1, num_frontiers + 1):
            f_ys, f_xs = np.where(frontier_labels == f_id)
            if len(f_ys) >= 4:  # minimum size
                f_centroid = (float(np.mean(f_ys)), float(np.mean(f_xs)))
                
                # Associate with closest zone
                best_zone = None
                min_d = float('inf')
                for z_id, z_data in self.graph.items():
                    if z_data["area"] > 0:
                        zy, zx = z_data["centroid"]
                        d = math.hypot(f_centroid[0] - zy, f_centroid[1] - zx)
                        if d < min_d:
                            min_d = d
                            best_zone = z_id
                            
                valid_frontiers.append({
                    "centroid": f_centroid,
                    "cells": list(zip(f_ys, f_xs)),
                    "zone_id": best_zone
                })
                
        # Associate frontiers to graph
        for f in valid_frontiers:
            z_id = f["zone_id"]
            if z_id in self.graph:
                self.graph[z_id]["frontiers"].append(f)
                
        # 8. Publish Topological RViz Markers
        self.publish_markers(num_features, doorway_mask, resolution, origin_x, origin_y)

    def select_best_exploration_target(self):
        if not self.graph or self.robot_pose is None:
            return None
            
        msg = self.latest_map
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        rx, ry = self.robot_pose.x, self.robot_pose.y
        rcx = int((rx - origin_x) / resolution)
        rcy = int((ry - origin_y) / resolution)
        
        best_zone = None
        best_score = -float('inf')
        
        # Filter frontiers based on blacklist first
        filtered_graph = {}
        for z_id, z_data in self.graph.items():
            valid_fs = []
            for f in z_data["frontiers"]:
                fy, fx = f["centroid"]
                wx = fx * resolution + origin_x
                wy = fy * resolution + origin_y
                is_bl = False
                for bx, by in self.blacklist:
                    if math.hypot(wx - bx, wy - by) < 0.8:
                        is_bl = True
                        break
                if not is_bl:
                    valid_fs.append(f)
            filtered_graph[z_id] = z_data.copy()
            filtered_graph[z_id]["frontiers"] = valid_fs
            
        # Zone Scoring
        for z_id, z_data in filtered_graph.items():
            if len(z_data["frontiers"]) > 0:
                zy, zx = z_data["centroid"]
                dist = math.hypot(zy - rcy, zx - rcx) * resolution
                # Score = frontier count bonus - distance penalty
                score = len(z_data["frontiers"]) * 15.0 - dist * 2.0
                if score > best_score:
                    best_score = score
                    best_zone = z_id
                    
        if best_zone is not None:
            self.target_zone = best_zone
            # Find closest frontier inside target zone
            frontiers = filtered_graph[best_zone]["frontiers"]
            best_f = None
            min_fd = float('inf')
            for f in frontiers:
                fy, fx = f["centroid"]
                fd = math.hypot(fy - rcy, fx - rcx)
                if fd < min_fd:
                    min_fd = fd
                    best_f = f
            if best_f is not None:
                self.target_frontier = best_f
                # Return world coordinates of the frontier centroid
                target_grid_y, target_grid_x = best_f["centroid"]
                wx = target_grid_x * resolution + origin_x
                wy = target_grid_y * resolution + origin_y
                return (wx, wy)
                
        return None

    def plan_and_smooth_path(self, target_pose):
        msg = self.latest_map
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        
        # Inflate obstacles for safe planning (about 0.3m)
        obstacle_mask = (grid > 50).astype(np.uint8)
        inflated_obstacles = cv2.dilate(obstacle_mask, np.ones((7, 7), np.uint8))
        
        # Grid start & goal
        start = (int((self.robot_pose.y - origin_y) / resolution), int((self.robot_pose.x - origin_x) / resolution))
        goal = (int((target_pose[1] - origin_y) / resolution), int((target_pose[0] - origin_x) / resolution))
        
        # A* Global Path Planning
        grid_path = self.astar_plan(inflated_obstacles, start, goal)
        if grid_path is None or len(grid_path) == 0:
            return None
            
        # Line-of-Sight Path Smoothing
        smoothed_grid_path = self.los_smooth(grid_path, inflated_obstacles)
        
        # Convert to world coordinates
        world_path = []
        for r, c in smoothed_grid_path:
            wx = c * resolution + origin_x
            wy = r * resolution + origin_y
            world_path.append((wx, wy))
            
        return world_path

    def astar_plan(self, obstacle_map, start, goal):
        height, width = obstacle_map.shape
        if not (0 <= start[0] < height and 0 <= start[1] < width):
            return None
        if not (0 <= goal[0] < height and 0 <= goal[1] < width):
            return None
            
        # If start is blocked, find nearest free cell
        if obstacle_map[start] == 1:
            start = self.find_nearest_free_cell(obstacle_map, start)
            if start is None:
                return None
        # If goal is blocked, find nearest free cell
        if obstacle_map[goal] == 1:
            goal = self.find_nearest_free_cell(obstacle_map, goal)
            if goal is None:
                return None
                
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
                if 0 <= neighbor[0] < height and 0 <= neighbor[1] < width:
                    if obstacle_map[neighbor] == 1:
                        continue
                        
                    tentative_g_score = g_score[current] + cost
                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        h = math.hypot(goal[0] - neighbor[0], goal[1] - neighbor[1])
                        f_score[neighbor] = tentative_g_score + h
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        
        return None

    def find_nearest_free_cell(self, obstacle_map, cell):
        height, width = obstacle_map.shape
        q = [cell]
        visited = {cell}
        while q:
            curr = q.pop(0)
            if obstacle_map[curr] == 0:
                return curr
            r, c = curr
            for nr, nc in [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]:
                if 0 <= nr < height and 0 <= nc < width and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    q.append((nr, nc))
        return None

    def los_smooth(self, path, obstacle_map):
        if len(path) < 3:
            return path
        smoothed = [path[0]]
        curr = 0
        while curr < len(path) - 1:
            next_idx = curr + 1
            for i in range(len(path) - 1, curr, -1):
                if self.check_los(path[curr], path[i], obstacle_map):
                    next_idx = i
                    break
            smoothed.append(path[next_idx])
            curr = next_idx
        return smoothed

    def check_los(self, p1, p2, obstacle_map):
        y1, x1 = p1
        y2, x2 = p2
        dy = abs(y2 - y1)
        dx = abs(x2 - x1)
        sy = 1 if y1 < y2 else -1
        sx = 1 if x1 < x2 else -1
        err = dx - dy
        y, x = y1, x1
        while True:
            if obstacle_map[y, x] == 1:
                return False
            if y == y2 and x == x2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return True

    def plan_coverage_path(self):
        msg = self.latest_map
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
        free_mask = (grid == 0).astype(np.uint8)
        
        # We need a super-cell size for STC
        # Waffle Waffle width is 0.5m. A super-cell should be 0.5m x 0.5m (10x10 grid cells)
        super_cell_size = 10 
        
        sh = height // super_cell_size
        sw = width // super_cell_size
        
        # Build super-grid
        super_grid = np.zeros((sh, sw), dtype=bool)
        for r in range(sh):
            for c in range(sw):
                sub = free_mask[r*super_cell_size:(r+1)*super_cell_size, c*super_cell_size:(c+1)*super_cell_size]
                if sub.size > 0 and np.all(sub == 1):
                    super_grid[r, c] = True
                    
        # Find start super-cell
        rx, ry = self.robot_pose.x, self.robot_pose.y
        start_rc = (int((ry - origin_y) / resolution), int((rx - origin_x) / resolution))
        start_super = (start_rc[0] // super_cell_size, start_rc[1] // super_cell_size)
        
        # Ensure start_super is in bounds and free
        if not (0 <= start_super[0] < sh and 0 <= start_super[1] < sw) or not super_grid[start_super[0], start_super[1]]:
            # Fallback to closest free super-cell
            min_d = float('inf')
            for r in range(sh):
                for c in range(sw):
                    if super_grid[r, c]:
                        d = math.hypot(r - start_super[0], c - start_super[1])
                        if d < min_d:
                            min_d = d
                            start_super = (r, c)
                            
        if not super_grid[start_super[0], start_super[1]]:
            self.get_logger().error("No free super-cells available for STC planning.")
            return
            
        # Spanning Tree via DFS
        visited = set()
        mst_edges = {}
        
        def dfs(node):
            visited.add(node)
            r, c = node
            neighbors = [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]
            for nr, nc in neighbors:
                if 0 <= nr < sh and 0 <= nc < sw:
                    if super_grid[nr, nc] and (nr, nc) not in visited:
                        if node not in mst_edges:
                            mst_edges[node] = set()
                        if (nr, nc) not in mst_edges:
                            mst_edges[(nr, nc)] = set()
                        mst_edges[node].add((nr, nc))
                        mst_edges[(nr, nc)].add(node)
                        dfs((nr, nc))
                        
        dfs(start_super)
        
        # Sub-cell path transitions
        sub_edges = {}
        for r in range(sh):
            for c in range(sw):
                if super_grid[r, c]:
                    # default clockwise loop inside super-cell
                    sub_edges[(r, c, 0)] = (r, c, 1)
                    sub_edges[(r, c, 1)] = (r, c, 2)
                    sub_edges[(r, c, 2)] = (r, c, 3)
                    sub_edges[(r, c, 3)] = (r, c, 0)
                    
        # Apply Spanning Tree breaks
        processed = set()
        for u, neighbors in mst_edges.items():
            ur, uc = u
            for v in neighbors:
                vr, vc = v
                edge_key = tuple(sorted([u, v]))
                if edge_key in processed:
                    continue
                processed.add(edge_key)
                
                if vr == ur and vc == uc + 1: # Right
                    sub_edges[(ur, uc, 1)] = (vr, vc, 0)
                    sub_edges[(vr, vc, 3)] = (ur, uc, 2)
                elif vr == ur + 1 and vc == uc: # Down
                    sub_edges[(ur, uc, 2)] = (vr, vc, 1)
                    sub_edges[(vr, vc, 0)] = (ur, uc, 3)
                elif vr == ur and vc == uc - 1: # Left
                    sub_edges[(ur, uc, 3)] = (vr, vc, 2)
                    sub_edges[(vr, vc, 1)] = (ur, uc, 0)
                elif vr == ur - 1 and vc == uc: # Up
                    sub_edges[(ur, uc, 0)] = (vr, vc, 3)
                    sub_edges[(vr, vc, 2)] = (ur, uc, 1)
                    
        # Traverse coverage loop
        curr = (start_super[0], start_super[1], 0)
        grid_stc_path = []
        visited_subs = set()
        
        def get_sub_coords(r, c, idx):
            if idx == 0:
                return (int(r*super_cell_size + super_cell_size/4), int(c*super_cell_size + super_cell_size/4))
            elif idx == 1:
                return (int(r*super_cell_size + super_cell_size/4), int(c*super_cell_size + 3*super_cell_size/4))
            elif idx == 2:
                return (int(r*super_cell_size + 3*super_cell_size/4), int(c*super_cell_size + 3*super_cell_size/4))
            else:
                return (int(r*super_cell_size + 3*super_cell_size/4), int(c*super_cell_size + super_cell_size/4))
                
        while curr not in visited_subs:
            visited_subs.add(curr)
            r, c, idx = curr
            grid_stc_path.append(get_sub_coords(r, c, idx))
            if curr in sub_edges:
                curr = sub_edges[curr]
            else:
                break
                
        # Convert to world coordinates
        self.stc_path = []
        for r, c in grid_stc_path:
            wx = c * resolution + origin_x
            wy = r * resolution + origin_y
            self.stc_path.append((wx, wy))
            
        # Publish STC path for visualization
        self.publish_stc_path()

    def send_nav_goal(self, target):
        self.nav_active = True
        self.nav_success = False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target[0]
        goal_msg.pose.pose.position.y = target[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f"Sending goal to Nav2: ({target[0]:.2f}, {target[1]:.2f})")
        self.nav_client.wait_for_server()
        
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.nav_goal_response_callback)

    def nav_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            self.nav_active = False
            return
            
        self.nav_goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 goal reached successfully.")
            self.nav_success = True
        else:
            self.get_logger().warn(f"Nav2 goal failed or canceled with status code: {status}")
            self.nav_success = False
            
        # Blacklist the current target frontier to avoid loop
        if self.target_frontier is not None:
            fy, fx = self.target_frontier["centroid"]
            msg = self.latest_map
            if msg is not None:
                wx = fx * msg.info.resolution + msg.info.origin.position.x
                wy = fy * msg.info.resolution + msg.info.origin.position.y
                self.blacklist.append((wx, wy))
                self.get_logger().info(f"Blacklisted target: ({wx:.2f}, {wy:.2f})")
            
        self.nav_active = False

    def publish_path(self, path_wpts):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for wp in path_wpts:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_stc_path(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for wp in self.stc_path:
            pose = PoseStamped()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_markers(self, num_features, doorway_mask, resolution, origin_x, origin_y):
        marker_array = MarkerArray()
        
        # Clear old markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        self.marker_pub.publish(marker_array)
        
        marker_array = MarkerArray()
        
        # 1. Publish Room Zone Centroids (Green Spheres)
        for zone_id, z_data in self.graph.items():
            if z_data["area"] > 0:
                zy, zx = z_data["centroid"]
                wx = zx * resolution + origin_x
                wy = zy * resolution + origin_y
                
                # Sphere Marker
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'room_centroids'
                marker.id = zone_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = wx
                marker.pose.position.y = wy
                marker.pose.position.z = 0.3
                marker.scale.x = 0.4
                marker.scale.y = 0.4
                marker.scale.z = 0.4
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                marker_array.markers.append(marker)
                
                # Text Marker
                text_marker = Marker()
                text_marker.header.frame_id = 'map'
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = 'room_labels'
                text_marker.id = zone_id + 1000
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = wx
                text_marker.pose.position.y = wy
                text_marker.pose.position.z = 0.6
                text_marker.scale.z = 0.3  # text size
                text_marker.text = f"Zone {zone_id}"
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                marker_array.markers.append(text_marker)
                
        # 2. Publish Doorways (Yellow Cylinders)
        doorway_ys, doorway_xs = np.where(doorway_mask)
        for idx, (dy, dx) in enumerate(zip(doorway_ys, doorway_xs)):
            wx = dx * resolution + origin_x
            wy = dy * resolution + origin_y
            
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'doorways'
            marker.id = idx + 2000
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = wx
            marker.pose.position.y = wy
            marker.pose.position.z = 0.1
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.8
            marker.color.b = 0.0
            marker.color.a = 0.7
            marker_array.markers.append(marker)
            
        # 3. Publish topological graph edges (Red lines connecting adjacent zones)
        edge_id = 0
        for zone_id, z_data in self.graph.items():
            if z_data["area"] > 0:
                zy1, zx1 = z_data["centroid"]
                wx1 = zx1 * resolution + origin_x
                wy1 = zy1 * resolution + origin_y
                
                for neighbor_id in z_data["neighbors"]:
                    # Draw each edge once
                    if neighbor_id > zone_id and neighbor_id in self.graph:
                        zy2, zx2 = self.graph[neighbor_id]["centroid"]
                        wx2 = zx2 * resolution + origin_x
                        wy2 = zy2 * resolution + origin_y
                        
                        line_marker = Marker()
                        line_marker.header.frame_id = 'map'
                        line_marker.header.stamp = self.get_clock().now().to_msg()
                        line_marker.ns = 'graph_edges'
                        line_marker.id = edge_id + 3000
                        line_marker.type = Marker.LINE_STRIP
                        line_marker.action = Marker.ADD
                        line_marker.scale.x = 0.05  # Line width
                        line_marker.color.r = 1.0
                        line_marker.color.g = 0.0
                        line_marker.color.b = 0.0
                        line_marker.color.a = 0.9
                        
                        p1 = Point()
                        p1.x, p1.y, p1.z = wx1, wy1, 0.3
                        p2 = Point()
                        p2.x, p2.y, p2.z = wx2, wy2, 0.3
                        
                        line_marker.points.append(p1)
                        line_marker.points.append(p2)
                        marker_array.markers.append(line_marker)
                        edge_id += 1
                        
        if len(marker_array.markers) > 0:
            self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = TopologicalExplorerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
