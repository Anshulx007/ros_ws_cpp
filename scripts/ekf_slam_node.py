#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

# EKF parameters matching PythonRobotics
Cx = np.diag([0.1, 0.1, np.deg2rad(5.0)]) ** 2  # EKF state covariance
M_DIST_TH = 1.5  # Mahalanobis distance threshold for data association
STATE_SIZE = 3
LM_SIZE = 2

def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class EKFSLAMNode(Node):
    def __init__(self):
        super().__init__('ekf_slam_node')
        
        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
            
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/ekf_slam_pose', 10)
        self.path_pub = self.create_publisher(Path, '/ekf_slam_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/ekf_landmarks', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # EKF State Variables
        self.xEst = np.zeros((STATE_SIZE, 1))
        self.PEst = np.eye(STATE_SIZE) * 0.1
        
        # Timing and motion input variables
        self.last_time = self.get_clock().now()
        self.v = 0.0
        self.yaw_rate = 0.0
        self.latest_scan = None
        
        # Path history
        self.path = Path()
        self.path.header.frame_id = 'map'
        
        # Timer for EKF update loop at 10Hz
        self.timer = self.create_timer(0.1, self.update_loop)
        
        self.get_logger().info("EKF SLAM Node (PythonRobotics) started successfully.")

    def odom_callback(self, msg):
        self.v = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z

    def scan_callback(self, msg):
        self.latest_scan = msg

    def update_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        
        # Prevent division by zero or extremely large dt
        if dt <= 0.0 or dt > 1.0:
            dt = 0.1
            
        # 1. Predict step
        u = np.array([[self.v, self.yaw_rate]]).T
        self.predict(u, dt)
        
        # 2. Update step if scan data is available
        if self.latest_scan is not None:
            landmarks = self.extract_landmarks(self.latest_scan)
            if len(landmarks) > 0:
                self.update(landmarks)
                
        # 3. Publish results
        self.publish_results(now)

    def motion_model(self, x, u, dt):
        F = np.array([[1.0, 0, 0],
                      [0, 1.0, 0],
                      [0, 0, 1.0]])
        B = np.array([[dt * math.cos(x[2, 0]), 0],
                      [dt * math.sin(x[2, 0]), 0],
                      [0.0, dt]])
        return (F @ x) + (B @ u)

    def calc_n_lm(self):
        return int((len(self.xEst) - STATE_SIZE) / LM_SIZE)

    def predict(self, u, dt):
        nLM = self.calc_n_lm()
        Fx = np.hstack((np.eye(STATE_SIZE), np.zeros((STATE_SIZE, LM_SIZE * nLM))))
        
        jF = np.array([[0.0, 0.0, -dt * u[0, 0] * math.sin(self.xEst[2, 0])],
                       [0.0, 0.0, dt * u[0, 0] * math.cos(self.xEst[2, 0])],
                       [0.0, 0.0, 0.0]], dtype=float)
                       
        G = np.eye(len(self.xEst)) + Fx.T @ jF @ Fx
        
        self.xEst[0:STATE_SIZE] = self.motion_model(self.xEst[0:STATE_SIZE], u, dt)
        self.PEst = G.T @ self.PEst @ G + Fx.T @ Cx @ Fx

    def extract_landmarks(self, scan):
        # Extract clusters from raw scan ranges
        angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
        ranges = np.array(scan.ranges)
        
        # Filter valid readings
        valid = (ranges > scan.range_min) & (ranges < scan.range_max) & (ranges < 4.0)
        valid_ranges = ranges[valid]
        valid_angles = angles[valid]
        
        if len(valid_ranges) == 0:
            return []
            
        # Convert to 2D points in robot's local frame
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)
        
        clusters = []
        current_cluster = []
        
        for i in range(len(x_points)):
            if len(current_cluster) == 0:
                current_cluster.append((x_points[i], y_points[i]))
            else:
                prev_pt = current_cluster[-1]
                dist = math.hypot(x_points[i] - prev_pt[0], y_points[i] - prev_pt[1])
                if dist < 0.25:
                    current_cluster.append((x_points[i], y_points[i]))
                else:
                    clusters.append(current_cluster)
                    current_cluster = [(x_points[i], y_points[i])]
        if len(current_cluster) > 0:
            clusters.append(current_cluster)
            
        landmarks = []
        for cluster in clusters:
            # We want point-like features (e.g. pillars/corners)
            if len(cluster) >= 3 and len(cluster) <= 15:
                pts = np.array(cluster)
                # Compute cluster width
                width = np.linalg.norm(pts[0] - pts[-1])
                if width < 0.6:  # Pillar width
                    centroid = np.mean(pts, axis=0)
                    r = np.linalg.norm(centroid)
                    b = math.atan2(centroid[1], centroid[0])
                    landmarks.append([r, b])
        return landmarks

    def update(self, landmarks):
        initP = np.eye(2) * 0.1
        
        for z in landmarks:
            min_id = self.search_correspond_landmark_id(z)
            nLM = self.calc_n_lm()
            
            if min_id == nLM:
                # Add new landmark to state vector and covariance matrix
                self.xEst = np.vstack((self.xEst, self.calc_landmark_position(z)))
                self.PEst = np.vstack((
                    np.hstack((self.PEst, np.zeros((len(self.PEst), LM_SIZE)))),
                    np.hstack((np.zeros((LM_SIZE, len(self.PEst))), initP))
                ))
                
            lm = self.get_landmark_position_from_state(min_id)
            y, S, H = self.calc_innovation(lm, z, min_id)
            
            # Kalman gain
            K = (self.PEst @ H.T) @ np.linalg.inv(S)
            self.xEst = self.xEst + (K @ y)
            self.PEst = (np.eye(len(self.xEst)) - (K @ H)) @ self.PEst
            
        self.xEst[2] = pi_2_pi(self.xEst[2])

    def calc_landmark_position(self, z):
        zp = np.zeros((2, 1))
        zp[0, 0] = self.xEst[0, 0] + z[0] * math.cos(self.xEst[2, 0] + z[1])
        zp[1, 0] = self.xEst[1, 0] + z[0] * math.sin(self.xEst[2, 0] + z[1])
        return zp

    def get_landmark_position_from_state(self, ind):
        return self.xEst[STATE_SIZE + LM_SIZE * ind: STATE_SIZE + LM_SIZE * (ind + 1), :]

    def search_correspond_landmark_id(self, z):
        nLM = self.calc_n_lm()
        min_dist = []
        
        for i in range(nLM):
            lm = self.get_landmark_position_from_state(i)
            y, S, _ = self.calc_innovation(lm, z, i)
            min_dist.append((y.T @ np.linalg.inv(S) @ y)[0, 0])
            
        min_dist.append(M_DIST_TH)
        return min_dist.index(min(min_dist))

    def calc_innovation(self, lm, z, LMid):
        delta = lm - self.xEst[0:2]
        q = (delta.T @ delta)[0, 0]
        z_angle = math.atan2(delta[1, 0], delta[0, 0]) - self.xEst[2, 0]
        zp = np.array([[math.sqrt(q), pi_2_pi(z_angle)]])
        y = (np.array([z]).reshape(2, 1) - zp.T)
        y[1] = pi_2_pi(y[1])
        H = self.jacob_h(q, delta, LMid + 1)
        S = H @ self.PEst @ H.T + Cx[0:2, 0:2]
        return y, S, H

    def jacob_h(self, q, delta, i):
        sq = math.sqrt(q)
        G = np.array([[-sq * delta[0, 0], -sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                      [delta[1, 0], -delta[0, 0], -q, -delta[1, 0], delta[0, 0]]])
        G = G / q
        nLM = self.calc_n_lm()
        F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
        F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                        np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))
        F = np.vstack((F1, F2))
        return G @ F

    def publish_results(self, time_msg):
        # Publish estimated pose
        pose = PoseStamped()
        pose.header.stamp = time_msg.to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.xEst[0, 0]
        pose.pose.position.y = self.xEst[1, 0]
        pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        yaw = self.xEst[2, 0]
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.pose_pub.publish(pose)
        
        # Publish path
        self.path.header.stamp = time_msg.to_msg()
        self.path.poses.append(pose)
        if len(self.path.poses) > 500:
            self.path.poses.pop(0)
        self.path_pub.publish(self.path)
        
        # Publish TF transform from map to odom_ekf
        t = TransformStamped()
        t.header.stamp = time_msg.to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom_ekf'
        t.transform.translation.x = self.xEst[0, 0]
        t.transform.translation.y = self.xEst[1, 0]
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        # Publish estimated landmarks as visualization markers in RViz
        marker_array = MarkerArray()
        nLM = self.calc_n_lm()
        
        # Delete old markers first
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        self.marker_pub.publish(marker_array)
        
        marker_array = MarkerArray()
        for i in range(nLM):
            lm_x = self.xEst[STATE_SIZE + 2 * i, 0]
            lm_y = self.xEst[STATE_SIZE + 2 * i + 1, 0]
            
            marker = Marker()
            marker.header.stamp = time_msg.to_msg()
            marker.header.frame_id = 'map'
            marker.ns = 'ekf_landmarks'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = lm_x
            marker.pose.position.y = lm_y
            marker.pose.position.z = 0.5
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)
            
        if len(marker_array.markers) > 0:
            self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = EKFSLAMNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
