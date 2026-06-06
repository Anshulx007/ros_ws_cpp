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

# FastSLAM 2.0 Parameters
Q_COV = np.diag([0.2, np.deg2rad(5.0)]) ** 2  # Measurement noise covariance
R_COV = np.diag([0.05, np.deg2rad(2.0)]) ** 2  # Motion noise covariance

MAX_LANDMARKS = 100
N_PARTICLE = 15
NTH = N_PARTICLE / 1.5
M_DIST_TH = 2.0  # Mahalanobis distance threshold

def pi_2_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class Particle:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.P = np.eye(3) * 0.1
        self.lm = np.zeros((MAX_LANDMARKS, 2))
        self.lmP = np.zeros((MAX_LANDMARKS * 2, 2))
        self.w = 1.0 / N_PARTICLE
        self.n_lm = 0

class FastSLAM2Node(Node):
    def __init__(self):
        super().__init__('fast_slam2_node')
        
        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
            
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/fast_slam_pose', 10)
        self.path_pub = self.create_publisher(Path, '/fast_slam_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/fast_slam_landmarks', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Initialize particles
        self.particles = [Particle() for _ in range(N_PARTICLE)]
        
        # Motion variables
        self.last_time = self.get_clock().now()
        self.v = 0.0
        self.yaw_rate = 0.0
        self.latest_scan = None
        
        # Path history
        self.path = Path()
        self.path.header.frame_id = 'map'
        
        # Timer for FastSLAM update loop at 10Hz
        self.timer = self.create_timer(0.1, self.update_loop)
        
        self.get_logger().info("FastSLAM 2.0 Node (PythonRobotics) started successfully.")

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
            
        u = np.array([[self.v, self.yaw_rate]]).T
        
        # 1. Predict step (motion prediction for all particles)
        self.predict_particles(u, dt)
        
        # 2. Update step with observation
        if self.latest_scan is not None:
            landmarks = self.extract_landmarks(self.latest_scan)
            if len(landmarks) > 0:
                self.update_with_observation(landmarks)
                self.resampling()
                
        # 3. Publish results
        self.publish_results(now)

    def motion_model(self, px, u, dt):
        F = np.array([[1.0, 0, 0],
                      [0, 1.0, 0],
                      [0, 0, 1.0]])
        B = np.array([[dt * math.cos(px[2, 0]), 0],
                      [dt * math.sin(px[2, 0]), 0],
                      [0.0, dt]])
        return (F @ px) + (B @ u)

    def predict_particles(self, u, dt):
        for i in range(N_PARTICLE):
            px = np.zeros((3, 1))
            px[0, 0] = self.particles[i].x
            px[1, 0] = self.particles[i].y
            px[2, 0] = self.particles[i].yaw
            ud = u + (np.random.randn(1, 2) @ R_COV ** 0.5).T  # add noise
            px = self.motion_model(px, ud, dt)
            self.particles[i].x = px[0, 0]
            self.particles[i].y = px[1, 0]
            self.particles[i].yaw = pi_2_pi(px[2, 0])

    def extract_landmarks(self, scan):
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
            if len(cluster) >= 3 and len(cluster) <= 15:
                pts = np.array(cluster)
                width = np.linalg.norm(pts[0] - pts[-1])
                if width < 0.6:  # Pillar width
                    centroid = np.mean(pts, axis=0)
                    r = np.linalg.norm(centroid)
                    b = math.atan2(centroid[1], centroid[0])
                    landmarks.append([r, b])
        return landmarks

    def add_new_lm(self, particle, z, lm_id):
        r = z[0]
        b = z[1]
        s = math.sin(pi_2_pi(particle.yaw + b))
        c = math.cos(pi_2_pi(particle.yaw + b))
        
        particle.lm[lm_id, 0] = particle.x + r * c
        particle.lm[lm_id, 1] = particle.y + r * s
        
        # Covariance
        dx = r * c
        dy = r * s
        d2 = dx ** 2 + dy ** 2
        d = math.sqrt(d2)
        Gz = np.array([[dx / d, dy / d],
                       [-dy / d2, dx / d2]])
        try:
            particle.lmP[2 * lm_id:2 * lm_id + 2] = np.linalg.inv(Gz) @ Q_COV @ np.linalg.inv(Gz.T)
        except np.linalg.linalg.LinAlgError:
            particle.lmP[2 * lm_id:2 * lm_id + 2] = np.eye(2) * 0.1
        return particle

    def compute_jacobians(self, particle, xf, Pf):
        dx = xf[0, 0] - particle.x
        dy = xf[1, 0] - particle.y
        d2 = dx ** 2 + dy ** 2
        d = math.sqrt(d2)
        
        zp = np.array([d, pi_2_pi(math.atan2(dy, dx) - particle.yaw)]).reshape(2, 1)
        Hv = np.array([[-dx / d, -dy / d, 0.0],
                       [dy / d2, -dx / d2, -1.0]])
        Hf = np.array([[dx / d, dy / d],
                       [-dy / d2, dx / d2]])
        Sf = Hf @ Pf @ Hf.T + Q_COV
        return zp, Hv, Hf, Sf

    def update_kf_with_cholesky(self, xf, Pf, v, Hf):
        PHt = Pf @ Hf.T
        S = Hf @ PHt + Q_COV
        S = (S + S.T) * 0.5
        try:
            SChol = np.linalg.cholesky(S).T
            SCholInv = np.linalg.inv(SChol)
            W1 = PHt @ SCholInv
            W = W1 @ SCholInv.T
            x = xf + W @ v
            P = Pf - W1 @ W1.T
            return x, P
        except np.linalg.linalg.LinAlgError:
            return xf, Pf

    def update_landmark(self, particle, z, lm_id):
        xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
        Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])
        zp, Hv, Hf, Sf = self.compute_jacobians(particle, xf, Pf)
        
        dz = np.array(z[0:2]).reshape(2, 1) - zp
        dz[1, 0] = pi_2_pi(dz[1, 0])
        
        xf, Pf = self.update_kf_with_cholesky(xf, Pf, dz, Hf)
        
        particle.lm[lm_id, :] = xf.T
        particle.lmP[2 * lm_id:2 * lm_id + 2, :] = Pf
        return particle

    def compute_weight(self, particle, z, lm_id):
        xf = np.array(particle.lm[lm_id, :]).reshape(2, 1)
        Pf = np.array(particle.lmP[2 * lm_id:2 * lm_id + 2])
        zp, Hv, Hf, Sf = self.compute_jacobians(particle, xf, Pf)
        
        dz = np.array(z[0:2]).reshape(2, 1) - zp
        dz[1, 0] = pi_2_pi(dz[1, 0])
        
        try:
            invS = np.linalg.inv(Sf)
            num = np.exp(-0.5 * dz.T @ invS @ dz)[0, 0]
            den = 2.0 * math.pi * math.sqrt(np.linalg.det(Sf))
            return num / den
        except np.linalg.linalg.LinAlgError:
            return 1.0

    def proposal_sampling(self, particle, z, lm_id):
        xf = particle.lm[lm_id, :].reshape(2, 1)
        Pf = particle.lmP[2 * lm_id:2 * lm_id + 2]
        
        x = np.array([particle.x, particle.y, particle.yaw]).reshape(3, 1)
        P = particle.P
        zp, Hv, Hf, Sf = self.compute_jacobians(particle, xf, Pf)
        
        try:
            Sfi = np.linalg.inv(Sf)
            dz = np.array(z[0:2]).reshape(2, 1) - zp
            dz[1, 0] = pi_2_pi(dz[1, 0])
            Pi = np.linalg.inv(P)
            
            proposal_P = np.linalg.inv(Hv.T @ Sfi @ Hv + Pi)
            x += proposal_P @ Hv.T @ Sfi @ dz
            
            # Sample new pose from proposal distribution
            sampled_x = np.random.multivariate_normal(x.flatten(), proposal_P).reshape(3, 1)
            particle.x = sampled_x[0, 0]
            particle.y = sampled_x[1, 0]
            particle.yaw = pi_2_pi(sampled_x[2, 0])
            particle.P = proposal_P
        except np.linalg.linalg.LinAlgError:
            pass
        return particle

    def search_correspond_landmark_id(self, particle, z):
        min_dist = []
        for i in range(particle.n_lm):
            xf = particle.lm[i, :].reshape(2, 1)
            Pf = particle.lmP[2 * i:2 * i + 2]
            zp, Hv, Hf, Sf = self.compute_jacobians(particle, xf, Pf)
            dz = np.array(z[0:2]).reshape(2, 1) - zp
            dz[1, 0] = pi_2_pi(dz[1, 0])
            try:
                dist = (dz.T @ np.linalg.inv(Sf) @ dz)[0, 0]
                min_dist.append(dist)
            except np.linalg.linalg.LinAlgError:
                min_dist.append(1e9)
        min_dist.append(M_DIST_TH)
        return min_dist.index(min(min_dist))

    def update_with_observation(self, landmarks):
        for z in landmarks:
            for ip in range(N_PARTICLE):
                min_id = self.search_correspond_landmark_id(self.particles[ip], z)
                
                # New landmark
                if min_id == self.particles[ip].n_lm:
                    if self.particles[ip].n_lm < MAX_LANDMARKS:
                        self.particles[ip] = self.add_new_lm(self.particles[ip], z, min_id)
                        self.particles[ip].n_lm += 1
                # Known landmark
                else:
                    w = self.compute_weight(self.particles[ip], z, min_id)
                    self.particles[ip].w *= w
                    self.particles[ip] = self.update_landmark(self.particles[ip], z, min_id)
                    self.particles[ip] = self.proposal_sampling(self.particles[ip], z, min_id)

    def resampling(self):
        # Normalize particle weights
        sum_w = sum([p.w for p in self.particles])
        if sum_w <= 0.0:
            for i in range(N_PARTICLE):
                self.particles[i].w = 1.0 / N_PARTICLE
        else:
            for i in range(N_PARTICLE):
                self.particles[i].w /= sum_w
                
        pw = np.array([p.w for p in self.particles])
        n_eff = 1.0 / (pw @ pw.T)
        
        if n_eff < NTH:
            w_cum = np.cumsum(pw)
            base = np.cumsum(pw * 0.0 + 1 / N_PARTICLE) - 1 / N_PARTICLE
            resample_id = base + np.random.rand(base.shape[0]) / N_PARTICLE
            
            indexes = []
            index = 0
            for ip in range(N_PARTICLE):
                while (index < w_cum.shape[0] - 1) and (resample_id[ip] > w_cum[index]):
                    index += 1
                indexes.append(index)
                
            new_particles = []
            for idx in indexes:
                p = Particle()
                p.x = self.particles[idx].x
                p.y = self.particles[idx].y
                p.yaw = self.particles[idx].yaw
                p.P = np.array(self.particles[idx].P)
                p.lm = np.array(self.particles[idx].lm)
                p.lmP = np.array(self.particles[idx].lmP)
                p.n_lm = self.particles[idx].n_lm
                p.w = 1.0 / N_PARTICLE
                new_particles.append(p)
            self.particles = new_particles

    def calc_final_state(self):
        # Calculate final state as weighted average of particles
        x_est = np.zeros(3)
        sum_w = sum([p.w for p in self.particles])
        if sum_w <= 0.0:
            sum_w = 1.0
            
        for p in self.particles:
            x_est[0] += (p.w / sum_w) * p.x
            x_est[1] += (p.w / sum_w) * p.y
            x_est[2] += (p.w / sum_w) * p.yaw
        x_est[2] = pi_2_pi(x_est[2])
        return x_est

    def publish_results(self, time_msg):
        x_est = self.calc_final_state()
        self.get_logger().info(f"[FastSLAM 2.0] Pose: x={x_est[0]:.2f}, y={x_est[1]:.2f}, yaw={x_est[2]:.2f}")
        
        # Publish pose
        pose = PoseStamped()
        pose.header.stamp = time_msg.to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x_est[0]
        pose.pose.position.y = x_est[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(x_est[2] / 2.0)
        pose.pose.orientation.w = math.cos(x_est[2] / 2.0)
        self.pose_pub.publish(pose)
        
        # Publish path
        self.path.header.stamp = time_msg.to_msg()
        self.path.poses.append(pose)
        if len(self.path.poses) > 500:
            self.path.poses.pop(0)
        self.path_pub.publish(self.path)
        
        # Publish TF (map -> odom_fastslam)
        t = TransformStamped()
        t.header.stamp = time_msg.to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom_fastslam'
        t.transform.translation.x = x_est[0]
        t.transform.translation.y = x_est[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(x_est[2] / 2.0)
        t.transform.rotation.w = math.cos(x_est[2] / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        # Publish landmarks from the best particle (highest weight)
        best_p_idx = int(np.argmax([p.w for p in self.particles]))
        best_p = self.particles[best_p_idx]
        
        marker_array = MarkerArray()
        # Delete old markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        self.marker_pub.publish(marker_array)
        
        marker_array = MarkerArray()
        for i in range(best_p.n_lm):
            lm_x = best_p.lm[i, 0]
            lm_y = best_p.lm[i, 1]
            
            marker = Marker()
            marker.header.stamp = time_msg.to_msg()
            marker.header.frame_id = 'map'
            marker.ns = 'fast_slam_landmarks'
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
            marker.color.g = 0.0
            marker.color.b = 1.0  # Colored blue for FastSLAM 2.0 landmarks
            marker.color.a = 0.8
            marker_array.markers.append(marker)
            
        if len(marker_array.markers) > 0:
            self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = FastSLAM2Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
