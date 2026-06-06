#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

class WallExplorer : public rclcpp::Node {
public:
    WallExplorer() : Node("wall_explorer") {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallExplorer::scan_callback, this, std::placeholders::_1));
        
        // Control loop at 10Hz (scaled automatically under sim time)
        timer_ = this->create_timer(100ms, std::bind(&WallExplorer::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Wall Explorer Node Initialized.");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
    }

    void control_loop() {
        if (!latest_scan_) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for LiDAR scan data...");
            return;
        }

        // Lidar range index mapping: 0 to 359 degrees counter-clockwise
        // - Front: -20 to 20 degrees
        // - Right: -100 to -60 degrees (index 260 to 300)
        // - Left: 60 to 100 degrees
        
        double front_min = get_min_range(340, 20);
        double left_min = get_min_range(60, 100);
        double right_min = get_min_range(260, 300);

        double linear_x = 0.0;
        double angular_z = 0.0;

        // Target wall distance
        double target_dist = 0.50;

        if (front_min < 0.8) {
            // Obstacle in front: turn left quickly in place or with small forward speed
            linear_x = 0.05;
            angular_z = 0.8;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                 "Obstacle in front! Turning left. front: %.2f, right: %.2f", front_min, right_min);
        } else {
            // No front obstacle
            if (right_min > 1.2) {
                // No wall on the right: turn right to find a wall or follow a outer corner
                linear_x = 0.15;
                angular_z = -0.6;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "No wall on right. Turning right to find wall. right: %.2f", right_min);
            } else {
                // Wall detected on the right: follow it
                linear_x = 0.22;
                
                // Simple proportional control to maintain target distance
                double error = right_min - target_dist;
                angular_z = -1.5 * error;
                
                // Clamp angular speed
                if (angular_z > 0.6) angular_z = 0.6;
                if (angular_z < -0.6) angular_z = -0.6;
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "Following wall. right: %.2f, error: %.2f, angular: %.2f", right_min, error, angular_z);
            }
        }

        // Publish command velocity (TwistStamped)
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = this->now();
        twist_msg.header.frame_id = "base_link";
        twist_msg.twist.linear.x = linear_x;
        twist_msg.twist.angular.z = angular_z;
        cmd_pub_->publish(twist_msg);
    }

    double get_min_range(int start_idx, int end_idx) {
        if (!latest_scan_) return 0.0;
        
        double min_val = std::numeric_limits<double>::infinity();
        int num_ranges = latest_scan_->ranges.size();
        
        if (start_idx > end_idx) {
            // Wraps around 0 degrees
            for (int i = start_idx; i < num_ranges; ++i) {
                double r = latest_scan_->ranges[i];
                if (r > latest_scan_->range_min && r < latest_scan_->range_max && r < min_val) {
                    min_val = r;
                }
            }
            for (int i = 0; i <= end_idx; ++i) {
                double r = latest_scan_->ranges[i];
                if (r > latest_scan_->range_min && r < latest_scan_->range_max && r < min_val) {
                    min_val = r;
                }
            }
        } else {
            for (int i = start_idx; i <= end_idx; ++i) {
                if (i >= 0 && i < num_ranges) {
                    double r = latest_scan_->ranges[i];
                    if (r > latest_scan_->range_min && r < latest_scan_->range_max && r < min_val) {
                        min_val = r;
                    }
                }
            }
        }
        
        if (std::isinf(min_val)) {
            return latest_scan_->range_max;
        }
        return min_val;
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
