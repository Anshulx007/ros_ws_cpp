#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class WallExplorer : public rclcpp::Node {
public:
    enum class ExploreState {
        FOLLOW_WALL,
        EXPLORE_ROOM,
        BACKTRACK,
        EXITING_ROOM
    };

    WallExplorer() : Node("wall_explorer") {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallExplorer::scan_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&WallExplorer::odom_callback, this, std::placeholders::_1));
        
        // Control loop at 10Hz (scaled automatically under sim time)
        timer_ = this->create_timer(100ms, std::bind(&WallExplorer::control_loop, this));
        
        state_ = ExploreState::FOLLOW_WALL;
        state_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        sub_state_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        
        RCLCPP_INFO(this->get_logger(), "Wall Explorer Node Initialized with Backtracking State Machine.");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        current_yaw_ = 2.0 * atan2(qz, qw);
        
        // Initialize current quadrant on first odom message
        if (current_quadrant_ == 0) {
            current_quadrant_ = get_quadrant(current_x_, current_y_);
            RCLCPP_INFO(this->get_logger(), "Initial quadrant set to Room %d", current_quadrant_);
        }
        
        // Track distance travelled in room
        if (state_ == ExploreState::EXPLORE_ROOM) {
            if (last_x_ != 9999.0 && last_y_ != 9999.0) {
                double ds = hypot(current_x_ - last_x_, current_y_ - last_y_);
                room_explore_distance_ += ds;
            }
        }
        last_x_ = current_x_;
        last_y_ = current_y_;
    }

    int get_quadrant(double x, double y) {
        if (x >= 0.0 && y >= 0.0) return 1;
        if (x < 0.0 && y >= 0.0) return 2;
        if (x < 0.0 && y < 0.0) return 3;
        return 4; // x >= 0.0 && y < 0.0
    }

    double pi_2_pi(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void control_loop() {
        if (!latest_scan_ || current_quadrant_ == 0) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for LiDAR scan and Odometry data...");
            return;
        }

        // Initialize state_start_time_ once clock is valid
        if (state_start_time_.seconds() <= 0.0 && this->now().seconds() > 0.0) {
            state_start_time_ = this->now();
            sub_state_start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "State start time initialized to: %.2f", state_start_time_.seconds());
        }

        // Determine time in current state using simulation clock
        double time_in_state = 0.0;
        if (state_start_time_.seconds() > 0.0) {
            time_in_state = (this->now() - state_start_time_).seconds();
        }

        // 1. Doorway Crossing Detection (Crossing between Room Quadrants)
        int quad = get_quadrant(current_x_, current_y_);
        if (quad != current_quadrant_) {
            RCLCPP_INFO(this->get_logger(), "Doorway crossed: Room %d -> Room %d", current_quadrant_, quad);
            
            // Save the doorway point as the entrance
            entrance_x_ = current_x_;
            entrance_y_ = current_y_;
            has_entrance_ = true;
            
            room_explore_distance_ = 0.0;
            current_quadrant_ = quad;
            
            // Switch to room exploration mode
            state_ = ExploreState::EXPLORE_ROOM;
            state_start_time_ = this->now();
            time_in_state = 0.0;
        }

        // Lidar range index mapping: 0 to 359 degrees counter-clockwise
        double front_min = get_min_range(340, 20);
        double left_min = get_min_range(60, 100);
        double right_min = get_min_range(260, 300);

        double linear_x = 0.0;
        double angular_z = 0.0;
        double target_dist = 0.45;

        // State Machine behaviors
        switch (state_) {
            case ExploreState::FOLLOW_WALL: {
                // Loop-breaking wall follow behavior (switch wall-following side periodically)
                double sub_time = (this->now() - sub_state_start_time_).seconds();
                bool follow_right = true;
                bool driving_straight = false;

                if (sub_time < 12.0) {
                    follow_right = true;
                } else if (sub_time < 16.0) {
                    driving_straight = true;
                } else if (sub_time < 28.0) {
                    follow_right = false;
                } else if (sub_time < 30.5) {
                    // Turn random
                    linear_x = 0.0;
                    angular_z = 1.2;
                } else {
                    sub_state_start_time_ = this->now();
                }

                if (!driving_straight && sub_time <= 28.0) {
                    if (follow_right) {
                        if (front_min < 0.65) {
                            linear_x = 0.02;
                            angular_z = 1.5;
                        } else if (front_min < 1.0) {
                            linear_x = 0.15;
                            angular_z = 1.0;
                        } else {
                            if (right_min > 1.2) {
                                linear_x = 0.35;
                                angular_z = -0.8;
                            } else {
                                linear_x = 0.50;
                                double error = right_min - target_dist;
                                angular_z = -2.0 * error;
                                if (angular_z > 1.0) angular_z = 1.0;
                                if (angular_z < -1.0) angular_z = -1.0;
                            }
                        }
                    } else {
                        // Follow left
                        if (front_min < 0.65) {
                            linear_x = 0.02;
                            angular_z = -1.5;
                        } else if (front_min < 1.0) {
                            linear_x = 0.15;
                            angular_z = -1.0;
                        } else {
                            if (left_min > 1.2) {
                                linear_x = 0.35;
                                angular_z = 0.8;
                            } else {
                                linear_x = 0.50;
                                double error = left_min - target_dist;
                                angular_z = 2.0 * error;
                                if (angular_z > 1.0) angular_z = 1.0;
                                if (angular_z < -1.0) angular_z = -1.0;
                            }
                        }
                    }
                } else if (driving_straight) {
                    if (front_min < 0.80) {
                        linear_x = 0.02;
                        angular_z = 1.2;
                    } else {
                        linear_x = 0.45;
                        angular_z = 0.0;
                    }
                }

                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                     "[Follow Wall] sub_time: %.1f, right_min: %.2f, left_min: %.2f", sub_time, right_min, left_min);
                break;
            }

            case ExploreState::EXPLORE_ROOM: {
                // Same loop-breaking wall follow behavior, but checking for loop-backs (dead ends)
                double sub_time = (this->now() - sub_state_start_time_).seconds();
                bool follow_right = true;
                bool driving_straight = false;

                if (sub_time < 12.0) {
                    follow_right = true;
                } else if (sub_time < 16.0) {
                    driving_straight = true;
                } else if (sub_time < 28.0) {
                    follow_right = false;
                } else if (sub_time < 30.5) {
                    linear_x = 0.0;
                    angular_z = 1.2;
                } else {
                    sub_state_start_time_ = this->now();
                }

                if (!driving_straight && sub_time <= 28.0) {
                    if (follow_right) {
                        if (front_min < 0.65) {
                            linear_x = 0.02;
                            angular_z = 1.5;
                        } else if (front_min < 1.0) {
                            linear_x = 0.15;
                            angular_z = 1.0;
                        } else {
                            if (right_min > 1.2) {
                                linear_x = 0.35;
                                angular_z = -0.8;
                            } else {
                                linear_x = 0.50;
                                double error = right_min - target_dist;
                                angular_z = -2.0 * error;
                                if (angular_z > 1.0) angular_z = 1.0;
                                if (angular_z < -1.0) angular_z = -1.0;
                            }
                        }
                    } else {
                        if (front_min < 0.65) {
                            linear_x = 0.02;
                            angular_z = -1.5;
                        } else if (front_min < 1.0) {
                            linear_x = 0.15;
                            angular_z = -1.0;
                        } else {
                            if (left_min > 1.2) {
                                linear_x = 0.35;
                                angular_z = 0.8;
                            } else {
                                linear_x = 0.50;
                                double error = left_min - target_dist;
                                angular_z = 2.0 * error;
                                if (angular_z > 1.0) angular_z = 1.0;
                                if (angular_z < -1.0) angular_z = -1.0;
                            }
                        }
                    }
                } else if (driving_straight) {
                    if (front_min < 0.80) {
                        linear_x = 0.02;
                        angular_z = 1.2;
                    } else {
                        linear_x = 0.45;
                        angular_z = 0.0;
                    }
                }

                // Check for Dead-End / Loop-Back condition
                double dist_to_entrance = hypot(current_x_ - entrance_x_, current_y_ - entrance_y_);
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                     "[Explore Room] travelled: %.2fm, dist_to_entrance: %.2fm", 
                                     room_explore_distance_, dist_to_entrance);

                // If we have explored a significant distance (e.g. 10m) and returned to the entrance,
                // it's a loop with no other exit! Backtrack!
                if (has_entrance_ && room_explore_distance_ > 10.0 && dist_to_entrance < 0.90) {
                    state_ = ExploreState::BACKTRACK;
                    state_start_time_ = this->now();
                    RCLCPP_WARN(this->get_logger(), "Dead end detected in Room %d! Backtracking to entrance...", current_quadrant_);
                }
                break;
            }

            case ExploreState::BACKTRACK: {
                double dx = entrance_x_ - current_x_;
                double dy = entrance_y_ - current_y_;
                double dist = hypot(dx, dy);
                double target_angle = atan2(dy, dx);
                double angle_err = pi_2_pi(target_angle - current_yaw_);
                
                if (dist < 0.35) {
                    // Reached the entrance! Transition to exiting room
                    state_ = ExploreState::EXITING_ROOM;
                    state_start_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "Reached entrance! Exiting room...");
                    linear_x = 0.35;
                    angular_z = 0.0;
                } else {
                    if (std::abs(angle_err) > 0.4) {
                        // Turn in place to face entrance
                        linear_x = 0.0;
                        angular_z = (angle_err > 0.0) ? 1.2 : -1.2;
                    } else {
                        // Drive straight toward entrance
                        linear_x = 0.40;
                        angular_z = 1.8 * angle_err;
                    }
                }
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "[Backtrack] dist: %.2fm, angle_err: %.2f", dist, angle_err);
                break;
            }

            case ExploreState::EXITING_ROOM: {
                // Drive straight for 3.5 seconds to pass through the door
                linear_x = 0.35;
                angular_z = 0.0;
                if (time_in_state > 3.5) {
                    state_ = ExploreState::FOLLOW_WALL;
                    state_start_time_ = this->now();
                    sub_state_start_time_ = this->now();
                    has_entrance_ = false;
                    RCLCPP_INFO(this->get_logger(), "Backtrack exit complete, returning to FOLLOW_WALL.");
                }
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "[Exiting Room] time: %.1fs", time_in_state);
                break;
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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    
    ExploreState state_;
    rclcpp::Time state_start_time_;
    rclcpp::Time sub_state_start_time_;

    // Odometry tracking variables
    double current_x_ = 0.0;
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;
    
    double last_x_ = 9999.0;
    double last_y_ = 9999.0;

    // Room tracking variables
    int current_quadrant_ = 0;
    double entrance_x_ = 0.0;
    double entrance_y_ = 0.0;
    bool has_entrance_ = false;
    double room_explore_distance_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
