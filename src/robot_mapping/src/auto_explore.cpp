#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <queue>
#include <algorithm>
#include <limits>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

struct GridCell {
    int x;
    int y;
    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const GridCell& other) const {
        return !(*this == other);
    }
    bool operator<(const GridCell& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
};

struct BlacklistEntry {
    double x;
    double y;
    rclcpp::Time timestamp;
};

struct Centroid {
    double wx;
    double wy;
    size_t size;
};

struct ScoredFrontier {
    double score;
    double wx;
    double wy;
    double gx;
    double gy;
    size_t size;
};

using PQElement = std::pair<double, GridCell>;

class FrontierExplorer : public rclcpp::Node {
public:
    FrontierExplorer() : Node("frontier_explorer") {
        state_ = "SELECTING_GOAL";
        current_cleaning_index_ = 0;

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/explorer_path", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/frontier_markers", 10);

        // Subscriber
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&FrontierExplorer::map_callback, this, std::placeholders::_1));

        // Nav2 Action Client
        nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        // Control loop timer (1.0 second interval in ROS/sim time)
        timer_ = this->create_timer(1.0s, std::bind(&FrontierExplorer::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Advanced Frontier Explorer with Coverage Cleaning initialized (C++ version).");
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_map_ = msg;
    }

    bool get_robot_pose(double& rx, double& ry) {
        std::vector<std::string> frames = {"base_footprint", "base_link"};
        for (const auto& frame : frames) {
            try {
                auto trans = tf_buffer_->lookupTransform("map", frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
                rx = trans.transform.translation.x;
                ry = trans.transform.translation.y;
                return true;
            } catch (const tf2::TransformException& ex) {
                continue;
            }
        }
        return false;
    }

    bool is_blacklisted(double x, double y) {
        rclcpp::Time now = this->now();
        // Keep blacklist active for 300 seconds (per summary instructions)
        blacklist_.erase(
            std::remove_if(blacklist_.begin(), blacklist_.end(),
                           [&](const BlacklistEntry& b) {
                               return (now - b.timestamp).seconds() >= 300.0;
                           }),
            blacklist_.end());

        for (const auto& b : blacklist_) {
            if (std::hypot(x - b.x, y - b.y) < 0.6) {
                return true;
            }
        }
        return false;
    }

    void control_loop() {
        if (!latest_map_) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for map...");
            return;
        }

        double rx, ry;
        if (!get_robot_pose(rx, ry)) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for robot pose...");
            return;
        }

        // Record cleaned cell (0.25m grid)
        int cx = static_cast<int>(std::round(rx / 0.25));
        int cy = static_cast<int>(std::round(ry / 0.25));
        cleaned_set_.insert({cx, cy});
        publish_cleaned_markers();

        if (state_ == "SELECTING_GOAL") {
            select_and_send_goal(rx, ry);
        } else if (state_ == "NAVIGATING") {
            if (goal_start_time_) {
                double elapsed = (this->now() - *goal_start_time_).seconds();
                if (elapsed > 60.0) {
                    RCLCPP_WARN(this->get_logger(), "Navigation timeout (> 60s)! Cancelling goal...");
                    cancel_goal();
                }
            }
        } else if (state_ == "GENERATING_CLEANING_PATH") {
            generate_cleaning_path(rx, ry);
        } else if (state_ == "CLEANING") {
            if (goal_start_time_) {
                double elapsed = (this->now() - *goal_start_time_).seconds();
                if (elapsed > 45.0) {
                    RCLCPP_WARN(this->get_logger(), "Cleaning waypoint timeout! Skipping to next.");
                    cancel_goal();
                }
            }
        }
    }

    bool has_line_of_sight(GridCell p1, GridCell p2, int width, int height,
                           const std::vector<int8_t>& data, const std::vector<bool>& is_inflated) {
        int x1 = p1.x;
        int y1 = p1.y;
        int x2 = p2.x;
        int y2 = p2.y;

        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;

        while (true) {
            if (x1 < 0 || x1 >= width || y1 < 0 || y1 >= height) {
                return false;
            }
            int idx = y1 * width + x1;
            if (data[idx] > 50 || is_inflated[idx] || data[idx] == -1) {
                return false;
            }
            if (x1 == x2 && y1 == y2) {
                break;
            }
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x1 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y1 += sy;
            }
        }
        return true;
    }

    std::vector<GridCell> smooth_path(const std::vector<GridCell>& path, int width, int height,
                                     const std::vector<int8_t>& data, const std::vector<bool>& is_inflated) {
        if (path.size() <= 2) return path;
        std::vector<GridCell> smoothed;
        smoothed.push_back(path.front());
        size_t current = 0;
        while (current < path.size() - 1) {
            size_t next = current + 1;
            for (size_t i = path.size() - 1; i > current + 1; --i) {
                if (has_line_of_sight(path[current], path[i], width, height, data, is_inflated)) {
                    next = i;
                    break;
                }
            }
            smoothed.push_back(path[next]);
            current = next;
        }
        return smoothed;
    }

    double calculate_information_gain(GridCell centroid, int width, int height,
                                       const std::vector<int8_t>& data) {
        int radius_cells = 30; // ~1.5m at 0.05m resolution
        int start_x = std::max(0, centroid.x - radius_cells);
        int end_x = std::min(width - 1, centroid.x + radius_cells);
        int start_y = std::max(0, centroid.y - radius_cells);
        int end_y = std::min(height - 1, centroid.y + radius_cells);

        double unknown_count = 0;
        for (int y = start_y; y <= end_y; ++y) {
            for (int x = start_x; x <= end_x; ++x) {
                int idx = y * width + x;
                if (data[idx] == -1) {
                    unknown_count += 1.0;
                }
            }
        }
        return unknown_count;
    }

    std::vector<std::pair<double, double>> optimize_cleaning_path(const std::vector<std::pair<double, double>>& waypoints, double rx, double ry) {
        std::vector<std::pair<double, double>> optimized;
        std::vector<std::pair<double, double>> unvisited = waypoints;
        double cx = rx;
        double cy = ry;

        while (!unvisited.empty()) {
            size_t best_idx = 0;
            double min_dist = std::numeric_limits<double>::max();
            for (size_t i = 0; i < unvisited.size(); ++i) {
                double dist = std::hypot(unvisited[i].first - cx, unvisited[i].second - cy);
                if (dist < min_dist) {
                    min_dist = dist;
                    best_idx = i;
                }
            }
            optimized.push_back(unvisited[best_idx]);
            cx = unvisited[best_idx].first;
            cy = unvisited[best_idx].second;
            unvisited.erase(unvisited.begin() + best_idx);
        }
        return optimized;
    }

    std::vector<GridCell> astar_path(GridCell start, GridCell goal, int width, int height,
                                     const std::vector<int8_t>& data, const std::vector<bool>& is_inflated) {
        if (start == goal) {
            return {start};
        }

        auto heuristic = [&](const GridCell& p) {
            return std::hypot(p.x - goal.x, p.y - goal.y);
        };

        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> open_pq;
        std::vector<double> g_score(width * height, std::numeric_limits<double>::infinity());
        std::vector<int> came_from(width * height, -1);
        std::vector<bool> closed_set(width * height, false);

        int start_idx = start.y * width + start.x;
        g_score[start_idx] = 0.0;
        open_pq.push({heuristic(start), start});

        while (!open_pq.empty()) {
            auto top = open_pq.top();
            open_pq.pop();
            GridCell curr = top.second;

            int curr_idx = curr.y * width + curr.x;
            if (closed_set[curr_idx]) {
                continue;
            }
            closed_set[curr_idx] = true;

            if (curr == goal) {
                std::vector<GridCell> path;
                int idx = curr_idx;
                while (idx != -1) {
                    int cx = idx % width;
                    int cy = idx / width;
                    path.push_back({cx, cy});
                    idx = came_from[idx];
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = curr.x + dx;
                    int ny = curr.y + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int n_idx = ny * width + nx;
                        if (data[n_idx] >= 0 && data[n_idx] <= 50) {
                            double step_cost = std::hypot(dx, dy);
                            if (is_inflated[n_idx]) {
                                step_cost += 10.0;
                            }
                            double tentative_g = g_score[curr_idx] + step_cost;
                            if (tentative_g < g_score[n_idx]) {
                                g_score[n_idx] = tentative_g;
                                came_from[n_idx] = curr_idx;
                                open_pq.push({tentative_g + heuristic({nx, ny}), {nx, ny}});
                            }
                        }
                    }
                }
            }
        }

        return {};
    }

    void select_and_send_goal(double rx, double ry) {
        auto msg = latest_map_;
        int width = msg->info.width;
        int height = msg->info.height;
        double resolution = msg->info.resolution;
        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;
        const auto& data = msg->data;

        // 1. Obstacle Inflation
        std::vector<bool> is_inflated(width * height, false);
        int rcells = static_cast<int>(0.30 / resolution);
        std::vector<std::pair<int, int>> occupied_coords;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (data[y * width + x] > 50) {
                    occupied_coords.push_back({x, y});
                }
            }
        }

        for (const auto& coord : occupied_coords) {
            int ox = coord.first;
            int oy = coord.second;
            for (int dy = -rcells; dy <= rcells; ++dy) {
                int ny = oy + dy;
                if (ny < 0 || ny >= height) continue;
                for (int dx = -rcells; dx <= rcells; ++dx) {
                    int nx = ox + dx;
                    if (nx < 0 || nx >= width) continue;
                    if (dx * dx + dy * dy <= rcells * rcells) {
                        is_inflated[ny * width + nx] = true;
                    }
                }
            }
        }

        // 2. Frontier Detection
        std::vector<GridCell> frontiers;
        for (int y = 1; y < height - 1; ++y) {
            for (int x = 1; x < width - 1; ++x) {
                int idx = y * width + x;
                if (data[idx] == 0 && !is_inflated[idx]) {
                    bool has_unknown = false;
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (dx == 0 && dy == 0) continue;
                            int n_idx = (y + dy) * width + (x + dx);
                            if (data[n_idx] == -1) {
                                has_unknown = true;
                                break;
                            }
                        }
                        if (has_unknown) break;
                    }
                    if (has_unknown) {
                        frontiers.push_back({x, y});
                    }
                }
            }
        }

        // 3. Frontier Clustering
        std::vector<bool> is_frontier(width * height, false);
        for (const auto& f : frontiers) {
            is_frontier[f.y * width + f.x] = true;
        }

        std::vector<bool> visited(width * height, false);
        std::vector<std::vector<GridCell>> clusters;
        const size_t min_cluster_size = 10;

        for (const auto& f : frontiers) {
            int f_idx = f.y * width + f.x;
            if (visited[f_idx]) continue;

            std::vector<GridCell> cluster;
            std::vector<GridCell> queue;
            queue.push_back(f);
            visited[f_idx] = true;

            size_t head = 0;
            while (head < queue.size()) {
                GridCell curr = queue[head++];
                cluster.push_back(curr);

                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = curr.x + dx;
                        int ny = curr.y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            int n_idx = ny * width + nx;
                            if (is_frontier[n_idx] && !visited[n_idx]) {
                                visited[n_idx] = true;
                                queue.push_back({nx, ny});
                            }
                        }
                    }
                }
            }

            if (cluster.size() >= min_cluster_size) {
                clusters.push_back(cluster);
            }
        }

        // 4. Information Gain and Scoring
        std::vector<ScoredFrontier> scored_frontiers;
        std::vector<Centroid> centroids;

        for (const auto& cluster : clusters) {
            double sum_x = 0;
            double sum_y = 0;
            for (const auto& cell : cluster) {
                sum_x += cell.x;
                sum_y += cell.y;
            }
            double avg_x = sum_x / cluster.size();
            double avg_y = sum_y / cluster.size();

            double wx = avg_x * resolution + origin_x;
            double wy = avg_y * resolution + origin_y;

            centroids.push_back({wx, wy, cluster.size()});

            if (is_blacklisted(wx, wy)) {
                continue;
            }

            double dist = std::hypot(wx - rx, wy - ry);
            if (dist < 0.4) {
                continue;
            }

            // Calculate advanced information gain (unknown space around centroid)
            double info_gain = calculate_information_gain({static_cast<int>(avg_x), static_cast<int>(avg_y)}, width, height, data);

            // Advanced Score = (Information Gain * Cluster Size) / (Distance to robot + 0.5)
            double score = (info_gain * static_cast<double>(cluster.size())) / (dist + 0.5);
            scored_frontiers.push_back({score, wx, wy, avg_x, avg_y, cluster.size()});
        }

        publish_frontier_markers(centroids);

        if (scored_frontiers.empty()) {
            RCLCPP_INFO(this->get_logger(), "No valid frontiers found. Mapping complete! Transitioning to CLEANING mode.");
            if (goal_handle_) {
                nav_client_->async_cancel_goal(goal_handle_);
                goal_handle_ = nullptr;
            }
            state_ = "GENERATING_CLEANING_PATH";
            return;
        }

        std::sort(scored_frontiers.begin(), scored_frontiers.end(),
                  [](const ScoredFrontier& a, const ScoredFrontier& b) {
                      return a.score > b.score;
                  });

        // 5. A* Path Planning & Reachability Verification
        GridCell start_grid = {
            static_cast<int>((rx - origin_x) / resolution),
            static_cast<int>((ry - origin_y) / resolution)
        };

        std::vector<GridCell> best_path;
        std::pair<double, double> best_target_w = {0.0, 0.0};
        bool found_best = false;

        for (const auto& sf : scored_frontiers) {
            GridCell goal_grid = { static_cast<int>(sf.gx), static_cast<int>(sf.gy) };
            auto path = astar_path(start_grid, goal_grid, width, height, data, is_inflated);
            if (!path.empty()) {
                // Smooth the A* path using line-of-sight check for a clean navigation path
                best_path = smooth_path(path, width, height, data, is_inflated);
                best_target_w = {sf.wx, sf.wy};
                found_best = true;
                break;
            } else {
                RCLCPP_WARN(this->get_logger(), "Frontier at (%.2f, %.2f) unreachable. Blacklisting.", sf.wx, sf.wy);
                blacklist_.push_back({sf.wx, sf.wy, this->now()});
            }
        }

        if (!found_best) {
            RCLCPP_WARN(this->get_logger(), "All detected frontiers are unreachable. Transitioning to CLEANING mode.");
            if (goal_handle_) {
                nav_client_->async_cancel_goal(goal_handle_);
                goal_handle_ = nullptr;
            }
            state_ = "GENERATING_CLEANING_PATH";
            return;
        }

        publish_path(best_path, resolution, origin_x, origin_y);

        double target_x = best_target_w.first;
        double target_y = best_target_w.second;
        publish_target_marker(target_x, target_y);
        send_navigation_goal(target_x, target_y);
    }

    void generate_cleaning_path(double rx, double ry) {
        auto msg = latest_map_;
        int width = msg->info.width;
        int height = msg->info.height;
        double resolution = msg->info.resolution;
        double origin_x = msg->info.origin.position.x;
        double origin_y = msg->info.origin.position.y;
        const auto& data = msg->data;

        // 1. Obstacle Inflation
        std::vector<bool> is_inflated(width * height, false);
        int rcells = static_cast<int>(0.30 / resolution);
        std::vector<std::pair<int, int>> occupied_coords;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (data[y * width + x] > 50) {
                    occupied_coords.push_back({x, y});
                }
            }
        }

        for (const auto& coord : occupied_coords) {
            int ox = coord.first;
            int oy = coord.second;
            for (int dy = -rcells; dy <= rcells; ++dy) {
                int ny = oy + dy;
                if (ny < 0 || ny >= height) continue;
                for (int dx = -rcells; dx <= rcells; ++dx) {
                    int nx = ox + dx;
                    if (nx < 0 || nx >= width) continue;
                    if (dx * dx + dy * dy <= rcells * rcells) {
                        is_inflated[ny * width + nx] = true;
                    }
                }
            }
        }

        // 2. Find all reachable free cells using BFS from robot position
        int sx = static_cast<int>((rx - origin_x) / resolution);
        int sy = static_cast<int>((ry - origin_y) / resolution);

        std::vector<GridCell> reachable;
        std::vector<bool> visited(width * height, false);
        std::vector<GridCell> queue;

        if (sx >= 0 && sx < width && sy >= 0 && sy < height) {
            queue.push_back({sx, sy});
            visited[sy * width + sx] = true;
        }

        size_t head = 0;
        while (head < queue.size()) {
            GridCell curr = queue[head++];
            int idx = curr.y * width + curr.x;
            if (data[idx] == 0 && !is_inflated[idx]) {
                reachable.push_back(curr);

                for (const auto& dir : std::vector<std::pair<int, int>>{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}) {
                    int nx = curr.x + dir.first;
                    int ny = curr.y + dir.second;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                        int n_idx = ny * width + nx;
                        if (!visited[n_idx]) {
                            visited[n_idx] = true;
                            queue.push_back({nx, ny});
                        }
                    }
                }
            }
        }

        if (reachable.empty()) {
            RCLCPP_WARN(this->get_logger(), "No reachable free cells found to clean! Mission complete.");
            state_ = "DONE";
            return;
        }

        std::vector<bool> is_reachable(width * height, false);
        for (const auto& cell : reachable) {
            is_reachable[cell.y * width + cell.x] = true;
        }

        int min_x = reachable[0].x;
        int max_x = reachable[0].x;
        int min_y = reachable[0].y;
        int max_y = reachable[0].y;
        for (const auto& cell : reachable) {
            if (cell.x < min_x) min_x = cell.x;
            if (cell.x > max_x) max_x = cell.x;
            if (cell.y < min_y) min_y = cell.y;
            if (cell.y > max_y) max_y = cell.y;
        }

        int spacing = static_cast<int>(0.50 / resolution);
        if (spacing <= 0) spacing = 1;

        std::vector<std::pair<double, double>> waypoints_grid;
        std::vector<int> x_coords;
        for (int x = min_x; x <= max_x; x += spacing) {
            x_coords.push_back(x);
        }

        for (size_t i = 0; i < x_coords.size(); ++i) {
            int x = x_coords[i];
            std::vector<int> y_coords;
            for (int y = min_y; y <= max_y; y += spacing) {
                y_coords.push_back(y);
            }
            if (i % 2 == 1) {
                std::reverse(y_coords.begin(), y_coords.end());
            }

            for (int y : y_coords) {
                GridCell chosen_cell = {0, 0};
                bool found_reachable = false;
                for (int dy = -2; dy <= 2; ++dy) {
                    for (int dx = -2; dx <= 2; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            if (is_reachable[ny * width + nx]) {
                                chosen_cell = {nx, ny};
                                found_reachable = true;
                                break;
                            }
                        }
                    }
                    if (found_reachable) break;
                }

                if (found_reachable) {
                    double wx = chosen_cell.x * resolution + origin_x;
                    double wy = chosen_cell.y * resolution + origin_y;
                    waypoints_grid.push_back({wx, wy});
                }
            }
        }

        if (waypoints_grid.empty()) {
            RCLCPP_WARN(this->get_logger(), "No waypoints could be placed for cleaning! Mission complete.");
            state_ = "DONE";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Generated lawn mower cleaning path with %zu waypoints.", waypoints_grid.size());
        // Optimize path using Nearest Neighbor TSP solver to prevent arbitrary leaps
        cleaning_path_ = optimize_cleaning_path(waypoints_grid, rx, ry);
        current_cleaning_index_ = 0;
        state_ = "CLEANING";

        publish_cleaning_path();
        send_next_cleaning_goal();
    }

    void send_next_cleaning_goal() {
        if (current_cleaning_index_ < cleaning_path_.size()) {
            auto pt = cleaning_path_[current_cleaning_index_];
            double wx = pt.first;
            double wy = pt.second;
            RCLCPP_INFO(this->get_logger(), "Cleaning: target waypoint %zu/%zu at (%.2f, %.2f)",
                        current_cleaning_index_ + 1, cleaning_path_.size(), wx, wy);

            publish_target_marker(wx, wy);
            send_navigation_goal(wx, wy);
        }
    }

    void send_navigation_goal(double x, double y) {
        if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;

        double rx, ry;
        if (get_robot_pose(rx, ry)) {
            double heading = std::atan2(y - ry, x - rx);
            goal_msg.pose.pose.orientation.z = std::sin(heading / 2.0);
            goal_msg.pose.pose.orientation.w = std::cos(heading / 2.0);
        } else {
            goal_msg.pose.pose.orientation.w = 1.0;
        }

        RCLCPP_INFO(this->get_logger(), "Navigating to: (%.2f, %.2f)", x, y);
        current_target_ = {x, y};
        goal_start_time_ = this->now();

        if (state_ != "CLEANING" && state_ != "RETURNING_TO_ORIGIN") {
            state_ = "NAVIGATING";
        }

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&FrontierExplorer::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&FrontierExplorer::get_result_callback, this, std::placeholders::_1);

        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(), "Goal was rejected by Nav2!");
            if (state_ == "CLEANING") {
                current_cleaning_index_++;
                send_next_cleaning_goal();
            } else {
                state_ = "SELECTING_GOAL";
            }
            return;
        }

        goal_handle_ = goal_handle;
    }

    void get_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
        if (!goal_handle_ || goal_handle_->get_goal_id() != result.goal_id) {
            RCLCPP_INFO(this->get_logger(), "Ignoring result callback for stale or cancelled goal.");
            return;
        }

        auto status = result.code;

        if (state_ == "CLEANING") {
            if (status == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Cleaned waypoint %zu/%zu!",
                            current_cleaning_index_ + 1, cleaning_path_.size());
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to reach cleaning waypoint %zu. Skipping.",
                            current_cleaning_index_ + 1);
            }

            current_cleaning_index_++;
            goal_handle_ = nullptr;

            if (current_cleaning_index_ >= cleaning_path_.size()) {
                RCLCPP_INFO(this->get_logger(), "Lawn mower cleaning coverage complete! Returning to origin.");
                state_ = "RETURNING_TO_ORIGIN";
                send_navigation_goal(0.0, 0.0);
            } else {
                send_next_cleaning_goal();
            }
        } else if (state_ == "RETURNING_TO_ORIGIN") {
            RCLCPP_INFO(this->get_logger(), "Returned to origin. Mission accomplished!");
            state_ = "DONE";
            goal_handle_ = nullptr;
        } else {
            if (status == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Reached target successfully!");
            } else {
                RCLCPP_WARN(this->get_logger(), "Navigation failed!");
                if (current_target_) {
                    blacklist_.push_back({current_target_->first, current_target_->second, this->now()});
                }
            }

            state_ = "SELECTING_GOAL";
            goal_handle_ = nullptr;
            current_target_ = std::nullopt;
        }
    }

    void cancel_goal() {
        if (goal_handle_) {
            nav_client_->async_cancel_goal(goal_handle_);
            if (current_target_) {
                blacklist_.push_back({current_target_->first, current_target_->second, this->now()});
            }
        }

        goal_handle_ = nullptr; // Ignore result callback of cancelled goal

        if (state_ == "CLEANING") {
            current_cleaning_index_++;
            if (current_cleaning_index_ >= cleaning_path_.size()) {
                RCLCPP_INFO(this->get_logger(), "Lawn mower cleaning coverage complete! Returning to origin.");
                state_ = "RETURNING_TO_ORIGIN";
                send_navigation_goal(0.0, 0.0);
            } else {
                send_next_cleaning_goal();
            }
        } else if (state_ == "RETURNING_TO_ORIGIN") {
            state_ = "DONE";
        } else {
            state_ = "SELECTING_GOAL";
        }

        current_target_ = std::nullopt;
    }

    void publish_path(const std::vector<GridCell>& path, double resolution, double origin_x, double origin_y) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        for (const auto& cell : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = cell.x * resolution + origin_x;
            pose.pose.position.y = cell.y * resolution + origin_y;
            pose.pose.position.z = 0.05;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
    }

    void publish_cleaning_path() {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        for (const auto& pt : cleaning_path_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = pt.first;
            pose.pose.position.y = pt.second;
            pose.pose.position.z = 0.02;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        path_pub_->publish(path_msg);
    }

    void publish_frontier_markers(const std::vector<Centroid>& centroids) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "frontiers";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.15;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 0.8;

        for (const auto& c : centroids) {
            geometry_msgs::msg::Point p;
            p.x = c.wx;
            p.y = c.wy;
            p.z = 0.1;
            marker.points.push_back(p);
        }

        marker_pub_->publish(marker);
    }

    void publish_target_marker(double tx, double ty) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "target_frontier";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = tx;
        marker.pose.position.y = ty;
        marker.pose.position.z = 0.2;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_pub_->publish(marker);
    }

    void publish_cleaned_markers() {
        if (cleaned_set_.empty()) {
            return;
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "cleaned_zones";
        marker.id = 2;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.23;
        marker.scale.y = 0.23;
        marker.scale.z = 0.005;
        marker.color.r = 0.0;
        marker.color.g = 0.8;
        marker.color.b = 0.0;
        marker.color.a = 0.4;

        for (const auto& cell : cleaned_set_) {
            geometry_msgs::msg::Point p;
            p.x = cell.first * 0.25;
            p.y = cell.second * 0.25;
            p.z = 0.002;
            marker.points.push_back(p);
        }

        marker_pub_->publish(marker);
    }

    // Node state and publishers/subscribers
    std::string state_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    std::optional<rclcpp::Time> goal_start_time_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
    std::optional<std::pair<double, double>> current_target_;
    std::vector<BlacklistEntry> blacklist_;

    // Cleaning parameters
    std::vector<std::pair<double, double>> cleaning_path_;
    size_t current_cleaning_index_;
    std::set<std::pair<int, int>> cleaned_set_;

    // TF and communication
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierExplorer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
