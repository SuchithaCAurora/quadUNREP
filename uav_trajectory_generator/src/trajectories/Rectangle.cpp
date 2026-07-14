/**
 * @file Rectangle.cpp
 * @brief Rectangle trajectory class (like Square, but with two different side lengths)
 * @author Russell Perez
 * @date 2025-06-27
 */

#include "trajectory_generator_ros2/trajectories/Rectangle.hpp"
#include <cmath>

namespace trajectory_generator {

Rectangle::Rectangle(double alt, double side_a, double side_b, double cx, double cy, double orientation,
           std::vector<double> v_goals, double t_traj, double accel, double dt)
    : alt_(alt), side_a_(side_a), side_b_(side_b), cx_(cx), cy_(cy), orientation_(orientation),
      v_goals_(v_goals), t_traj_(t_traj), accel_(accel), Trajectory(dt) {}

Rectangle::~Rectangle() {}

void Rectangle::generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                             std::unordered_map<int,std::string>& index_msgs,
                             const rclcpp::Clock::SharedPtr& clock)
{
    rclcpp::Time tstart = clock->now();

    // Calculate the four corners of the rectangle (counter-clockwise, starting at top-left)
    double half_a = side_a_ / 2.0;
    double half_b = side_b_ / 2.0;
    std::vector<Eigen::Vector2d> corners(4);
    // Unrotated corners (relative to center)
    corners[0] = Eigen::Vector2d(-half_a,  half_b); // top-left
    corners[1] = Eigen::Vector2d( half_a,  half_b); // top-right
    corners[2] = Eigen::Vector2d( half_a, -half_b); // bottom-right
    corners[3] = Eigen::Vector2d(-half_a, -half_b); // bottom-left

    // Apply orientation (rotation) to each corner
    double c = std::cos(orientation_);
    double s = std::sin(orientation_);
    for (auto& pt : corners) {
        double x_new = c * pt.x() - s * pt.y();
        double y_new = s * pt.x() + c * pt.y();
        pt.x() = x_new + cx_;
        pt.y() = y_new + cy_;
    }

    // Use the first velocity goal (constant velocity)
    double v_goal = v_goals_.empty() ? 1.0 : v_goals_[0];
    double total_perimeter = 2 * (side_a_ + side_b_);
    double time_per_lap = total_perimeter / v_goal;
    int num_laps = std::ceil(t_traj_ / time_per_lap);
    double dt = dt_;

    // Start at the first corner
    int current_corner = 0;
    double x = corners[0].x();
    double y = corners[0].y();
    double heading = orientation_ + M_PI; // Facing right (along first side)
    double t = 0.0;

    goals.push_back(createRectangleGoal(x, y, v_goal, 0, heading));
    index_msgs[goals.size() - 1] = "Rectangle traj: starting at corner 0";

    // For each lap
    for (int lap = 0; lap < num_laps; ++lap) {
        for (int side = 0; side < 4; ++side) {
            int next_corner = (current_corner + 1) % 4;
            Eigen::Vector2d start = corners[current_corner];
            Eigen::Vector2d end = corners[next_corner];
            Eigen::Vector2d dir = (end - start).normalized();
            double side_length = (end - start).norm();
            double side_time = side_length / v_goal;
            int steps = std::ceil(side_time / dt);

            for (int step = 1; step <= steps && t < t_traj_; ++step) {
                double frac = static_cast<double>(step) / steps;
                double x_new = start.x() + frac * (end.x() - start.x());
                double y_new = start.y() + frac * (end.y() - start.y());
                double heading = std::atan2(end.y() - start.y(), end.x() - start.x());
                goals.push_back(createRectangleGoal(x_new, y_new, v_goal, 0, heading));
                index_msgs[goals.size() - 1] = "Rectangle traj: moving along side " + std::to_string(side);
                t += dt;
                if (t >= t_traj_) break;
            }
            current_corner = next_corner;
        }
    }

    index_msgs[goals.size() - 1] = "Rectangle traj: completed";
    RCLCPP_INFO(logger_, "Time to calculate the traj (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size = %lu", goals.size());
}

snapstack_msgs2::msg::Goal Rectangle::createRectangleGoal(double x, double y, double v, double accel, double heading) const {
    snapstack_msgs2::msg::Goal goal;
    goal.header.frame_id = "world";
    goal.p.x = x;
    goal.p.y = y;
    goal.p.z = alt_;
    goal.v.x = v * std::cos(heading);
    goal.v.y = v * std::sin(heading);
    goal.v.z = 0;
    goal.a.x = accel * std::cos(heading);
    goal.a.y = accel * std::sin(heading);
    goal.a.z = 0;
    goal.psi = heading;
    goal.dpsi = 0;
    goal.power = true;
    return goal;
}

void Rectangle::generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                                 std::unordered_map<int,std::string>& index_msgs,
                                 int& pub_index,
                                 const rclcpp::Clock::SharedPtr& clock) {
    rclcpp::Time tstart = clock->now();
    auto last_goal = goals[pub_index];
    double v = std::sqrt(std::pow(last_goal.v.x, 2) + std::pow(last_goal.v.y, 2));
    double heading = std::atan2(last_goal.v.y, last_goal.v.x);

    std::vector<snapstack_msgs2::msg::Goal> stop_goals;
    std::unordered_map<int, std::string> stop_msgs;
    stop_msgs[0] = "Rectangle traj: pressed END, decelerating to 0 m/s";

    while (v > 0) {
        v = std::max(v - accel_ * dt_, 0.0);
        stop_goals.push_back(createRectangleGoal(last_goal.p.x, last_goal.p.y, v, -accel_, heading));
    }
    stop_msgs[stop_goals.size() - 1] = "Rectangle traj: stopped";
    goals = std::move(stop_goals);
    index_msgs = std::move(stop_msgs);
    pub_index = 0;

    RCLCPP_INFO(logger_, "Time to calculate the braking traj (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size = %lu", goals.size());
}

bool Rectangle::trajectoryInsideBounds(double xmin, double xmax,
                                       double ymin, double ymax,
                                       double zmin, double zmax) {
    // Check all four corners
    double half_a = side_a_ / 2.0;
    double half_b = side_b_ / 2.0;
    double c = std::cos(orientation_);
    double s = std::sin(orientation_);
    std::vector<Eigen::Vector3d> corners(4);
    corners[0] = Eigen::Vector3d(cx_ + c * -half_a - s * half_b, cy_ + s * -half_a + c * half_b, alt_);
    corners[1] = Eigen::Vector3d(cx_ + c * half_a - s * half_b, cy_ + s * half_a + c * half_b, alt_);
    corners[2] = Eigen::Vector3d(cx_ + c * half_a - s * -half_b, cy_ + s * half_a + c * -half_b, alt_);
    corners[3] = Eigen::Vector3d(cx_ + c * -half_a - s * -half_b, cy_ + s * -half_a + c * -half_b, alt_);
    for (const auto& pt : corners) {
        if (!isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax, pt)) {
            return false;
        }
    }
    return true;
}

} // namespace trajectory_generator
