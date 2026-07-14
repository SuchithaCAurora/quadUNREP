/**
 * @file Reciprocating.cpp
 * @brief Reciprocating trajectory class (back-and-forth along a line with 180 deg yaw turns)
 * @author Russell Perez
 * @date 2025-06-27
 */

#include "trajectory_generator_ros2/trajectories/Reciprocating.hpp"
#include <cmath>

namespace trajectory_generator {

Reciprocating::Reciprocating(double alt, Eigen::Vector3d A, Eigen::Vector3d B,
                             std::vector<double> v_goals, double a1, double a3, double t_traj, double dt)
    : alt_(alt), A_(A), B_(B), v_goals_(v_goals), a1_(a1), a3_(a3), t_traj_(t_traj), Trajectory(dt)
{
    theta_fwd_ = std::atan2(B_.y() - A_.y(), B_.x() - A_.x());
    theta_rev_ = std::atan2(A_.y() - B_.y(), A_.x() - B_.x());
}

Reciprocating::~Reciprocating() {}

void Reciprocating::generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                                 std::unordered_map<int,std::string>& index_msgs,
                                 const rclcpp::Clock::SharedPtr& clock)
{
    rclcpp::Time tstart = clock->now();

    double v_goal = v_goals_.empty() ? 1.0 : v_goals_[0];
    double dt = dt_;
    double t = 0.0;
    bool forward = true;
    Eigen::Vector3d start = A_;
    Eigen::Vector3d end = B_;
    double heading = theta_fwd_;

    while (t < t_traj_) {
        // Move from start to end
        double distance = (end - start).head<2>().norm();
        int steps = std::ceil(distance / (v_goal * dt));
        for (int i = 0; i <= steps && t < t_traj_; ++i) {
            double frac = static_cast<double>(i) / steps;
            double x = start.x() + frac * (end.x() - start.x());
            double y = start.y() + frac * (end.y() - start.y());
            goals.push_back(createReciprocatingGoal(x, y, v_goal, 0, heading));
            index_msgs[goals.size() - 1] = forward ? "Reciprocating: forward" : "Reciprocating: reverse";
            t += dt;
        }
        // At the end, rotate yaw 180 degrees (heading flips)
        forward = !forward;
        std::swap(start, end);
        heading = forward ? theta_fwd_ : theta_rev_;
        // Optionally, add a goal at the endpoint with the new heading to ensure yaw change
        goals.push_back(createReciprocatingGoal(start.x(), start.y(), 0, 0, heading));
        index_msgs[goals.size() - 1] = "Reciprocating: yaw flip at endpoint";
        t += dt;
    }

    RCLCPP_INFO(logger_, "Reciprocating traj generated in %f s, %lu goals", (clock->now() - tstart).seconds(), goals.size());
}

snapstack_msgs2::msg::Goal Reciprocating::createReciprocatingGoal(double x, double y, double v, double accel, double heading) const
{
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

void Reciprocating::generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                                     std::unordered_map<int,std::string>& index_msgs,
                                     int& pub_index,
                                     const rclcpp::Clock::SharedPtr& clock)
{
    rclcpp::Time tstart = clock->now();
    auto last_goal = goals[pub_index];
    double v = std::sqrt(std::pow(last_goal.v.x, 2) + std::pow(last_goal.v.y, 2));
    double heading = std::atan2(last_goal.v.y, last_goal.v.x);

    std::vector<snapstack_msgs2::msg::Goal> stop_goals;
    std::unordered_map<int, std::string> stop_msgs;
    stop_msgs[0] = "Reciprocating traj: pressed END, decelerating to 0 m/s";

    while (v > 0) {
        v = std::max(v - a3_ * dt_, 0.0);
        stop_goals.push_back(createReciprocatingGoal(last_goal.p.x, last_goal.p.y, v, -a3_, heading));
    }
    stop_msgs[stop_goals.size() - 1] = "Reciprocating traj: stopped";
    goals = std::move(stop_goals);
    index_msgs = std::move(stop_msgs);
    pub_index = 0;

    RCLCPP_INFO(logger_, "Time to calculate the braking traj (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size = %lu", goals.size());
}

bool Reciprocating::trajectoryInsideBounds(double xmin, double xmax,
                                           double ymin, double ymax,
                                           double zmin, double zmax)
{
    // Check both endpoints
    std::vector<Eigen::Vector3d> points = {A_, B_};
    for (const auto& pt : points) {
        if (!isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax, pt)) {
            return false;
        }
    }
    return true;
}
} //namespace trajectory_generator