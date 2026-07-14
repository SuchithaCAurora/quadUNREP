/**
 * @file Bounce.cpp
 * @brief Bounce trajectory class (vertical up-and-down motion at a fixed x-y position)
 * @author Russell Perez
 * @date 2025-06-27
 */

#include "trajectory_generator_ros2/trajectories/Bounce.hpp"
#include <cmath>

namespace trajectory_generator {

Bounce::Bounce(double cx, double cy, double Az, double Bz,
               std::vector<double> v_goals, double t_traj, double orientation, double dt)
    : cx_(cx), cy_(cy), Az_(Az), Bz_(Bz), orientation_(orientation), v_goals_(v_goals), t_traj_(t_traj), Trajectory(dt) {}

Bounce::~Bounce() {}

void Bounce::generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                          std::unordered_map<int,std::string>& index_msgs,
                          const rclcpp::Clock::SharedPtr& clock)
{
    rclcpp::Time tstart = clock->now();

    double v_goal = v_goals_.empty() ? 1.0 : v_goals_[0];
    double dt = dt_;
    double t = 0.0;
    bool going_up = true;
    double z_start = Az_;
    double z_end = Bz_;
    double heading = orientation_; // Use specified orientation

    while (t < t_traj_) {
        double distance = std::abs(z_end - z_start);
        int steps = std::ceil(distance / (v_goal * dt));
        for (int i = 0; i <= steps && t < t_traj_; ++i) {
            double frac = static_cast<double>(i) / steps;
            double z = z_start + frac * (z_end - z_start);
            double vz = (z_end > z_start) ? v_goal : -v_goal;
            goals.push_back(createBounceGoal(cx_, cy_, z, vz, heading));
            index_msgs[goals.size() - 1] = going_up ? "Bounce: ascending" : "Bounce: descending";
            t += dt;
        }
        // Swap direction
        going_up = !going_up;
        std::swap(z_start, z_end);
    }

    index_msgs[goals.size() - 1] = "Bounce: completed";
    RCLCPP_INFO(logger_, "Bounce traj generated in %f s, %lu goals", (clock->now() - tstart).seconds(), goals.size());
}

snapstack_msgs2::msg::Goal Bounce::createBounceGoal(double x, double y, double z, double vz, double heading) const
{
    snapstack_msgs2::msg::Goal goal;
    goal.header.frame_id = "world";
    goal.p.x = x;
    goal.p.y = y;
    goal.p.z = z;
    goal.v.x = 0;
    goal.v.y = 0;
    goal.v.z = vz;
    goal.a.x = 0;
    goal.a.y = 0;
    goal.a.z = 0;
    goal.psi = heading;
    goal.dpsi = 0;
    goal.power = true;
    return goal;
}

void Bounce::generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                              std::unordered_map<int,std::string>& index_msgs,
                              int& pub_index,
                              const rclcpp::Clock::SharedPtr& clock)
{
    rclcpp::Time tstart = clock->now();
    auto last_goal = goals[pub_index];
    double vz = last_goal.v.z;
    double z = last_goal.p.z;
    double heading = last_goal.psi;

    std::vector<snapstack_msgs2::msg::Goal> stop_goals;
    std::unordered_map<int, std::string> stop_msgs;
    stop_msgs[0] = "Bounce traj: pressed END, decelerating to 0 m/s";

    // Decelerate to zero vertical velocity
    double decel = (vz > 0) ? -std::abs(vz) : std::abs(vz);
    while (std::abs(vz) > 0.01) {
        vz *= 0.8; // Simple exponential decay for smooth stop
        if (std::abs(vz) < 0.01) vz = 0.0;
        stop_goals.push_back(createBounceGoal(cx_, cy_, z, vz, heading));
    }
    stop_msgs[stop_goals.size() - 1] = "Bounce traj: stopped";
    goals = std::move(stop_goals);
    index_msgs = std::move(stop_msgs);
    pub_index = 0;

    RCLCPP_INFO(logger_, "Time to calculate the braking traj (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size = %lu", goals.size());
}

bool Bounce::trajectoryInsideBounds(double xmin, double xmax,
                                    double ymin, double ymax,
                                    double zmin, double zmax)
{
    // Check both endpoints
    std::vector<Eigen::Vector3d> points = {
        Eigen::Vector3d(cx_, cy_, Az_),
        Eigen::Vector3d(cx_, cy_, Bz_)
    };
    for (const auto& pt : points) {
        if (!isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax, pt)) {
            return false;
        }
    }
    return true;
}

}