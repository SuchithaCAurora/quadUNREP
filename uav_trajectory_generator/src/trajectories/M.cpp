#include "trajectory_generator_ros2/trajectories/M.hpp"
#include <cmath>

namespace trajectory_generator {

M::M(double cx, double cy, double length, double width, double alt,
     std::vector<double> v_goals, double t_traj, double orientation, double dt)
    : cx_(cx), cy_(cy), length_(length), width_(width), alt_(alt), orientation_(orientation),
      v_goals_(v_goals), t_traj_(t_traj), Trajectory(dt) {}

M::~M() {}

void M::generateTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
                     std::unordered_map<int,std::string>& index_msgs,
                     const rclcpp::Clock::SharedPtr& clock)
{
    rclcpp::Time tstart = clock->now();

    // Define the 5 points of the M in local coordinates (centered at 0,0)
    std::vector<Eigen::Vector2d> base_points = {
        {-width_/2, -length_/2},
        {-width_/2,  length_/2},
        {     0.0,  -length_/2},
        { width_/2,  length_/2},
        { width_/2, -length_/2}
    };

    // Apply orientation and shift to center
    double c = std::cos(orientation_);
    double s = std::sin(orientation_);
    std::vector<Eigen::Vector2d> points(5);
    for (size_t i = 0; i < base_points.size(); ++i) {
        double x_new = c * base_points[i].x() - s * base_points[i].y() + cx_;
        double y_new = s * base_points[i].x() + c * base_points[i].y() + cy_;
        points[i].x() = x_new;
        points[i].y() = y_new;
    }

    double v_goal = v_goals_.empty() ? 1.0 : v_goals_[0];
    double dt = dt_;
    double t = 0.0;
    bool forward = true;

    while (t < t_traj_) {
        const std::vector<Eigen::Vector2d>& path = forward ? points : std::vector<Eigen::Vector2d>(points.rbegin(), points.rend());
        for (size_t seg = 0; seg < path.size() - 1 && t < t_traj_; ++seg) {
            Eigen::Vector2d start = path[seg];
            Eigen::Vector2d end = path[seg + 1];
            double heading = std::atan2(end.y() - start.y(), end.x() - start.x());
            double distance = (end - start).norm();
            int steps = std::ceil(distance / (v_goal * dt));
            for (int i = 0; i <= steps && t < t_traj_; ++i) {
                double frac = static_cast<double>(i) / steps;
                double x = start.x() + frac * (end.x() - start.x());
                double y = start.y() + frac * (end.y() - start.y());
                goals.push_back(createMGoal(x, y, v_goal, 0, heading));
                index_msgs[goals.size() - 1] = std::string("M traj: segment ") + std::to_string(seg) + (forward ? " fwd" : " rev");
                t += dt;
            }
        }
        forward = !forward; // Reverse direction for next lap
    }

    index_msgs[goals.size() - 1] = "M traj: completed";
    RCLCPP_INFO(logger_, "M traj generated in %f s, %lu goals", (clock->now() - tstart).seconds(), goals.size());
}

snapstack_msgs2::msg::Goal M::createMGoal(double x, double y, double v, double accel, double heading) const
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
    goal.psi = heading; // Always face direction of travel
    goal.dpsi = 0;
    goal.power = true;
    return goal;
}

void M::generateStopTraj(std::vector<snapstack_msgs2::msg::Goal>& goals,
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
    stop_msgs[0] = "M traj: pressed END, decelerating to 0 m/s";

    double decel = 1.0; // You can make this a parameter if desired
    while (v > 0) {
        v = std::max(v - decel * dt_, 0.0);
        stop_goals.push_back(createMGoal(last_goal.p.x, last_goal.p.y, v, -decel, heading));
    }
    stop_msgs[stop_goals.size() - 1] = "M traj: stopped";
    goals = std::move(stop_goals);
    index_msgs = std::move(stop_msgs);
    pub_index = 0;

    RCLCPP_INFO(logger_, "Time to calculate the braking traj (s): %f", (clock->now() - tstart).seconds());
    RCLCPP_INFO(logger_, "Goal vector size = %lu", goals.size());
}

bool M::trajectoryInsideBounds(double xmin, double xmax,
                              double ymin, double ymax,
                              double zmin, double zmax)
{
    // Check all 5 points of the M
    std::vector<Eigen::Vector3d> points = {
        Eigen::Vector3d(cx_ - width_/2, cy_ - length_/2, alt_),
        Eigen::Vector3d(cx_ - width_/2, cy_ + length_/2, alt_),
        Eigen::Vector3d(cx_,            cy_ - length_/2, alt_),
        Eigen::Vector3d(cx_ + width_/2, cy_ + length_/2, alt_),
        Eigen::Vector3d(cx_ + width_/2, cy_ - length_/2, alt_)
    };

    // Apply orientation to each point
    double c = std::cos(orientation_);
    double s = std::sin(orientation_);
    for (auto& pt : points) {
        double x_shift = pt.x() - cx_;
        double y_shift = pt.y() - cy_;
        double x_rot = c * x_shift - s * y_shift + cx_;
        double y_rot = s * x_shift + c * y_shift + cy_;
        pt.x() = x_rot;
        pt.y() = y_rot;
        if (!isPointInsideBounds(xmin, xmax, ymin, ymax, zmin, zmax, pt)) {
            return false;
        }
    }
    return true;
}

} // namespace trajectory_generator